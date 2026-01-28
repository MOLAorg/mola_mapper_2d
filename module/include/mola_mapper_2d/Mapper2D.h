/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   Mapper2D.h
 * @brief  Main 2D Lidar Graph-SLAM class.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 23, 2026
 */
#pragma once

// MRPT
#include <mrpt/containers/bimap.h>
#include <mrpt/graphs/CDirectedGraph.h>
#include <mrpt/graphs/dijkstra.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

// MP2P_ICP
#include <mp2p_icp/ICP.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>

// MOLA
#include <mola_kernel/interfaces/FrontEndBase.h>

// GTSAM:
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// STD
#include <mutex>
#include <optional>

namespace mola
{
struct DummyEdgeData
{
};

using KeyFrameID = mrpt::graphs::TNodeID;
using KeyFrameConnectivity = mrpt::graphs::CDirectedGraph<DummyEdgeData>;
using KeyFrameConnectivityDijkstra = mrpt::graphs::CDijkstra<KeyFrameConnectivity>;

/**
 * Encapsulates the state of a 2D SLAM session.
 * 
 * This class manages:
 * - The factor graph (poses and constraints)
 * - Keyframe observations and point clouds
 * - Odometry accumulation
 * - Keyframe connectivity for topological queries
 */
class SlamMapperState
{
public:
  SlamMapperState();

  /** @name Configuration from YAML files
	 *  @{ */
  mp2p_icp_filters::GeneratorSet pointcloud_generators;
  mp2p_icp_filters::FilterPipeline pointcloud_filters;
  /** @} */

  /** @name SLAM State Data
	 *  @{ */

  // PIMPL to gtsam types to avoid requiring gtsam headers in client code
  struct Impl
  {
    gtsam::Values graph_values;
    gtsam::NonlinearFactorGraph graph_factors;
  };
  Impl gtsam_data;

  // Mapping: timestamp <-> keyframe ID
  mrpt::containers::bimap<mrpt::Clock::time_point, KeyFrameID> time_to_kf_id;

  // Raw observations for each keyframe
  std::map<KeyFrameID, mrpt::obs::CSensoryFrame> keyframe_observations;

  // Last estimated robot localization (may come from optimized pose, odometry, or external input)
  mrpt::poses::CPose3D last_localization;
  std::optional<mrpt::Clock::time_point> last_localization_time;

  // Accumulated odometry since last keyframe insertion
  mrpt::poses::CPosePDFGaussian accum_odom_since_last_kf{
    mrpt::poses::CPose2D::Identity(), mrpt::math::CMatrixDouble33::Zero()};

  // Accumulated odometry since last localization (3D pose)
  mrpt::poses::CPose3D accum_odom_since_last_localization;

private:
  // Cached metric maps (3D point clouds) for keyframes
  mutable std::map<KeyFrameID, mp2p_icp::metric_map_t::Ptr> cached_keyframe_maps;

  // Connectivity graph for topological radius queries
  KeyFrameConnectivity kf_connectivity;

  // Cache for nearby keyframes queries
  mutable std::map<KeyFrameID, std::set<KeyFrameID>> cached_nearby_kfs;

public:
  /** @} */

  /** @name Methods
	 *  @{ */

  void clear() { *this = SlamMapperState(); }
  bool empty() const { return time_to_kf_id.empty(); }

  void serialize_to(mrpt::serialization::CArchive & out) const;
  void serialize_from(mrpt::serialization::CArchive & in);

  KeyFrameID last_kf_id() const
  {
    ASSERT_(!time_to_kf_id.empty());
    return time_to_kf_id.getInverseMap().rbegin()->first;
  }

  KeyFrameID generate_new_kf_id() const { return time_to_kf_id.empty() ? 0 : last_kf_id() + 1; }

  void delete_keyframe(KeyFrameID kf_id);

  mrpt::maps::CSimpleMap as_simple_map() const;

  mrpt::maps::CSimpleMap as_simple_map_around_pose(
    const mrpt::poses::CPose3D & pose, double distance_meters) const;

  // Get/create point cloud for a keyframe
  const mp2p_icp::metric_map_t::Ptr & get_keyframe_metric_map(KeyFrameID kf_id) const;

  void set_keyframe_metric_map(KeyFrameID kf_id, const mp2p_icp::metric_map_t::Ptr & map)
  {
    cached_keyframe_maps[kf_id] = map;
  }

  mp2p_icp::metric_map_t::Ptr build_metric_map_from_observations(
    const mrpt::obs::CSensoryFrame & observations) const;

  const KeyFrameConnectivity & get_kf_connectivity() const { return kf_connectivity; }

  void add_kf_connectivity(KeyFrameID id1, KeyFrameID id2);

  void add_kf_connectivity_from_factor_graph(const gtsam::NonlinearFactorGraph & fg);

  // Returns all keyframes within a topological distance from a given keyframe
  std::set<KeyFrameID> get_keyframes_in_topological_radius(
    KeyFrameID id, size_t max_topological_distance = 0) const;

  /** @} */
};

/**
 * 2D Lidar Graph-SLAM system.
 * 
 * This is a backend SLAM system that:
 * - Processes 2D/3D lidar observations
 * - Maintains a pose graph with SE(3) poses
 * - Performs loop closure detection
 * - Optimizes the graph using GTSAM
 * 
 * Integration with MOLA:
 * - Can be used as a backend in a MOLA-based pipeline
 * - Receives observations via processActionObservation()
 * - Publishes optimized maps via getCurrentBestMap()
 */
class Mapper2D : public mola::FrontEndBase
{
public:
  Mapper2D();
  ~Mapper2D() = default;

  // Prevent copying and moving
  Mapper2D(const Mapper2D &) = delete;
  Mapper2D & operator=(const Mapper2D &) = delete;
  Mapper2D(Mapper2D &&) = delete;
  Mapper2D & operator=(Mapper2D &&) = delete;

  /** @name Main API
	 * @{ */

protected:
  /**
	 * Initialize from YAML configuration file.
	 * Expected structure: see package example YAML files.
	 */
  void initialize_frontend(const Yaml & cfg) override;

public:
  /**
	 * Main entry point for SLAM processing.
	 * 
	 * Processes an action-observation pair:
	 * @param action Robot movement (odometry)
	 * @param observations Sensor observations (lidar, etc.)
	 */
  void process_action_observation(
    const mrpt::obs::CActionCollection & action, const mrpt::obs::CSensoryFrame & observations);

  /**
	 * Get the current best estimate of the robot pose.
	 */
  mrpt::poses::CPose3D get_current_pose() const;

  /**
	 * Get the current best estimated map.
	 */
  mrpt::maps::CSimpleMap get_current_map() const;

  /**
	 * Resume a SLAM session from a saved state.
	 * 
	 * Call this after deserializing a state for session resumption.
	 * The pose should come from external localization (e.g., particle filter).
	 * Small pose errors are tolerated and corrected after robot movement resumes.
	 * 
	 * @param current_pose Initial pose estimate for resumption
	 */
  void resume_session(const mrpt::poses::CPose3D & current_pose);

  /**
	 * Access the complete SLAM state for serialization.
	 * 
	 * @warning Do not call process_action_observation() from another thread
	 *          while holding a reference to the returned state object.
	 */
  const SlamMapperState & get_state() const { return mapper_state; }

  /// Non-const version for deserialization.
  SlamMapperState & get_state() { return mapper_state; }

  /** @} */

private:
  // ===== Internal structures =====

  struct RelocalizeCheckOutput
  {
    mp2p_icp::metric_map_t::Ptr current_frame_map = mp2p_icp::metric_map_t::Create();
    mrpt::Clock::time_point current_frame_timestamp;
    bool should_relocalize = false;
  };

  struct NearbyKeyFramesOutput
  {
    std::map<double, KeyFrameID> distance_to_kf_ids;  // sorted by distance
    std::set<KeyFrameID> kfs_in_topological_ball;
  };

  struct ICPEdgesOutput
  {
    std::map<KeyFrameID, mp2p_icp::Results> icp_results;
    mp2p_icp::metric_map_t::Ptr current_observations;
    mrpt::Clock::time_point current_observations_timestamp;
  };

  struct LocalizationOutput
  {
    gtsam::NonlinearFactorGraph factor_graph_edges;
    KeyFrameID tentative_new_kf_id;
  };

  // ===== Internal methods =====

  RelocalizeCheckOutput check_needs_relocalization(
    const mrpt::obs::CActionCollection & action, const mrpt::obs::CSensoryFrame & observations);

  NearbyKeyFramesOutput find_nearby_keyframes(
    const std::optional<size_t> & max_frames = std::nullopt) const;

  ICPEdgesOutput run_icp_on_nearby_keyframes(
    const NearbyKeyFramesOutput & nearby_kfs, const RelocalizeCheckOutput & relocalize_out) const;

  LocalizationOutput run_localization_step(const ICPEdgesOutput & icp_edges);

  void insert_new_keyframe_and_odometry_edge(
    const ICPEdgesOutput & icp_edges_out, const mrpt::obs::CSensoryFrame & observations);

  void add_icp_edges_to_graph(const LocalizationOutput & loc_out);

  bool try_replace_old_keyframes(
    const LocalizationOutput & loc_out, const ICPEdgesOutput & icp_edges,
    const mrpt::obs::CSensoryFrame & observations);

  void optimize_pose_graph();

  void save_debug_visualization_if_enabled();

  void save_state_to_3d_scene(const std::string & filename) const;

  mrpt::opengl::CSetOfObjects::Ptr build_visualization() const;

  void internal_delete_keyframe(KeyFrameID kf_id);

  // ===== Configuration parameters =====
  mrpt::containers::yaml viz_params;

  SlamMapperState mapper_state;
  mutable std::mutex state_mutex;

  mrpt::system::CTimeLogger profiler;

  // Distance threshold for finding ICP edges
  double max_icp_search_distance = 3.0;

  // ICP quality thresholds
  double min_icp_quality_odometry = 0.30;
  double min_icp_quality_loop_closure = 0.60;
  double min_icp_quality_keyframe_replacement = 0.60;

  // Keyframe management
  double min_keyframe_age_for_replacement = 10.0;  // [s]
  double max_translation_between_keyframes = 0.5;  // [m]
  double max_rotation_between_keyframes = mrpt::DEG2RAD(20.0);
  double max_time_for_odometry_edge = 10.0;  // [s]

  // Loop closure detection
  uint32_t min_topological_distance_for_loop_closure = 20;

  // Localization frequency control
  double max_time_between_localizations = 5.0;         // [s]
  double max_translation_between_localizations = 0.5;  // [m]
  double max_rotation_between_localizations = mrpt::DEG2RAD(20.0);
  double max_lost_without_icp = 4.0;  // [m], threshold for forced KF insertion

  uint32_t max_icp_edges_per_localization = 10;

  // Edge noise parameters
  double odometry_edge_sigma = 0.10;
  double icp_edge_sigma = 0.10;
  double icp_edge_robust_parameter = 10.0;

  // Debug/output options
  int save_3d_scenes_decimation = 0;  // 0: disabled
  std::string save_3d_scenes_prefix = "./_debug_mapper2d";
  bool debug_print_factor_graphs = false;

  std::vector<std::string> sensor_labels_for_simplemap;

  // Visualization cache
  mutable std::map<KeyFrameID, mrpt::opengl::CSetOfObjects::Ptr> cached_viz_point_clouds;

  // ICP instances for odometry and loop closure
  mp2p_icp::ICP::Ptr icp_odometry;
  mp2p_icp::Parameters icp_odometry_params;

  mp2p_icp::ICP::Ptr icp_loop_closure;
  mp2p_icp::Parameters icp_loop_closure_params;
};

}  // namespace mola