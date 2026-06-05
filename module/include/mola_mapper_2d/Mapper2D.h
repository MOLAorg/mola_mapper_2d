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
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/graphs/CDirectedGraph.h>
#include <mrpt/graphs/dijkstra.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
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
#include <mola_kernel/Yaml.h>
#include <mola_kernel/interfaces/FrontEndBase.h>

// GTSAM:
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// STD
#include <mutex>
#include <optional>
#include <regex>
#include <set>

// Forward declarations
namespace nanogui
{
class Window;
class Label;
class CheckBox;
}  // namespace nanogui

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
*
*
* ## Flow diagram for incoming observations
* 
* ```
* External source -> onNewObservation(obs)
*                    |
*                    +--> [Odometry?] -> onOdometry() -> Update accum_odom_*
*                    |
*                    +--> [GNSS?] -> onGNSS() -> Store in last_gnss_ queue
*                    |
*                    +--> [LiDAR?] -> worker_lidar_.enqueue(onLidar)
*                                     |
*                                     v
*                                [Worker thread]
*                                     |
*                                     v
*                                processLidarScan()
*                                     |
*                                     v
*                                processSlamStep()
* ```
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
 * - Receives observations via onNewObservation()
 * - Publishes optimized maps via get_current_map()
 */
class Mapper2D : public mola::FrontEndBase
{
  DEFINE_MRPT_OBJECT(Mapper2D, mola)

public:
  Mapper2D();
  ~Mapper2D();

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
   * @warning Do not call onNewObservation() from another thread
   *          while holding a reference to the returned state object.
   */
  const SlamMapperState & get_state() const { return mapper_state_; }

  /// Non-const version for deserialization.
  SlamMapperState & get_state() { return mapper_state_; }

  /** Returns true if the worker thread is busy processing observations. */
  bool isBusy() const;

  /** Returns true if the SLAM system is active and processing observations. */
  bool isActive() const;

  /** Enable or disable SLAM processing. */
  void setActive(bool active);

  /** @} */

  // ===== Implementation of mola_kernel interfaces =====
  void onNewObservation(const CObservation::ConstPtr & o) override;
  void spinOnce() override;

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

  // ===== State flags (protected by state_flags_mtx_) =====
  struct StateFlags
  {
    bool initialized = false;
    bool fatal_error = false;
    bool active = true;
    uint32_t worker_tasks_lidar = 0;
  };
  StateFlags state_flags_;
  mutable std::mutex state_flags_mtx_;

  // ===== Odometry state (protected by odometry_mtx_) =====
  struct OdometryState
  {
    std::optional<mrpt::poses::CPose2D> last_absolute_odometry;
    std::optional<mrpt::Clock::time_point> last_odometry_timestamp;
  };
  OdometryState odometry_state_;
  mutable std::mutex odometry_mtx_;

  // ===== GNSS state (protected by gnss_mtx_) =====
  std::map<mrpt::Clock::time_point, std::shared_ptr<const mrpt::obs::CObservationGPS>> last_gnss_;
  mutable std::mutex gnss_mtx_;
  static constexpr size_t GNSS_QUEUE_MAX_SIZE = 100;

  // ===== Internal methods =====

  /** Worker thread callback for processing lidar observations */
  void onLidar(const CObservation::ConstPtr & o);

  /** Process a single lidar scan (called from worker thread) */
  void processLidarScan(const mrpt::obs::CObservation2DRangeScan::ConstPtr & scan);

  /** Handle odometry observation */
  void onOdometry(const mrpt::obs::CObservationOdometry::ConstPtr & o);

  /** Handle GNSS observation */
  void onGNSS(const mrpt::obs::CObservationGPS::ConstPtr & o);

  /** Main SLAM processing step */
  void processSlamStep(
    const mrpt::obs::CSensoryFrame & observations, const mrpt::poses::CPosePDFGaussian & odom_incr);

  RelocalizeCheckOutput check_needs_relocalization(
    const mrpt::obs::CSensoryFrame & observations, const mrpt::poses::CPosePDFGaussian & odom_incr);

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

  mrpt::viz::CSetOfObjects::Ptr build_visualization() const;

  void internal_delete_keyframe(KeyFrameID kf_id);

  /** Get closest GNSS observation to the given timestamp */
  mrpt::obs::CObservationGPS::ConstPtr getClosestGNSS(
    const mrpt::Clock::time_point & timestamp, double max_age_seconds = 1.0) const;

  // ===== Configuration parameters =====

  /** List of sensor labels or regex's for LiDAR observations */
  std::vector<std::regex> lidar_sensor_labels_;

  /** Sensor label regex for odometry observations */
  std::optional<std::regex> odometry_sensor_label_;

  /** Sensor label regex for GNSS observations */
  std::optional<std::regex> gnss_sensor_label_;

  /** Minimum time between scans to process */
  double min_time_between_scans_ = 0.01;  // [s]

  mrpt::containers::yaml viz_params_ = mrpt::containers::yaml::Map();

  SlamMapperState mapper_state_;
  mutable std::recursive_mutex state_mtx_;

  // Distance threshold for finding ICP edges
  double max_icp_search_distance_ = 3.0;

  // ICP quality thresholds
  double min_icp_quality_odometry_ = 0.30;
  double min_icp_quality_loop_closure_ = 0.60;
  double min_icp_quality_keyframe_replacement_ = 0.60;

  // Keyframe management
  double min_keyframe_age_for_replacement_ = 10.0;  // [s]
  double max_translation_between_keyframes_ = 0.5;  // [m]
  double max_rotation_between_keyframes_ = mrpt::DEG2RAD(20.0);
  double max_time_for_odometry_edge_ = 10.0;  // [s]

  // Loop closure detection
  uint32_t min_topological_distance_for_loop_closure_ = 20;

  // Localization frequency control
  double max_time_between_localizations_ = 5.0;         // [s]
  double max_translation_between_localizations_ = 0.5;  // [m]
  double max_rotation_between_localizations_ = mrpt::DEG2RAD(20.0);
  double max_lost_without_icp_ = 4.0;  // [m], threshold for forced KF insertion

  uint32_t max_icp_edges_per_localization_ = 10;

  // Edge noise parameters
  double odometry_edge_sigma_ = 0.10;
  double icp_edge_sigma_ = 0.10;
  double icp_edge_robust_parameter_ = 10.0;

  // Debug/output options
  int save_3d_scenes_decimation_ = 0;  // 0: disabled
  std::string save_3d_scenes_prefix_ = "./_debug_mapper2d";
  bool debug_print_factor_graphs_ = false;

  std::vector<std::string> sensor_labels_for_simplemap_;

  // Visualization cache
  mutable std::map<KeyFrameID, mrpt::viz::CSetOfObjects::Ptr> cached_viz_point_clouds_;

  // ===== Worker thread pool =====
  mrpt::WorkerThreadsPool worker_lidar_{
    1 /*num threads*/, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_lidar"};

  mutable std::mutex is_busy_mtx_;
  bool destructor_called_ = false;

  // Last processed timestamp to avoid duplicate processing
  std::optional<mrpt::Clock::time_point> last_lidar_timestamp_;

  // ===== GUI/Visualization =====

  struct StateUI
  {
    StateUI() = default;

    double timestampLastUpdateUI = 0;

    nanogui::Window * ui = nullptr;
    nanogui::Label * lbIcpQuality = nullptr;
    nanogui::Label * lbKeyframes = nullptr;
    nanogui::Label * lbLoopClosures = nullptr;
    nanogui::Label * lbTime = nullptr;
    nanogui::CheckBox * cbActive = nullptr;
    nanogui::CheckBox * cbMapping = nullptr;
  };

  StateUI gui_;
  mutable std::mutex state_gui_mtx_;

  // Visualization state
  mrpt::viz::CSetOfObjects::Ptr gl_vehicle_frame_;
  mrpt::viz::CSetOfObjects::Ptr gl_path_group_;
  mrpt::viz::CSetOfLines::Ptr gl_estimated_path_;
  int map_update_counter_ = std::numeric_limits<int>::max();
  bool local_map_needs_viz_update_ = true;
  std::optional<double> last_yaw_for_viz_camera_;
  double last_icp_quality_ = 0.0;

  // Visualization methods
  void updateVisualization();
  void updateVisualizationInitVehFrame();
  void updateVisualizationCurrentObservation(
    const mp2p_icp::metric_map_t & current_obs, std::vector<std::function<void()>> & update_tasks);
  void updateVisualizationLocalMap(std::vector<std::function<void()>> & update_tasks);
  void updateVisualizationPath(std::vector<std::function<void()>> & update_tasks);
  void updateVisualizationTextLabels();
  void internalBuildGUI();

  // ICP instances for odometry and loop closure
  mp2p_icp::ICP::Ptr icp_odometry_;
  mp2p_icp::Parameters icp_odometry_params_;

  mp2p_icp::ICP::Ptr icp_loop_closure_;
  mp2p_icp::Parameters icp_loop_closure_params_;
};

}  // namespace mola