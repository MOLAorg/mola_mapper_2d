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
 * @file   Mapper2D.cpp
 * @brief  Main 2D Lidar Graph-SLAM class.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 23, 2026
 */

// This package:
#include <mola_mapper_2d/Mapper2D.h>

// MRPT:
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/stock_objects.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/serialization/bimap_serialization.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// MOLA / MP2P_ICP
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>

// Other libs:
#include <gtsam2mrpt_serial/serialize.h>

// STD
#include <random>

namespace mola
{
// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(Mapper2D, FrontEndBase, mola)

// ============================================================================
// SlamMapperState Implementation
// ============================================================================

SlamMapperState::SlamMapperState()  // gtsam_data(mrpt::make_impl<SlamMapperState::Impl>()) {}
  = default;

void SlamMapperState::delete_keyframe(KeyFrameID kf_id)
{
  using gtsam::symbol_shorthand::X;

  time_to_kf_id.erase_by_value(kf_id);
  gtsam_data.graph_values.erase(X(kf_id));
  keyframe_observations.erase(kf_id);
  cached_keyframe_maps.erase(kf_id);
}

mrpt::maps::CSimpleMap SlamMapperState::as_simple_map() const
{
  using gtsam::symbol_shorthand::X;

  const auto pose_cov = mrpt::math::CMatrixDouble66::Identity();
  mrpt::maps::CSimpleMap sm;

  for (const auto & kv : gtsam_data.graph_values) {
    const auto key = kv.key;
    const auto pose =
      mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(kv.value.cast<gtsam::Pose3>()));

    const auto s = gtsam::Symbol(key);
    ASSERT_EQUAL_(s.chr(), 'x');

    auto pose_pdf = mrpt::poses::CPose3DPDFGaussian::Create(pose, pose_cov);
    auto sf = mrpt::obs::CSensoryFrame::Create();
    (*sf) += keyframe_observations.at(s.index());

    sm.insert(pose_pdf, sf);
  }

  return sm;
}

mrpt::maps::CSimpleMap SlamMapperState::as_simple_map_around_pose(
  const mrpt::poses::CPose3D & pose, double distance_meters) const
{
  const auto sm_all = as_simple_map();
  mrpt::maps::CSimpleMap sm;

  for (const auto & e : sm_all) {
    const double d = e.pose->getMeanVal().distanceTo(pose);
    if (d > distance_meters) {
      continue;
    }
    sm.insert(e.pose, e.sf);
  }

  return sm;
}

const mp2p_icp::metric_map_t::Ptr & SlamMapperState::get_keyframe_metric_map(KeyFrameID kf_id) const
{
  if (auto it = cached_keyframe_maps.find(kf_id); it != cached_keyframe_maps.end()) {
    return it->second;
  }

  const auto & observations = keyframe_observations.at(kf_id);
  auto & pc_ptr = cached_keyframe_maps[kf_id];
  pc_ptr = build_metric_map_from_observations(observations);

  return pc_ptr;
}

mp2p_icp::metric_map_t::Ptr SlamMapperState::build_metric_map_from_observations(
  const mrpt::obs::CSensoryFrame & observations) const
{
  auto pc_ptr = mp2p_icp::metric_map_t::Create();
  auto & pc = *pc_ptr;

  mp2p_icp_filters::apply_generators(pointcloud_generators, observations, pc);
  mp2p_icp_filters::apply_filter_pipeline(pointcloud_filters, pc);

  return pc_ptr;
}

std::set<KeyFrameID> SlamMapperState::get_keyframes_in_topological_radius(
  KeyFrameID id, size_t max_topological_distance) const
{
  if (const auto it = cached_nearby_kfs.find(id); it != cached_nearby_kfs.end()) {
    return it->second;
  }

  std::set<KeyFrameID> nodes;

  if (kf_connectivity.edges.empty()) {
    return {};
  }

  const KeyFrameConnectivityDijkstra dijkstra(
    kf_connectivity, id, {}, {}, max_topological_distance);

  const auto & tree = dijkstra.getTreeGraph();
  for (const auto & edges : tree.edges_to_children) {
    nodes.insert(edges.first);
    for (const auto & edge : edges.second) {
      nodes.insert(edge.id);
    }
  }

  cached_nearby_kfs[id] = nodes;
  return nodes;
}

void SlamMapperState::add_kf_connectivity(KeyFrameID id1, KeyFrameID id2)
{
  if (!kf_connectivity.edgeExists(id1, id2)) {
    cached_nearby_kfs.clear();
    kf_connectivity.insertEdge(id1, id2, {});
  }
}

void SlamMapperState::add_kf_connectivity_from_factor_graph(const gtsam::NonlinearFactorGraph & fg)
{
  for (const auto & f : fg) {
    const auto * between_factor = dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3> *>(f.get());

    if (between_factor == nullptr) {
      continue;
    }

    const auto key1 = gtsam::Symbol(between_factor->key1());
    const auto key2 = gtsam::Symbol(between_factor->key2());
    ASSERT_(key1.chr() == 'x');
    ASSERT_(key2.chr() == 'x');

    add_kf_connectivity(key1.index(), key2.index());
  }
}

void SlamMapperState::serialize_to(mrpt::serialization::CArchive & out) const
{
  using namespace gtsam2mrpt_serial;

  const uint8_t SERIALIZATION_VERSION = 0;
  out << SERIALIZATION_VERSION;
  out << time_to_kf_id << keyframe_observations << accum_odom_since_last_kf;
  out << gtsam_data.graph_factors << gtsam_data.graph_values;
}

void SlamMapperState::serialize_from(mrpt::serialization::CArchive & in)
{
  using namespace gtsam2mrpt_serial;

  auto backup_pc_gens = pointcloud_generators;
  auto backup_pc_filters = pointcloud_filters;

  this->clear();

  const auto version = in.ReadAs<uint8_t>();

  switch (version) {
    case 0:
      in >> time_to_kf_id >> keyframe_observations;
      in >> accum_odom_since_last_kf;
      in >> gtsam_data.graph_factors >> gtsam_data.graph_values;
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }

  pointcloud_generators = std::move(backup_pc_gens);
  pointcloud_filters = std::move(backup_pc_filters);

  this->add_kf_connectivity_from_factor_graph(gtsam_data.graph_factors);
}

// ============================================================================
// Mapper2D Implementation
// ============================================================================

Mapper2D::Mapper2D() { COutputLogger::setLoggerName("Mapper2D"); }

Mapper2D::~Mapper2D()
{
  using namespace std::chrono_literals;

  try  // a dtor should never throw
  {
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      destructor_called_ = true;
    }

    while (isBusy()) {
      MRPT_LOG_THROTTLE_WARN(
        2.0, "Destructor: waiting for remaining tasks on the worker threads...");
      std::this_thread::sleep_for(100ms);
    }
    worker_lidar_.clear();

  } catch (const std::exception & e) {
    std::cerr << "[~Mapper2D] Exception: " << e.what();
  }
}

void Mapper2D::initialize_frontend(const mola::Yaml & cfg)
{
  MRPT_START

  YAML_LOAD_MEMBER_REQ(max_icp_search_distance, double);
  YAML_LOAD_MEMBER_REQ(min_icp_quality_odometry, double);
  YAML_LOAD_MEMBER_REQ(min_icp_quality_loop_closure, double);
  YAML_LOAD_MEMBER_REQ(min_icp_quality_keyframe_replacement, double);
  YAML_LOAD_MEMBER_REQ(min_keyframe_age_for_replacement, double);
  YAML_LOAD_MEMBER_REQ(min_topological_distance_for_loop_closure, uint32_t);
  YAML_LOAD_MEMBER_REQ(max_lost_without_icp, double);
  YAML_LOAD_MEMBER_REQ(max_time_between_localizations, double);
  YAML_LOAD_MEMBER_REQ(max_translation_between_localizations, double);
  YAML_LOAD_MEMBER_REQ(max_rotation_between_localizations, double);
  YAML_LOAD_MEMBER_OPT(max_icp_edges_per_localization, uint32_t);
  YAML_LOAD_MEMBER_REQ(max_translation_between_keyframes, double);
  YAML_LOAD_MEMBER_REQ(max_rotation_between_keyframes, double);
  YAML_LOAD_MEMBER_REQ(max_time_for_odometry_edge, double);
  YAML_LOAD_MEMBER_OPT(save_3d_scenes_decimation, bool);
  YAML_LOAD_MEMBER_OPT(save_3d_scenes_prefix, std::string);
  YAML_LOAD_MEMBER_OPT(debug_print_factor_graphs, bool);
  YAML_LOAD_MEMBER_REQ(odometry_edge_sigma, double);
  YAML_LOAD_MEMBER_REQ(icp_edge_sigma, double);
  YAML_LOAD_MEMBER_REQ(icp_edge_robust_parameter, double);

  // Load visualization parameters
  if (cfg.has("visualization")) {
    viz_params_ = mrpt::containers::yaml(cfg["visualization"]);
  }

  sensor_labels_for_simplemap_ = cfg["sensor_labels_for_simplemap"].toStdVector<std::string>();

  {
    MRPT_LOG_INFO("Sensor labels for simplemap:");
    for (const auto & s : sensor_labels_for_simplemap_) {
      MRPT_LOG_INFO_STREAM("  - '" << s << "'");
    }
  }

  // Odometry ICP:
  {
    const auto cfg_icp = cfg["icp-lidar-odometry"];
    const auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(cfg_icp);

    icp_odometry_ = icp;
    icp_odometry_params_ = params;

    if (cfg_icp.has("filters")) {
      mapper_state_.pointcloud_filters =
        mp2p_icp_filters::filter_pipeline_from_yaml(cfg_icp["filters"]);
    } else {
      MRPT_LOG_WARN("No 'filters' section in YAML config.");
    }

    if (cfg_icp.has("generators")) {
      mapper_state_.pointcloud_generators =
        mp2p_icp_filters::generators_from_yaml(cfg_icp["generators"]);
    } else {
      MRPT_LOG_WARN("No 'generators' section in YAML config, using default.");
      mapper_state_.pointcloud_generators.emplace_back(
        std::make_shared<mp2p_icp_filters::Generator>());
    }
  }

  // Loop closure ICP:
  {
    const auto cfg_icp = cfg["icp-lidar-loop-closure"];
    const auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(cfg_icp);

    icp_loop_closure_ = icp;
    icp_loop_closure_params_ = params;
  }

  // end of initialization:
  {
    auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);

    state_flags_.initialized = true;
    //state_flags_.active = params_.start_active; // TODO
  }

  MRPT_END
}

#if 0
void Mapper2D::process_action_observation(
  const mrpt::obs::CActionCollection & action, const mrpt::obs::CSensoryFrame & observations)
{
  MRPT_START

  ASSERTMSG_(
    !mapper_state_.pointcloud_generators.empty(),
    "pointcloud_generators is empty: did you call initialize()?");

  auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "process_action_observation");

  const auto relocalize_out = check_needs_relocalization(action, observations);
  const auto should_relocalize = relocalize_out.should_relocalize;

  if (should_relocalize) {
    MRPT_LOG_INFO_STREAM(
      "Processing frame at " << mrpt::system::dateTimeLocalToString(
        relocalize_out.current_frame_timestamp));

    const auto nearby_kfs = find_nearby_keyframes(max_icp_edges_per_localization);
    const auto icp_edges = run_icp_on_nearby_keyframes(nearby_kfs, relocalize_out);

    const bool duplicated_timestamp =
      mapper_state_.time_to_kf_id.hasKey(icp_edges.current_observations_timestamp);

    if (icp_edges.icp_results.empty() && !mapper_state_.empty()) {
      MRPT_LOG_WARN("No valid ICP edges for localization.");

      if (
        mapper_state_.accum_odom_since_last_kf.mean.norm() > max_lost_without_icp &&
        relocalize_out.current_frame_map->size() > 0 && !duplicated_timestamp) {
        insert_new_keyframe_and_odometry_edge(icp_edges, observations);
        optimize_pose_graph();
      }
    } else {
      const auto loc_out = run_localization_step(icp_edges);

      const bool replaced = try_replace_old_keyframes(loc_out, icp_edges, observations);

      if (replaced) {
        optimize_pose_graph();
      } else {
        if (
          (nearby_kfs.distance_to_kf_ids.empty() ||
           nearby_kfs.distance_to_kf_ids.begin()->first > max_translation_between_keyframes) &&
          !duplicated_timestamp) {
          insert_new_keyframe_and_odometry_edge(icp_edges, observations);
          add_icp_edges_to_graph(loc_out);
          optimize_pose_graph();
        }
      }
    }
  }

  save_debug_visualization_if_enabled();

  if (visualizer_) {
    updateVisualization();
  }

  MRPT_END
}
#endif

Mapper2D::RelocalizeCheckOutput Mapper2D::check_needs_relocalization(
  const mrpt::obs::CSensoryFrame & observations, const mrpt::poses::CPosePDFGaussian & odom_incr)
{
  MRPT_START

  if (observations.empty()) {
    return {};
  }

  RelocalizeCheckOutput out;
  ASSERT_(out.current_frame_map);

  out.should_relocalize = false;
  out.current_frame_timestamp = observations.getObservationByIndex(0)->getTimeStamp();

  MRPT_LOG_DEBUG_FMT(
    "Processing frame at timestamp=%f", mrpt::Clock::toDouble(out.current_frame_timestamp));

  out.current_frame_map = mapper_state_.build_metric_map_from_observations(observations);

  if (out.current_frame_map->size() == 0) {
    MRPT_LOG_WARN("Cannot localize: empty observation map (missing sensor data).");
    return out;
  }

  if (mapper_state_.empty()) {
    MRPT_LOG_DEBUG("Map is empty, starting new SLAM session.");
    out.should_relocalize = true;
    return out;
  }

  const auto & abs_odo_incr = odom_incr;

  mapper_state_.accum_odom_since_last_kf += abs_odo_incr;
  mapper_state_.accum_odom_since_last_localization += mrpt::poses::CPose3D(abs_odo_incr.mean);

  const double odo_trans = mapper_state_.accum_odom_since_last_localization.norm();
  const double odo_rot = std::abs(mapper_state_.accum_odom_since_last_localization.yaw());

  MRPT_LOG_DEBUG_STREAM(
    "Odometry increment: trans=" << odo_trans << "m, rot=" << mrpt::RAD2DEG(odo_rot) << "deg");

  if (
    odo_trans > max_translation_between_localizations_ ||
    odo_rot > max_rotation_between_localizations_) {
    MRPT_LOG_DEBUG("Relocalization triggered by motion threshold.");
    out.should_relocalize = true;
    return out;
  }

  if (mapper_state_.last_localization_time.has_value()) {
    const auto t = observations.getObservationByIndex(0)->getTimeStamp();
    const double dt = mrpt::system::timeDifference(*mapper_state_.last_localization_time, t);

    if (dt > max_time_between_localizations_) {
      MRPT_LOG_DEBUG_STREAM("Relocalization triggered by time threshold (dt=" << dt << "s).");
      out.should_relocalize = true;
      return out;
    }
  }

  return out;

  MRPT_END
}

Mapper2D::NearbyKeyFramesOutput Mapper2D::find_nearby_keyframes(
  const std::optional<size_t> & max_frames) const
{
  using gtsam::symbol_shorthand::X;

  mrpt::maps::CSimplePointsMap kf_pts;
  mrpt::containers::bimap<KeyFrameID, size_t> kf2pt_idx;

  for (const auto & kv : mapper_state_.gtsam_data.graph_values) {
    const auto s = gtsam::Symbol(kv.key);
    if (s.chr() != 'x') {
      continue;
    }

    const auto p = mrpt::gtsam_wrappers::toTPose3D(kv.value.cast<gtsam::Pose3>());
    const auto kf_idx = s.index();
    const auto pt_idx = kf_pts.size();

    kf_pts.insertPoint(p.translation());
    kf2pt_idx.insert(kf_idx, pt_idx);
  }

  if (kf_pts.empty()) {
    return {};
  }

  kf_pts.kdTreeEnsureIndexBuilt2D();

  const auto cur_pose = this->get_current_pose();
  const auto last_kf_id = mapper_state_.last_kf_id();
  const auto topological_ball = mapper_state_.get_keyframes_in_topological_radius(
    last_kf_id, min_topological_distance_for_loop_closure_);

  std::vector<nanoflann::ResultItem<size_t, float>> neighbors;

  kf_pts.kdTreeRadiusSearch2D(
    mrpt::d2f(cur_pose.x()), mrpt::d2f(cur_pose.y()),
    mrpt::d2f(mrpt::square(max_icp_search_distance_)), neighbors);

  {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(neighbors.begin(), neighbors.end(), g);
  }

  NearbyKeyFramesOutput ret;
  for (const auto & idx_dist : neighbors) {
    const auto kf_id = kf2pt_idx.inverse(idx_dist.first);
    ret.distance_to_kf_ids[std::sqrt(idx_dist.second)] = kf_id;

    if (topological_ball.count(kf_id) != 0) {
      ret.kfs_in_topological_ball.insert(kf_id);
    }

    if (max_frames && ret.distance_to_kf_ids.size() >= *max_frames) {
      break;
    }
  }

  MRPT_LOG_DEBUG_STREAM("Found " << ret.distance_to_kf_ids.size() << " nearby keyframes");

  return ret;
}

Mapper2D::ICPEdgesOutput Mapper2D::run_icp_on_nearby_keyframes(
  const NearbyKeyFramesOutput & nearby_kfs, const RelocalizeCheckOutput & relocalize_out) const
{
  auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "run_icp_on_edges");

  ICPEdgesOutput ret;
  using gtsam::symbol_shorthand::X;

  const auto cur_pose = get_current_pose();

  ret.current_observations = relocalize_out.current_frame_map;
  ret.current_observations_timestamp = relocalize_out.current_frame_timestamp;

  ASSERT_(!ret.current_observations->empty());

  const auto tentative_kf_id = mapper_state_.generate_new_kf_id();
  ret.current_observations->id = tentative_kf_id;

  const auto & pc_local = *ret.current_observations;

  for (const auto & [dist, other_id] : nearby_kfs.distance_to_kf_ids) {
    auto tle2 = mrpt::system::CTimeLoggerEntry(profiler_, "run_icp_on_edges.icp");

    const auto other_pose = mrpt::poses::CPose3D(
      mrpt::gtsam_wrappers::toTPose3D(
        mapper_state_.gtsam_data.graph_values.at<gtsam::Pose3>(X(other_id))));

    const auto rel_pose = cur_pose - other_pose;

    const bool is_loop_closure = (nearby_kfs.kfs_in_topological_ball.count(other_id) == 0) &&
                                 !mapper_state_.get_kf_connectivity().edges.empty();

    const auto & pc_global = *mapper_state_.get_keyframe_metric_map(other_id);

    MRPT_LOG_DEBUG_STREAM(
      "ICP: local_pc=" << pc_local.size()
                       << "pts, "
                          "global_pc="
                       << pc_global.size()
                       << "pts, "
                          "loop_closure="
                       << (is_loop_closure ? "yes" : "no"));

    mp2p_icp::Results icp_results;
    double min_quality = 0;

    if (!is_loop_closure) {
      min_quality = min_icp_quality_odometry_;
      icp_odometry_->align(
        pc_local, pc_global, rel_pose.asTPose(), icp_odometry_params_, icp_results);
    } else {
      min_quality = min_icp_quality_loop_closure_;
      icp_loop_closure_->align(
        pc_local, pc_global, rel_pose.asTPose(), icp_loop_closure_params_, icp_results);
    }

    MRPT_LOG_DEBUG_STREAM(
      "ICP quality: " << (100.0 * icp_results.quality)
                      << "% "
                         "(threshold: "
                      << (100.0 * min_quality) << "%)");

    if (icp_results.quality < min_quality) {
      continue;
    }

    ret.icp_results[other_id] = icp_results;
  }

  return ret;
}

void Mapper2D::insert_new_keyframe_and_odometry_edge(
  const ICPEdgesOutput & icp_edges_out, const mrpt::obs::CSensoryFrame & observations)
{
  using gtsam::symbol_shorthand::X;

  const auto obs_time = icp_edges_out.current_observations_timestamp;
  mrpt::poses::CPose3D initial_pose_guess;
  std::optional<KeyFrameID> last_kf_id;
  bool is_first_kf = false;

  if (mapper_state_.empty()) {
    initial_pose_guess = mrpt::poses::CPose3D::Identity();
    is_first_kf = true;
  } else {
    last_kf_id = mapper_state_.last_kf_id();
    const auto last_pose = mrpt::poses::CPose3D(
      mrpt::gtsam_wrappers::toTPose3D(
        mapper_state_.gtsam_data.graph_values.at<gtsam::Pose3>(X(*last_kf_id))));

    initial_pose_guess = get_current_pose();
  }

  const auto kf_id = mapper_state_.generate_new_kf_id();

  MRPT_LOG_INFO_STREAM("Inserting new keyframe #" << kf_id);

  ASSERT_EQUAL_(kf_id, *icp_edges_out.current_observations->id);

  mapper_state_.gtsam_data.graph_values.insert(
    X(kf_id), mrpt::gtsam_wrappers::toPose3(initial_pose_guess));

  if (is_first_kf) {
    auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1.0);
    mapper_state_.gtsam_data.graph_factors.addPrior(
      X(kf_id), mrpt::gtsam_wrappers::toPose3(initial_pose_guess), prior_noise);
  }

  mapper_state_.time_to_kf_id.insert(obs_time, kf_id);

  {
    auto & sf = mapper_state_.keyframe_observations[kf_id];
    for (const auto & label : sensor_labels_for_simplemap_) {
      if (auto obs = observations.getObservationBySensorLabel(label); obs) {
        sf.insert(std::const_pointer_cast<mrpt::obs::CObservation>(obs));
      }
    }
  }

  mapper_state_.set_keyframe_metric_map(kf_id, icp_edges_out.current_observations);

  if (
    last_kf_id.has_value() &&
    std::abs(
      mrpt::system::timeDifference(mapper_state_.time_to_kf_id.inverse(*last_kf_id), obs_time)) <
      max_time_for_odometry_edge_) {
    mrpt::poses::CPose3DPDFGaussianInf odometry_edge;
    odometry_edge.mean = mrpt::poses::CPose3D(mapper_state_.accum_odom_since_last_kf.mean);

    gtsam::Matrix6 odo_cov = gtsam::Matrix6::Zero();
    odo_cov(0, 0) = odo_cov(1, 1) = odo_cov(2, 2) = mrpt::square(
      std::max(odometry_edge_sigma_, mapper_state_.accum_odom_since_last_kf.cov(2, 2)));
    odo_cov(3, 3) = mrpt::square(
      std::max(odometry_edge_sigma_, mapper_state_.accum_odom_since_last_kf.cov(0, 0)));
    odo_cov(4, 4) = mrpt::square(
      std::max(odometry_edge_sigma_, mapper_state_.accum_odom_since_last_kf.cov(1, 1)));
    odo_cov(5, 5) = mrpt::square(odometry_edge_sigma_);

    auto odo_noise = gtsam::noiseModel::Gaussian::Covariance(odo_cov);

    mapper_state_.gtsam_data.graph_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      X(*last_kf_id), X(kf_id), mrpt::gtsam_wrappers::toPose3(odometry_edge.mean), odo_noise);

    mapper_state_.add_kf_connectivity(*last_kf_id, kf_id);

    MRPT_LOG_DEBUG_STREAM("Added odometry edge: KF#" << *last_kf_id << " -> KF#" << kf_id);
  }

  mapper_state_.accum_odom_since_last_kf = {
    mrpt::poses::CPose2D::Identity(), mrpt::math::CMatrixDouble33::Zero()};
}

Mapper2D::LocalizationOutput Mapper2D::run_localization_step(const ICPEdgesOutput & icp_edges)
{
  using gtsam::symbol_shorthand::X;

  auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "run_localization_step");

  LocalizationOutput ret;

  const auto tentative_kf_id = mapper_state_.generate_new_kf_id();
  auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

  gtsam::NonlinearFactorGraph fg_edges;
  gtsam::NonlinearFactorGraph fg;
  gtsam::Values values;

  for (const auto & [kf_id, icp_result] : icp_edges.icp_results) {
    auto icp_edge = mrpt::poses::CPose3DPDFGaussianInf(
      icp_result.optimal_tf.mean, icp_result.optimal_tf.cov.inverse_LLt());

    auto icp_noise = gtsam::noiseModel::Isotropic::Sigma(6, icp_edge_sigma_);

    gtsam::noiseModel::Base::shared_ptr icp_rob_noise;

    if (icp_edge_robust_parameter_ > 0) {
      icp_rob_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Fair::Create(icp_edge_robust_parameter_), icp_noise);
    } else {
      icp_rob_noise = icp_noise;
    }

    fg_edges.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      X(kf_id), X(tentative_kf_id), mrpt::gtsam_wrappers::toPose3(icp_edge.mean), icp_rob_noise);

    const auto kf_pose = mapper_state_.gtsam_data.graph_values.at<gtsam::Pose3>(X(kf_id));
    fg.addPrior(X(kf_id), kf_pose, prior_noise);

    values.insert(X(kf_id), kf_pose);
  }

  const auto cur_pose = mrpt::gtsam_wrappers::toPose3(get_current_pose());

  if (icp_edges.icp_results.empty()) {
    fg.addPrior(X(tentative_kf_id), cur_pose, prior_noise);
  }

  values.insert(X(tentative_kf_id), cur_pose);
  fg += fg_edges;

  auto lm_params = gtsam::LevenbergMarquardtParams::LegacyDefaults();
  lm_params.maxIterations = 20;

  auto optimizer = gtsam::LevenbergMarquardtOptimizer(fg, values, lm_params);
  const auto & optimal_values = optimizer.optimize();

  const auto cur_optimal_pose = optimal_values.at<gtsam::Pose3>(X(tentative_kf_id));

  mapper_state_.last_localization =
    mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(cur_optimal_pose));
  mapper_state_.last_localization_time = icp_edges.current_observations_timestamp;

  mapper_state_.accum_odom_since_last_localization = mrpt::poses::CPose3D::Identity();

  MRPT_LOG_DEBUG_STREAM("Localization updated: pose=" << mapper_state_.last_localization);

  ret.factor_graph_edges = fg_edges;
  ret.tentative_new_kf_id = tentative_kf_id;

  return ret;
}

void Mapper2D::add_icp_edges_to_graph(const LocalizationOutput & loc_out)
{
  using gtsam::symbol_shorthand::X;

  mapper_state_.gtsam_data.graph_factors += loc_out.factor_graph_edges;
  mapper_state_.add_kf_connectivity_from_factor_graph(loc_out.factor_graph_edges);

  mapper_state_.gtsam_data.graph_values.update(
    X(loc_out.tentative_new_kf_id), mrpt::gtsam_wrappers::toPose3(get_current_pose()));
}

bool Mapper2D::try_replace_old_keyframes(  // NOLINT
  const LocalizationOutput & loc_out, const ICPEdgesOutput & icp_edges,
  const mrpt::obs::CSensoryFrame & observations)
{
  using gtsam::symbol_shorthand::X;

  if (icp_edges.icp_results.empty()) {
    return false;
  }

  const auto cur_kf_id = loc_out.tentative_new_kf_id;
  const double cur_kf_time = mrpt::Clock::toDouble(icp_edges.current_observations_timestamp);

  bool replacement_done = false;

  for (const auto & [other_kf_id, icp_edge] : icp_edges.icp_results) {
    {
      if (icp_edge.quality < min_icp_quality_keyframe_replacement_) {
        continue;
      }
    }

    const auto & rel_pose_old_to_cur = icp_edge.optimal_tf.mean;
    const double other_kf_time =
      mrpt::Clock::toDouble(mapper_state_.time_to_kf_id.inverse(other_kf_id));

    const double age = cur_kf_time - other_kf_time;
    if (age < min_keyframe_age_for_replacement_) {
      continue;
    }

    MRPT_LOG_DEBUG_STREAM(
      "Replacing old KF#" << other_kf_id << " (age=" << age
                          << "s) "
                             "with KF#"
                          << cur_kf_id);

    if (!replacement_done) {
      insert_new_keyframe_and_odometry_edge(icp_edges, observations);
    }
    replacement_done = true;

    gtsam::NonlinearFactorGraph new_factors;

    for (auto & f : mapper_state_.gtsam_data.graph_factors) {
      if (const auto * prior_factor =
            dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3> *>(f.get());
          prior_factor) {
        const auto key1 = gtsam::Symbol(prior_factor->key());
        ASSERT_(key1.chr() == 'x');

        if (key1.index() != other_kf_id) {
          continue;
        }

        gtsam::Pose3 new_prior_pose = mapper_state_.gtsam_data.graph_values.at<gtsam::Pose3>(key1) *
                                      mrpt::gtsam_wrappers::toPose3(rel_pose_old_to_cur);

        auto new_prior = boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          X(cur_kf_id), new_prior_pose, prior_factor->noiseModel());

        new_factors += new_prior;
        f.reset();
      } else if (const auto * between_factor =
                   dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3> *>(f.get());
                 between_factor) {
        const auto key1 = gtsam::Symbol(between_factor->key1());
        const auto key2 = gtsam::Symbol(between_factor->key2());
        ASSERT_(key1.chr() == 'x');
        ASSERT_(key2.chr() == 'x');

        std::optional<KeyFrameID> new_other_kf_id;
        gtsam::Pose3 old_to_new_other;

        if (key1.index() == other_kf_id) {
          new_other_kf_id = key2.index();
          old_to_new_other = between_factor->measured();
        } else if (key2.index() == other_kf_id) {
          new_other_kf_id = key1.index();
          old_to_new_other = between_factor->measured().inverse();
        }

        if (!new_other_kf_id.has_value()) {
          continue;
        }

        if (*new_other_kf_id == cur_kf_id) {
          f.reset();
          continue;
        }

        const auto new_edge_rel_pose =
          mrpt::gtsam_wrappers::toPose3(rel_pose_old_to_cur).inverse() * old_to_new_other;

        if (new_edge_rel_pose.translation().norm() < max_icp_search_distance_) {
          auto new_edge = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(cur_kf_id), X(*new_other_kf_id), new_edge_rel_pose, between_factor->noiseModel());

          mapper_state_.add_kf_connectivity(cur_kf_id, *new_other_kf_id);
          new_factors += new_edge;
        }

        f.reset();
      }
    }

    mapper_state_.gtsam_data.graph_factors += new_factors;
    internal_delete_keyframe(other_kf_id);
  }

  return replacement_done;
}

void Mapper2D::optimize_pose_graph()
{
  auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "optimize_pose_graph");

  auto lm_params = gtsam::LevenbergMarquardtParams::LegacyDefaults();
  lm_params.maxIterations = 100;

  const auto n_factors = mapper_state_.gtsam_data.graph_factors.size();
  const auto n_factors_1 = n_factors > 0 ? 1.0 / static_cast<double>(n_factors) : 1.0;

  lm_params.iterationHook = [n_factors_1](size_t iter, double err_init, double err_final) {
    std::cout << "[LM] iter: " << iter << " rmse: " << std::sqrt(err_init * n_factors_1) << " => "
              << std::sqrt(err_final * n_factors_1) << "\n";
  };

  auto optimizer = gtsam::LevenbergMarquardtOptimizer(
    mapper_state_.gtsam_data.graph_factors, mapper_state_.gtsam_data.graph_values, lm_params);

  if (debug_print_factor_graphs_) {
    mapper_state_.gtsam_data.graph_factors.print();
    mapper_state_.gtsam_data.graph_values.print();
  }

  const auto & optimal_values = optimizer.optimize();
  mapper_state_.gtsam_data.graph_values = optimal_values;
}

mrpt::viz::CSetOfObjects::Ptr Mapper2D::build_visualization() const
{
  using namespace std::string_literals;

  auto lck = mrpt::lockHelper(state_mtx_);

  auto gl_map = mrpt::viz::CSetOfObjects::Create();

  // Pose graph visualization
  auto gl_pose_graph = mrpt::viz::CSetOfObjects::Create();
  auto gl_edges = mrpt::viz::CSetOfLines::Create();
  gl_pose_graph->insert(gl_edges);
  gl_edges->setColor_u8(0x00, 0x00, 0xff, 0x60);

  // Keyframe poses
  for (const auto & kv : mapper_state_.gtsam_data.graph_values) {
    const auto key = kv.key;
    const auto pose = mrpt::gtsam_wrappers::toTPose3D(kv.value.cast<gtsam::Pose3>());

    auto gl_corner = mrpt::viz::stock_objects::CornerXYZSimple(0.20f);
    gl_corner->setPose(pose);
    gl_corner->setName("KF#"s + std::to_string(gtsam::Symbol(key).index()));

    gl_pose_graph->insert(gl_corner);
  }

  // Edges
  for (const auto & f : mapper_state_.gtsam_data.graph_factors) {
    const auto * between_fac = dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3> *>(f.get());
    if (between_fac == nullptr) {
      continue;
    }

    const auto key1 = mrpt::gtsam_wrappers::toTPose3D(
      mapper_state_.gtsam_data.graph_values.at<gtsam::Pose3>(between_fac->key1()));
    const auto key2 = mrpt::gtsam_wrappers::toTPose3D(
      mapper_state_.gtsam_data.graph_values.at<gtsam::Pose3>(between_fac->key2()));

    gl_edges->appendLine(key1.translation(), key2.translation());
  }

  gl_pose_graph->setLocation(0, 0, 0.1);
  gl_map->insert(gl_pose_graph);

  // Point clouds
  for (const auto & kv : mapper_state_.time_to_kf_id.getDirectMap()) {
    const auto kf_id = kv.second;
    const auto & pc = mapper_state_.get_keyframe_metric_map(kf_id);

    auto & gl_pts = cached_viz_point_clouds_[kf_id];
    if (!gl_pts) {
      mp2p_icp::render_params_t rp;
      auto & color_mode = rp.points.allLayers.colorMode.emplace();
      color_mode.colorMap = mrpt::img::cmHOT;
      color_mode.colorMapMinCoord = -0.5f;
      color_mode.colorMapMaxCoord = 5.0f;
      color_mode.recolorizeByField = "z";

      gl_pts = pc->get_visualization(rp);
    }

    using gtsam::symbol_shorthand::X;
    gl_pts->setPose(
      mrpt::gtsam_wrappers::toTPose3D(
        mapper_state_.gtsam_data.graph_values.at<gtsam::Pose3>(X(kf_id))));

    gl_map->insert(gl_pts);
  }

  // Localization poses
  {
    auto gl_loc_pose = mrpt::viz::stock_objects::CornerXYZ(1.5f);
    gl_loc_pose->setPose(mapper_state_.last_localization);
    gl_map->insert(gl_loc_pose);
  }

  {
    auto gl_cur_pose = mrpt::viz::stock_objects::CornerXYZ(2.0f);
    gl_cur_pose->setPose(
      mapper_state_.last_localization +
      mapper_state_.accum_odom_since_last_localization.getPoseMean());
    gl_map->insert(gl_cur_pose);
  }

  return gl_map;
}

void Mapper2D::save_state_to_3d_scene(const std::string & filename) const
{
  mrpt::viz::Scene scene;
  scene.insert(build_visualization());
  scene.saveToFile(filename);
}

mrpt::poses::CPose3D Mapper2D::get_current_pose() const
{
  auto lck = mrpt::lockHelper(state_mtx_);

  const auto cur_pose_estimate = mapper_state_.last_localization +
                                 mapper_state_.accum_odom_since_last_localization.getPoseMean();

  return cur_pose_estimate;
}

mrpt::maps::CSimpleMap Mapper2D::get_current_map() const
{
  auto lck = mrpt::lockHelper(state_mtx_);
  return mapper_state_.as_simple_map();
}

void Mapper2D::resume_session(const mrpt::poses::CPose3D & current_pose)
{
  mapper_state_.accum_odom_since_last_kf = {
    mrpt::poses::CPose2D::Identity(), mrpt::math::CMatrixDouble33::Zero()};
  mapper_state_.accum_odom_since_last_localization = mrpt::poses::CPose3D::Identity();

  mapper_state_.last_localization = current_pose;
  mapper_state_.last_localization_time = mrpt::Clock::now();
}

void Mapper2D::internal_delete_keyframe(KeyFrameID kf_id) { mapper_state_.delete_keyframe(kf_id); }

void Mapper2D::save_debug_visualization_if_enabled()
{
  static thread_local int save_scene_cnt = 0;
  static thread_local int save_cnt = 0;

  if (
    save_3d_scenes_decimation_ > 0 && !mapper_state_.empty() &&
    ++save_scene_cnt >= save_3d_scenes_decimation_) {
    save_scene_cnt = 0;

    save_state_to_3d_scene(
      save_3d_scenes_prefix_ +
      mrpt::format(
        "_%05u_KF%05u.3Dscene", save_cnt++, static_cast<unsigned int>(mapper_state_.last_kf_id())));
  }
}

void Mapper2D::spinOnce()
{
  MRPT_TRY_START

  const ProfilerEntry tle(profiler_, "spinOnce");

  // processPendingUserRequests();

#if 0
  // Force a refresh of the GUI?
  // Executed here since
  // otherwise the GUI would never show up if inactive, or if the LIDAR
  // observations are misconfigured and are not been fed in.
  if (visualizer_ && ((state_.local_map && state_.local_map->empty()) || !isActive())) {
    if (mrpt::Clock::nowDouble() - gui_.timestampLastUpdateUI > 1.0) {
      updateVisualization();
    }
  }

  // If SLAM/Localization is disabled, refresh the current map
  // here if needed, since it won't be published until observations arrive.
  {
    auto lckState = mrpt::lockHelper(state_mtx_);

    const auto mapStamp =
      state_.last_obs_timestamp ? *state_.last_obs_timestamp : mrpt::Clock::now();

    doPublishUpdatedLocalMap(mapStamp);
  }

  // Publish optional regular diagnostics:
  if (module_is_time_to_publish_diagnostics()) {
    onPublishDiagnostics();
  }
#endif

  MRPT_TRY_END
}

}  // namespace mola