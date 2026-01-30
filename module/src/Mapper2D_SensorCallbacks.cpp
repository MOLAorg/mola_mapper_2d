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
 * @file   Mapper2D_SensorCallbacks.cpp
 * @brief  Sensor observation callbacks for 2D Graph-SLAM
 * @author Jose Luis Blanco Claraco
 * @date   Jan 2026
 */

#include <mola_mapper_2d/Mapper2D.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationOdometry.h>

namespace mola
{

void Mapper2D::onNewObservation(const CObservation::ConstPtr & o)
{
  MRPT_TRY_START
  const ProfilerEntry tle(profiler_, "onNewObservation");

  ASSERT_(o);

  // Check initialization state
  {
    auto lck = mrpt::lockHelper(state_flags_mtx_);

    if (!state_flags_.initialized) {
      MRPT_LOG_THROTTLE_ERROR(
        2.0, "Discarding incoming observations: initialize() has not been called yet!");
      return;
    }
    if (state_flags_.fatal_error) {
      MRPT_LOG_THROTTLE_ERROR(2.0, "Discarding incoming observations: a fatal error occurred.");
      this->requestShutdown();
      return;
    }
    if (!state_flags_.active) {
      return;
    }
  }

  // Check if it's an Odometry observation
  if (odometry_sensor_label_ && std::regex_match(o->sensorLabel, *odometry_sensor_label_)) {
    auto odom = std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o);
    if (odom) {
      onOdometry(odom);
    } else {
      MRPT_LOG_WARN_STREAM(
        "Observation with label '" << o->sensorLabel
                                   << "' matched odometry regex but is not CObservationOdometry");
    }
    return;
  }

  // Check if it's a GNSS observation
  if (gnss_sensor_label_ && std::regex_match(o->sensorLabel, *gnss_sensor_label_)) {
    auto gnss = std::dynamic_pointer_cast<const mrpt::obs::CObservationGPS>(o);
    if (gnss) {
      onGNSS(gnss);
    } else {
      MRPT_LOG_WARN_STREAM(
        "Observation with label '" << o->sensorLabel
                                   << "' matched GNSS regex but is not CObservationGPS");
    }
    return;
  }

  // Check if it's a LiDAR observation
  for (const auto & re : lidar_sensor_labels_) {
    if (!std::regex_match(o->sensorLabel, re)) {
      continue;
    }

    // Enqueue for processing in worker thread
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      if (destructor_called_) {
        return;
      }
      auto lck2 = mrpt::lockHelper(state_flags_mtx_);
      state_flags_.worker_tasks_lidar++;
    }

    auto fut = worker_lidar_.enqueue(&Mapper2D::onLidar, this, o);
    (void)fut;

    break;  // Don't keep processing the list
  }

  MRPT_TRY_END
}

void Mapper2D::onLidar(const CObservation::ConstPtr & o)
{
  const bool abort_running = [this]() {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    return destructor_called_;
  }();

  if (!abort_running) {
    try {
      auto scan = std::dynamic_pointer_cast<const mrpt::obs::CObservation2DRangeScan>(o);
      if (scan) {
        processLidarScan(scan);
      } else {
        MRPT_LOG_WARN_STREAM(
          "LiDAR observation with label '" << o->sensorLabel << "' is not CObservation2DRangeScan");
      }
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("Exception in onLidar:\n" << mrpt::exception_to_str(e));
      auto lck = mrpt::lockHelper(state_flags_mtx_);
      state_flags_.fatal_error = true;
    }
  }

  {
    auto lck = mrpt::lockHelper(state_flags_mtx_);
    state_flags_.worker_tasks_lidar--;
  }
}

void Mapper2D::processLidarScan(const mrpt::obs::CObservation2DRangeScan::ConstPtr & scan)
{
  MRPT_START
  const ProfilerEntry tle(profiler_, "processLidarScan");

  const auto scan_timestamp = scan->timestamp;

  // Check minimum time between scans
  if (last_lidar_timestamp_.has_value()) {
    const double dt = mrpt::system::timeDifference(*last_lidar_timestamp_, scan_timestamp);
    if (dt < min_time_between_scans_) {
      MRPT_LOG_DEBUG_STREAM(
        "Skipping scan (dt=" << dt << "s < min=" << min_time_between_scans_ << "s)");
      return;
    }
  }

  // Compute odometry increment since last scan
  mrpt::poses::CPosePDFGaussian odom_incr;
  odom_incr.mean = mrpt::poses::CPose2D::Identity();
  odom_incr.cov.setZero();

  {
    auto lck = mrpt::lockHelper(odometry_mtx_);
    // If we have odometry, compute the increment
    // (handled via accumulated odometry in mapper_state_)
  }

  // Build sensory frame with the scan and optionally GNSS
  mrpt::obs::CSensoryFrame observations;
  observations.insert(std::const_pointer_cast<mrpt::obs::CObservation2DRangeScan>(scan));

  // Add closest GNSS observation if available
  auto gnss = getClosestGNSS(scan_timestamp);
  if (gnss) {
    observations.insert(std::const_pointer_cast<mrpt::obs::CObservationGPS>(gnss));
  }

  // Process SLAM step
  processSlamStep(observations, odom_incr);

  last_lidar_timestamp_ = scan_timestamp;

  MRPT_END
}

void Mapper2D::onOdometry(const mrpt::obs::CObservationOdometry::ConstPtr & o)
{
  MRPT_START
  const ProfilerEntry tle(profiler_, "onOdometry");

  MRPT_LOG_DEBUG_STREAM(
    "Odometry received: t=" << mrpt::Clock::toDouble(o->timestamp)
                            << " pose=" << o->odometry.asString());

  auto lck = mrpt::lockHelper(odometry_mtx_);

  // Compute increment from absolute odometry
  if (odometry_state_.last_absolute_odometry.has_value()) {
    const auto incr = o->odometry - *odometry_state_.last_absolute_odometry;

    // Skip if robot is stationary
    if (incr == mrpt::poses::CPose2D::Identity()) {
      return;
    }

    // Build increment with estimated covariance
    mrpt::poses::CPosePDFGaussian incr_pdf;
    incr_pdf.mean = incr;

    // Estimate covariance based on motion (simple model)
    const double trans = incr.norm();
    const double rot = std::abs(incr.phi());
    incr_pdf.cov(0, 0) = mrpt::square(0.05 * trans + 0.01);              // x variance
    incr_pdf.cov(1, 1) = mrpt::square(0.05 * trans + 0.01);              // y variance
    incr_pdf.cov(2, 2) = mrpt::square(0.05 * rot + mrpt::DEG2RAD(1.0));  // phi variance

    // Accumulate into mapper state
    {
      auto lck2 = mrpt::lockHelper(state_mtx_);
      mapper_state_.accum_odom_since_last_kf += incr_pdf;
      mapper_state_.accum_odom_since_last_localization += mrpt::poses::CPose3D(incr);
    }

    MRPT_LOG_DEBUG_STREAM(
      "Odometry increment: " << incr.asString() << " accumulated="
                             << mapper_state_.accum_odom_since_last_kf.mean.asString());
  }

  odometry_state_.last_absolute_odometry = o->odometry;
  odometry_state_.last_odometry_timestamp = o->timestamp;

  MRPT_END
}

void Mapper2D::onGNSS(const mrpt::obs::CObservationGPS::ConstPtr & o)
{
  MRPT_START
  const ProfilerEntry tle(profiler_, "onGNSS");

  MRPT_LOG_DEBUG_STREAM("GNSS observation received, t=" << mrpt::Clock::toDouble(o->timestamp));

  // Validate covariance if present
  if (o->covariance_enu) {
    const auto minCov = o->covariance_enu->minimumDiagonal();
    if (minCov < 0 || std::isnan(minCov) || std::isinf(minCov)) {
      MRPT_LOG_THROTTLE_WARN_STREAM(5.0, "Discarding GNSS observation with invalid covariance");
      return;
    }
  }

  auto lck = mrpt::lockHelper(gnss_mtx_);

  // Keep the latest GNSS observations for simplemap insertion
  last_gnss_.emplace(o->timestamp, o);

  // Remove old ones
  while (last_gnss_.size() > GNSS_QUEUE_MAX_SIZE) {
    last_gnss_.erase(last_gnss_.begin());
  }

  MRPT_END
}

mrpt::obs::CObservationGPS::ConstPtr Mapper2D::getClosestGNSS(
  const mrpt::Clock::time_point & timestamp, double max_age_seconds) const
{
  auto lck = mrpt::lockHelper(gnss_mtx_);

  if (last_gnss_.empty()) {
    return nullptr;
  }

  // Find the GNSS observation closest to the given timestamp
  auto it = last_gnss_.lower_bound(timestamp);

  mrpt::obs::CObservationGPS::ConstPtr best;
  double best_dt = std::numeric_limits<double>::max();

  // Check the element at or after timestamp
  if (it != last_gnss_.end()) {
    const double dt = std::abs(mrpt::system::timeDifference(timestamp, it->first));
    if (dt < best_dt && dt <= max_age_seconds) {
      best_dt = dt;
      best = it->second;
    }
  }

  // Check the element before timestamp
  if (it != last_gnss_.begin()) {
    --it;
    const double dt = std::abs(mrpt::system::timeDifference(timestamp, it->first));
    if (dt < best_dt && dt <= max_age_seconds) {
      best_dt = dt;
      best = it->second;
    }
  }

  return best;
}

void Mapper2D::processSlamStep(
  const mrpt::obs::CSensoryFrame & observations, const mrpt::poses::CPosePDFGaussian & odom_incr)
{
  MRPT_START

  auto lck = mrpt::lockHelper(state_mtx_);

  ASSERTMSG_(
    !mapper_state_.pointcloud_generators.empty(),
    "pointcloud_generators is empty: did you call initialize()?");

  auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "processSlamStep");

  const auto relocalize_out = check_needs_relocalization(observations, odom_incr);
  const auto should_relocalize = relocalize_out.should_relocalize;

  if (should_relocalize) {
    MRPT_LOG_INFO_STREAM(
      "Processing frame at " << mrpt::system::dateTimeLocalToString(
        relocalize_out.current_frame_timestamp));

    const auto nearby_kfs = find_nearby_keyframes(max_icp_edges_per_localization_);
    const auto icp_edges = run_icp_on_nearby_keyframes(nearby_kfs, relocalize_out);

    const bool duplicated_timestamp =
      mapper_state_.time_to_kf_id.hasKey(icp_edges.current_observations_timestamp);

    if (icp_edges.icp_results.empty() && !mapper_state_.empty()) {
      MRPT_LOG_WARN("No valid ICP edges for localization.");

      if (
        mapper_state_.accum_odom_since_last_kf.mean.norm() > max_lost_without_icp_ &&
        relocalize_out.current_frame_map->size() > 0 && !duplicated_timestamp) {
        insert_new_keyframe_and_odometry_edge(icp_edges, observations);
        optimize_pose_graph();
      }
    } else {
      // Store best ICP quality for visualization
      double best_quality = 0.0;
      for (const auto & [kf_id, result] : icp_edges.icp_results) {
        best_quality = std::max(best_quality, result.quality);
      }
      last_icp_quality_ = best_quality;

      const auto loc_out = run_localization_step(icp_edges);

      const bool replaced = try_replace_old_keyframes(loc_out, icp_edges, observations);

      if (replaced) {
        optimize_pose_graph();
      } else {
        if (
          (nearby_kfs.distance_to_kf_ids.empty() ||
           nearby_kfs.distance_to_kf_ids.begin()->first > max_translation_between_keyframes_) &&
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

bool Mapper2D::isBusy() const
{
  auto lck = mrpt::lockHelper(state_flags_mtx_);
  return state_flags_.worker_tasks_lidar > 0 || worker_lidar_.pendingTasks() > 0;
}

bool Mapper2D::isActive() const
{
  auto lck = mrpt::lockHelper(state_flags_mtx_);
  return state_flags_.active;
}

void Mapper2D::setActive(bool active)
{
  auto lck = mrpt::lockHelper(state_flags_mtx_);
  state_flags_.active = active;
}

}  // namespace mola