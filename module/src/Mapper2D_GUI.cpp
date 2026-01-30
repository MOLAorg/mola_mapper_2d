/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
*/

/**
 * @file   Mapper2D_GUI.cpp
 * @brief  GUI/Visualization for 2D Graph-SLAM
 * @author Jose Luis Blanco Claraco
 * @date   Jan 2026
 */

#include <mola_mapper_2d/Mapper2D.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>

// GTSAM:
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace mola
{

void Mapper2D::internalBuildGUI()
{
  ASSERT_(gui_.ui);

  gui_.ui->requestFocus();
  gui_.ui->setPosition({5, 700});

  gui_.ui->setLayout(
    new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
  gui_.ui->setFixedWidth(300);

  auto * tabWidget = gui_.ui->add<nanogui::TabWidget>();

  auto * tab1 = tabWidget->createTab("Status");
  tab1->setLayout(new nanogui::GroupLayout());

  auto * tab2 = tabWidget->createTab("Control");
  tab2->setLayout(new nanogui::GroupLayout());

  tabWidget->setActiveTab(0);

  // Tab 1: Status
  gui_.lbIcpQuality = tab1->add<nanogui::Label>(" ");
  gui_.lbKeyframes = tab1->add<nanogui::Label>(" ");
  gui_.lbLoopClosures = tab1->add<nanogui::Label>(" ");
  gui_.lbTime = tab1->add<nanogui::Label>(" ");

  // Tab 2: Control
  gui_.cbActive = tab2->add<nanogui::CheckBox>("Active");
  gui_.cbActive->setChecked(true);
  gui_.cbActive->setCallback([this](bool checked) {
    // TODO: Add active state control
    (void)checked;
  });

  gui_.cbMapping = tab2->add<nanogui::CheckBox>("Mapping enabled");
  gui_.cbMapping->setChecked(true);
  gui_.cbMapping->setCallback([this](bool checked) {
    // TODO: Add mapping enable/disable
    (void)checked;
  });
}

void Mapper2D::updateVisualization()
{
  if (!visualizer_) {
    return;
  }

  auto lck = mrpt::lockHelper(state_mtx_);

  std::vector<std::function<void()>> updateTasks;

  // Vehicle pose
  if (!gl_vehicle_frame_) {
    updateVisualizationInitVehFrame();
  }

  // Update vehicle pose
  gl_vehicle_frame_->setPose(mapper_state_.last_localization);
  updateTasks.emplace_back(
    [this]() { visualizer_->update_3d_object("mapper2d/vehicle", gl_vehicle_frame_); });

  // Update path visualization
  updateVisualizationPath(updateTasks);

  // Update local map visualization
  updateVisualizationLocalMap(updateTasks);

  // Camera follows vehicle (if enabled)
  if (
    viz_params_.has("camera_follows_vehicle") && viz_params_["camera_follows_vehicle"].as<bool>()) {
    updateTasks.emplace_back([this]() {
      visualizer_->update_viewport_look_at(mapper_state_.last_localization.translation());
    });
  }

  // Execute all update tasks
  for (const auto & ut : updateTasks) {
    ut();
  }

  // Create/update GUI subwindow
  auto lckGui = mrpt::lockHelper(state_gui_mtx_);
  if (gui_.ui == nullptr) {
    auto fut = visualizer_->create_subwindow("mola_mapper_2d");
    gui_.ui = fut.get();

    auto fut2 = visualizer_->enqueue_custom_nanogui_code([this]() { internalBuildGUI(); });
    fut2.get();
  }

  // Update text labels
  updateVisualizationTextLabels();
}

void Mapper2D::updateVisualizationInitVehFrame()
{
  gl_vehicle_frame_ = mrpt::opengl::CSetOfObjects::Create();

  // Add coordinate frame corner
  double cornerSize = 1.0;
  if (viz_params_.has("current_pose_corner_size")) {
    cornerSize = viz_params_["current_pose_corner_size"].as<double>();
  }

  if (cornerSize > 0) {
    auto glCorner = mrpt::opengl::stock_objects::CornerXYZ(static_cast<float>(cornerSize));
    gl_vehicle_frame_->insert(glCorner);
  }
}

void Mapper2D::updateVisualizationPath(std::vector<std::function<void()>> & update_tasks)
{
  using gtsam::symbol_shorthand::X;

  bool showTrajectory = true;
  if (viz_params_.has("show_trajectory")) {
    showTrajectory = viz_params_["show_trajectory"].as<bool>();
  }

  if (!showTrajectory) {
    return;
  }

  if (!gl_estimated_path_) {
    gl_estimated_path_ = mrpt::opengl::CSetOfLines::Create();
    gl_estimated_path_->setColor_u8(0x00, 0xff, 0x00, 0xff);  // Green
    gl_path_group_ = mrpt::opengl::CSetOfObjects::Create();
  }

  // Build path from keyframe poses
  const auto & values = mapper_state_.gtsam_data.graph_values;

  gl_estimated_path_->clear();

  bool first = true;
  mrpt::math::TPoint3D lastPt;

  for (const auto & [timestamp, kfId] : mapper_state_.time_to_kf_id) {
    // Get pose from GTSAM values
    if (values.exists(kfId)) {
      try {
        auto pose = values.at<gtsam::Pose3>(X(kfId));
        mrpt::math::TPoint3D pt(pose.x(), pose.y(), pose.z());

        if (first) {
          lastPt = pt;
          first = false;
        } else {
          gl_estimated_path_->appendLine(lastPt, pt);
          lastPt = pt;
        }
      } catch (...) {
        // Key might be of different type, skip
      }
    }
  }

  gl_path_group_->clear();
  gl_path_group_->insert(mrpt::opengl::CSetOfLines::Create(*gl_estimated_path_));

  update_tasks.emplace_back(
    [this]() { visualizer_->update_3d_object("mapper2d/path", gl_path_group_); });
}

void Mapper2D::updateVisualizationLocalMap(std::vector<std::function<void()>> & update_tasks)
{
  using gtsam::symbol_shorthand::X;

  bool showLocalMap = true;
  if (viz_params_.has("show_local_map")) {
    showLocalMap = viz_params_["show_local_map"].as<bool>();
  }

  if (!showLocalMap || !local_map_needs_viz_update_) {
    return;
  }

  int mapUpdateDecimation = 10;
  if (viz_params_.has("map_update_decimation")) {
    mapUpdateDecimation = viz_params_["map_update_decimation"].as<int>();
  }

  if (map_update_counter_++ < mapUpdateDecimation) {
    return;
  }

  map_update_counter_ = 0;
  local_map_needs_viz_update_ = false;

  // Build combined point cloud from nearby keyframes
  auto glMap = mrpt::opengl::CSetOfObjects::Create();

  float pointSize = 3.0f;
  if (viz_params_.has("local_map_point_size")) {
    pointSize = static_cast<float>(viz_params_["local_map_point_size"].as<double>());
  }

  // Get keyframes around current pose
  auto nearbyKfs = find_nearby_keyframes(50);  // max 50 keyframes

  for (const auto & [dist, kfId] : nearbyKfs.distance_to_kf_ids) {
    const auto & mm = mapper_state_.get_keyframe_metric_map(kfId);
    if (!mm) {
      continue;
    }

    // Get keyframe pose
    if (!mapper_state_.gtsam_data.graph_values.exists(kfId)) {
      continue;
    }

    try {
      auto pose = mapper_state_.gtsam_data.graph_values.at<gtsam::Pose3>(X(kfId));
      mrpt::poses::CPose3D mrptPose(
        pose.x(), pose.y(), pose.z(), pose.rotation().yaw(), pose.rotation().pitch(),
        pose.rotation().roll());

      mp2p_icp::render_params_t rp;
      rp.points.allLayers.pointSize = pointSize;

      auto glKf = mm->get_visualization(rp);
      glKf->setPose(mrptPose);
      glMap->insert(glKf);
    } catch (...) {
      // Skip on error
    }
  }

  update_tasks.emplace_back(
    [this, glMap]() { visualizer_->update_3d_object("mapper2d/localmap", glMap); });
}

void Mapper2D::updateVisualizationTextLabels()
{
  if (gui_.lbKeyframes == nullptr) {
    return;
  }

  gui_.lbKeyframes->setCaption(mrpt::format("Keyframes: %zu", mapper_state_.time_to_kf_id.size()));

  gui_.lbLoopClosures->setCaption(
    mrpt::format("Graph edges: %zu", mapper_state_.gtsam_data.graph_factors.size()));

  // ICP quality from last localization
  gui_.lbIcpQuality->setCaption("ICP quality: --");

  gui_.lbTime->setCaption(mrpt::format("Process time: -- ms"));
}

}  // namespace mola