// Copyright (c) 2026 Masazumi Imai
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mlivr_control/wb_control.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mlivr_control
{

WBControl::WBControl(const rclcpp::NodeOptions & options) : Node("wb_control", options)
{
  // Publisher
  cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_cmds", 10);
  ee_path_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("planned_trajectory", 10);

  // Subscriber
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&WBControl::jointStateCallback, this, std::placeholders::_1));
  trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/start_wbc", 10, std::bind(&WBControl::triggerCallback, this, std::placeholders::_1));

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&WBControl::publishCommandStep, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // ROS 2 parameters
  ee_frames_ = this->declare_parameter<std::vector<std::string>>("ee_frames");
  WbcSolverParams params;
  params.solver.horizon_steps = this->declare_parameter<int>("solver.horizon_steps", 100);
  params.solver.dt = this->declare_parameter<double>("solver.dt", 0.01);
  params.weights.swing_goal = this->declare_parameter<double>("weights.swing_goal");
  params.weights.ee_vel_damping = this->declare_parameter<double>("weights.ee_vel_damping");
  params.weights.state_reg = this->declare_parameter<double>("weights.state_reg");
  params.weights.control_reg = this->declare_parameter<double>("weights.control_reg");
  params.weights.state_limits = this->declare_parameter<double>("weights.state_limits");
  params.weights.control_limits = this->declare_parameter<double>("weights.control_limits");
  params.weights.momentum_reg = this->declare_parameter<double>("weights.momentum_reg");
  params.weights.ext_collision = this->declare_parameter<double>("weights.ext_collision");

  std::string urdf_path =
    ament_index_cpp::get_package_share_directory("mlivr_description") + "/urdf/mlivr.urdf";
  robot_core_ = std::make_unique<mlivr_model::RobotCore>(urdf_path);
  model_ptr_ = std::make_shared<pinocchio::Model>(robot_core_->getModel());

  wbc_solver_ = std::make_unique<WbcSolver>(model_ptr_, params);

  num_joints_ = model_ptr_->nv - 6;
  current_q_.resize(num_joints_, 0.0);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

void WBControl::publishCommandStep()
{
  const auto & optimized_xs = wbc_solver_->getOptimizedXs();
  const auto & optimized_us = wbc_solver_->getOptimizedUs();  // include only joint torque

  if (!is_initialized_ || optimized_xs.empty() || playback_idx_ >= optimized_xs.size()) {
    return;
  }

  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = this->now();

  Eigen::VectorXd q_des = optimized_xs[playback_idx_].segment(7, num_joints_);
  Eigen::VectorXd v_des = optimized_xs[playback_idx_].segment(model_ptr_->nv + 6, num_joints_);

  Eigen::VectorXd tau_opt(num_joints_);
  if (playback_idx_ < optimized_us.size()) {
    tau_opt = optimized_us[playback_idx_];
  } else {
    tau_opt = Eigen::VectorXd::Zero(num_joints_);
  }

  for (int i = 0; i < num_joints_; ++i) {
    cmd_msg.name.push_back("joint_" + std::to_string(i));  // TODO: Temporary

    cmd_msg.position.push_back(q_des(i));
    cmd_msg.velocity.push_back(v_des(i));
    cmd_msg.effort.push_back(tau_opt(i));
  }

  cmd_pub_->publish(cmd_msg);

  publishPlannedRobotState(optimized_xs);

  playback_idx_++;
}

void WBControl::publishPlannedRobotState(const std::vector<Eigen::VectorXd> & optimized_xs)
{
  pinocchio::Data data(*model_ptr_);

  Eigen::VectorXd q_all = optimized_xs[playback_idx_].head(model_ptr_->nq);

  pinocchio::forwardKinematics(*model_ptr_, data, q_all);
  pinocchio::updateFramePlacements(*model_ptr_, data);

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(model_ptr_->frames.size());

  // Broadcast the coordinates of all frames (links) in world frame
  for (size_t i = 0; i < model_ptr_->frames.size(); ++i) {
    const auto & frame = model_ptr_->frames[i];

    if (frame.type != pinocchio::BODY && frame.type != pinocchio::JOINT) continue;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "planned/" + frame.name;  // HACK

    const auto & oMf = data.oMf[i];
    Eigen::Vector3d trans = oMf.translation();
    Eigen::Quaterniond quat(oMf.rotation());

    tf_msg.transform.translation.x = trans.x();
    tf_msg.transform.translation.y = trans.y();
    tf_msg.transform.translation.z = trans.z();
    tf_msg.transform.rotation.x = quat.x();
    tf_msg.transform.rotation.y = quat.y();
    tf_msg.transform.rotation.z = quat.z();
    tf_msg.transform.rotation.w = quat.w();

    transforms.push_back(tf_msg);
  }

  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

void WBControl::publishTrajectoryMarker()
{
  const auto & xs = wbc_solver_->getOptimizedXs();
  if (xs.empty()) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = this->now();
  marker.ns = "wbc_planned_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;  // Line width [mm]

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  pinocchio::Data data(*model_ptr_);
  std::string swing_frame = ee_frames_[1];
  pinocchio::FrameIndex swing_id = model_ptr_->getFrameId(swing_frame);

  for (const auto & x : xs) {
    Eigen::VectorXd q = x.head(model_ptr_->nq);

    pinocchio::forwardKinematics(*model_ptr_, data, q);
    pinocchio::updateFramePlacements(*model_ptr_, data);

    Eigen::Vector3d pos = data.oMf[swing_id].translation();

    geometry_msgs::msg::Point p;
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
    marker.points.push_back(p);
  }

  ee_path_marker_pub_->publish(marker);
}

void WBControl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (is_initialized_) {
    return;
  }

  for (size_t i = 0; i < static_cast<size_t>(num_joints_) && i < msg->position.size(); ++i) {
    current_q_[i] = msg->position[i];
  }

  if (!is_triggered_) {
    std::vector<Eigen::VectorXd> dummy_xs;
    Eigen::VectorXd q_all = Eigen::VectorXd::Zero(model_ptr_->nq);

    // Initial base pose (match with MuJoCo) // TODO: Change hard cord
    q_all(0) = 0.0;
    q_all(1) = 0.1;
    q_all(2) = 1.0;
    q_all(6) = 1.0;
    for (int i = 0; i < num_joints_; ++i) q_all(7 + i) = current_q_[i];

    dummy_xs.push_back(q_all);

    size_t temp_idx = playback_idx_;
    playback_idx_ = 0;
    publishPlannedRobotState(dummy_xs);
    playback_idx_ = temp_idx;

    return;
  }

  Eigen::Vector3d offset(0.0, -0.2, 0.0);  // in world frame

  if (wbc_solver_->computeTrajectory(current_q_, ee_frames_[0], ee_frames_[1], offset)) {
    is_initialized_ = true;
    publishTrajectoryMarker();
  }
}

void WBControl::triggerCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if (!is_triggered_ && !is_initialized_) {
    RCLCPP_INFO(this->get_logger(), "Optimization Triggered! Computing WBC trajectory...");
    is_triggered_ = true;
  }
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::WBControl)
