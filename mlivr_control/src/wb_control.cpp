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
  cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_cmds", 10);

  // Subscriber
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&WBControl::jointStateCallback, this, std::placeholders::_1));

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&WBControl::publishCommandStep, this));

  // ROS 2 parameters
  ee_frames_ = this->declare_parameter<std::vector<std::string>>("ee_frames");
  WbcSolverParams params;
  params.solver.horizon_steps = this->declare_parameter<int>("solver.horizon_steps", 100);
  params.solver.dt = this->declare_parameter<double>("solver.dt", 0.01);
  params.weights.swing_goal = this->declare_parameter<double>("solver.swing_goal", 1e4);
  params.weights.state_reg = this->declare_parameter<double>("solver.state_reg", 1e-1);
  params.weights.control_reg = this->declare_parameter<double>("solver.control_reg", 1e-4);
  params.weights.state_limits = this->declare_parameter<double>("solver.state_limits", 1000.0);
  params.weights.control_limits = this->declare_parameter<double>("solver.control_limits", 1000.0);

  std::string urdf_path =
    ament_index_cpp::get_package_share_directory("mlivr_description") + "/urdf/mlivr.urdf";
  robot_core_ = std::make_unique<mlivr_model::RobotCore>(urdf_path);
  model_ptr_ = std::make_shared<pinocchio::Model>(robot_core_->getModel());

  wbc_solver_ = std::make_unique<WbcSolver>(model_ptr_, params);

  num_joints_ = model_ptr_->nv - 6;
  current_q_.resize(num_joints_, 0.0);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

void WBControl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (is_initialized_) {
    return;
  }

  for (size_t i = 0; i < static_cast<size_t>(num_joints_) && i < msg->position.size(); ++i) {
    current_q_[i] = msg->position[i];
  }

  Eigen::Vector3d offset(0.0, -0.5, -0.0);  // 手先座標系の移動量

  // Call solver to perform computation
  if (wbc_solver_->computeTrajectory(current_q_, ee_frames_[0], ee_frames_[1], offset)) {
    is_initialized_ = true;
  }
}

void WBControl::publishCommandStep()
{
  const auto & optimized_xs = wbc_solver_->getOptimizedXs();

  if (!is_initialized_ || optimized_xs.empty() || playback_idx_ >= optimized_xs.size()) {
    return;
  }

  Eigen::VectorXd q_opt = optimized_xs[playback_idx_].head(model_ptr_->nq);

  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    cmd_msg.data[i] = q_opt(7 + i);
  }
  cmd_pub_->publish(cmd_msg);

  playback_idx_++;
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::WBControl)
