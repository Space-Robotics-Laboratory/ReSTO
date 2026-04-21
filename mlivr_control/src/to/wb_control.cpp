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

#include "mlivr_control/to/wb_control.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace mlivr_control
{

WBControl::WBControl(const rclcpp::NodeOptions & options) : BaseController("wb_control", options)
{
  // === ROS 2 parameters ===
  WbcSolverParams params;
  params.solver.horizon_steps = this->declare_parameter<int>("solver.horizon_steps", 100);
  params.solver.dt = this->declare_parameter<double>("solver.dt", 0.01);
  params.solver.max_iter = this->declare_parameter<int>("solver.max_iter");

  params.weights.state_reg = this->declare_parameter<double>("weights.state_reg");
  params.weights.control_reg = this->declare_parameter<double>("weights.control_reg");
  params.weights.state_limits = this->declare_parameter<double>("weights.state_limits");
  params.weights.control_limits = this->declare_parameter<double>("weights.control_limits");
  params.weights.ee_tracking = this->declare_parameter<double>("weights.ee_tracking");
  params.weights.sup_ee_tracking = this->declare_parameter<double>("weights.sup_ee_tracking");
  params.weights.ee_vel_damping = this->declare_parameter<double>("weights.ee_vel_damping");
  params.weights.env_collision = this->declare_parameter<double>("weights.env_collision");
  params.weights.momentum_reg = this->declare_parameter<double>("weights.momentum_reg");

  params.ctrl_reg_schedule.s_accel =
    this->declare_parameter<double>("weight_schedules.ctrl_reg.s_accel");
  params.ctrl_reg_schedule.s_decel =
    this->declare_parameter<double>("weight_schedules.ctrl_reg.s_decel");
  params.ctrl_reg_schedule.accel_multi =
    this->declare_parameter<double>("weight_schedules.ctrl_reg.accel_multi");
  params.ctrl_reg_schedule.decel_multi =
    this->declare_parameter<double>("weight_schedules.ctrl_reg.decel_multi");
  params.ee_vel_schedule.s_accel =
    this->declare_parameter<double>("weight_schedules.ee_vel.s_accel");
  params.ee_vel_schedule.s_decel =
    this->declare_parameter<double>("weight_schedules.ee_vel.s_decel");
  params.ee_vel_schedule.accel_multi =
    this->declare_parameter<double>("weight_schedules.ee_vel.accel_multi");
  params.ee_vel_schedule.decel_multi =
    this->declare_parameter<double>("weight_schedules.ee_vel.decel_multi");

  params.ee_frames = ee_frames_;
  // ========================

  model_ptr_ = std::make_shared<pinocchio::Model>(robot_->getModel());
  wbc_solver_ = std::make_unique<WbcSolver>(model_ptr_, params);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

bool WBControl::generateTrajectory()
{
  Eigen::Vector3d offset(0.0, -0.2, 0.0);  // in world frame

  {
    pinocchio::Data data(*model_ptr_);
    Eigen::VectorXd q_all = Eigen::VectorXd::Zero(model_ptr_->nq);
    q_all.head(7) = current_base_pose_;
    for (int i = 0; i < num_joints_; ++i) {
      q_all(7 + i) = current_joint_pos_[i];
    }
    pinocchio::forwardKinematics(*model_ptr_, data, q_all);
    pinocchio::updateFramePlacements(*model_ptr_, data);

    pinocchio::FrameIndex swing_id = model_ptr_->getFrameId(ee_frames_[1]);
    target_ee_pose_se3_ = data.oMf[swing_id];
    target_ee_pose_se3_.translation() += offset;

    Eigen::Quaterniond q_target(target_ee_pose_se3_.rotation());
    this->publishTargetTF(target_ee_pose_se3_.translation(), q_target, "target/" + ee_frames_[1]);
  }

  bool success = wbc_solver_->computeTrajectory(
    current_base_pose_, current_base_twist_, current_joint_pos_, ee_frames_[0], ee_frames_[1],
    offset);

  if (success) {
    playback_idx_ = 0;
    RCLCPP_INFO(this->get_logger(), "WBC Trajectory optimization completed.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "WBC Trajectory optimization failed.");
  }

  return success;
}

Eigen::VectorXd WBControl::computeCommandStep()
{
  const auto & optimized_xs = wbc_solver_->getOptimizedXs();
  const auto & optimized_us = wbc_solver_->getOptimizedUs();

  if (optimized_xs.empty()) {
    return Eigen::VectorXd::Zero(1);
  }

  bool is_finished = (playback_idx_ >= optimized_xs.size() - 1);
  size_t current_idx = is_finished ? (optimized_xs.size() - 1) : playback_idx_;

  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = this->now();

  Eigen::VectorXd q_des = optimized_xs[current_idx].segment(7, num_joints_);

  Eigen::VectorXd v_des(num_joints_);
  Eigen::VectorXd tau_opt(num_joints_);

  if (is_finished) {
    v_des = Eigen::VectorXd::Zero(num_joints_);
    tau_opt = Eigen::VectorXd::Zero(num_joints_);
  } else {
    v_des = optimized_xs[current_idx].segment(model_ptr_->nv + 6, num_joints_);
    tau_opt = optimized_us[current_idx];
  }

  for (int i = 0; i < num_joints_; ++i) {
    cmd_msg.name.push_back(model_ptr_->names[i + 2]);
    cmd_msg.position.push_back(q_des(i));
    cmd_msg.velocity.push_back(v_des(i));
    cmd_msg.effort.push_back(tau_opt(i));
  }

  cmd_pub_->publish(cmd_msg);

  Eigen::VectorXd current_planned_q = optimized_xs[current_idx].head(model_ptr_->nq);
  publishPlannedRobotState(current_planned_q);

  Eigen::Quaterniond q_target(target_ee_pose_se3_.rotation());
  this->publishTargetTF(target_ee_pose_se3_.translation(), q_target, "target/" + ee_frames_[1]);

  if (!is_finished) {
    playback_idx_++;
  }

  return Eigen::VectorXd::Zero(1);
}

std::vector<Eigen::Vector3d> WBControl::getPlannedPath()
{
  std::vector<Eigen::Vector3d> path;
  const auto & optimized_xs = wbc_solver_->getOptimizedXs();
  if (optimized_xs.empty()) {
    return path;
  }

  pinocchio::Data data(*model_ptr_);
  pinocchio::FrameIndex swing_id = model_ptr_->getFrameId(ee_frames_[1]);

  for (const auto & x : optimized_xs) {
    Eigen::VectorXd q = x.head(model_ptr_->nq);
    pinocchio::forwardKinematics(*model_ptr_, data, q);
    pinocchio::updateFramePlacements(*model_ptr_, data);
    path.push_back(data.oMf[swing_id].translation());
  }

  return path;
}

void WBControl::publishWaitingState()
{
  Eigen::VectorXd q_all = Eigen::VectorXd::Zero(model_ptr_->nq);
  q_all.head(7) = current_base_pose_;
  for (int i = 0; i < num_joints_; ++i) {
    q_all(7 + i) = current_joint_pos_[i];
  }
  publishPlannedRobotState(q_all);
}

void WBControl::publishPlannedRobotState(const Eigen::VectorXd & q_all)
{
  pinocchio::Data data(*model_ptr_);

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

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::WBControl)
