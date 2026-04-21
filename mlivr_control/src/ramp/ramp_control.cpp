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

#include "mlivr_control/ramp/ramp_control.hpp"

#include <Eigen/QR>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace mlivr_control
{

RAMPControl::RAMPControl(const rclcpp::NodeOptions & options)
: BaseController("ramp_control", options)
{
  kinematics_ = std::make_unique<fbml::Kinematics>(*robot_);
  dynamics_ = std::make_unique<fbml::Dynamics>(*robot_);

  // ROS 2 parameters
  use_lrst_ = this->declare_parameter<bool>("use_lrst", true);
  default_solver_params_.dt = this->declare_parameter<double>("solver.dt", 0.01);
  default_solver_params_.step_duration =
    this->declare_parameter<double>("solver.step_duration", 0.0);
  default_solver_params_.step_height = this->declare_parameter<double>("solver.step_height", 0.0);
  default_solver_params_.relative_tol =
    this->declare_parameter<double>("solver.relative_tol", 1e-4);
  default_solver_params_.max_iter = this->declare_parameter<int>("solver.max_iter", 1000);
  default_weight_params_.force_max = this->declare_parameter<double>("weights.force_max", 1.0);
  default_weight_params_.moment_max = this->declare_parameter<double>("weights.moment_max", 1.0);
  default_weight_params_.step_height_max =
    this->declare_parameter<double>("weights.step_height_max", 1.0);
  default_weight_params_.step_height_ave =
    this->declare_parameter<double>("weights.step_height_ave", 1.0);
  momentum_distribution_factor_ =
    this->declare_parameter<double>("momentum_distribution_factor", 0.5);

  duration_ = default_solver_params_.step_duration;

  lrst_optimizer_ = std::make_unique<ramp::lrst::LowReactionSwingTrajectory>(
    kinematics_.get(), dynamics_.get(), num_joints_, ee_frames_.size());
  md_solver_ = std::make_unique<ramp::md::MomentumDistribution>(num_joints_, ee_frames_.size());

  target_joint_pos_.resize(num_joints_, 0.0);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

bool RAMPControl::generateTrajectory()
{
  int nq = robot_->getModel().nq;
  Eigen::VectorXd q = pinocchio::neutral(robot_->getModel());
  q.head(7) = current_base_pose_;
  for (int i = 0; i < num_joints_; ++i) {
    q(7 + i) = current_joint_pos_[i];
    target_joint_pos_[i] = current_joint_pos_[i];
  }

  pinocchio::SE3 pose_R = kinematics_->solveFK(q, ee_frames_[1]);
  Eigen::Vector3d start_pos = pose_R.translation();
  Eigen::Quaterniond start_quat(pose_R.rotation());

  auto displacement = Eigen::Vector3d(0.0, -0.2, 0.0);
  Eigen::Vector3d target_pos = start_pos + displacement;
  Eigen::Quaterniond target_quat = start_quat;

  if (use_lrst_) {
    RCLCPP_INFO(this->get_logger(), "Generating trajectory using LRST optimization.");
    lrst_optimizer_->setBoundaryConditions(start_pos, target_pos);

    std::vector<std::string> swing_limb_joint_names =
      robot_->getJointNamesBetweenFrames("base_link", ee_frames_[1]);

    lrst_optimizer_->setRobotState(q, ee_frames_[1], swing_limb_joint_names);

    ramp::lrst::SolverParams current_solver_params = default_solver_params_;
    ramp::lrst::WeightParams current_weight_params = default_weight_params_;

    optimized_bezier_P_ =
      lrst_optimizer_->optimizeTrajectory(current_solver_params, current_weight_params);
  } else {
    RCLCPP_INFO(this->get_logger(), "Generating trajectory using parabolic Spline.");

    Eigen::Vector3d swing_height = Eigen::Vector3d(0.0, 0.0, default_solver_params_.step_height);
    Eigen::Vector3d mid_pos = start_pos + displacement / 2.0 + swing_height;

    trajectory_generator::VectorStateConstraint start_p_c{
      0.0, start_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    trajectory_generator::VectorStateConstraint end_p_c{
      duration_, target_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    auto pos_constraints = trajectory_generator::createBoundaryConditions(start_p_c, end_p_c);

    trajectory_generator::VectorStateConstraint mid_p_c{duration_ / 2.0, mid_pos};
    trajectory_generator::addConstraint(pos_constraints, mid_p_c);

    pos_spline_ = std::make_unique<trajectory_generator::VectorSpline>(pos_constraints, 3);
  }

  // Generate orientation trajectory using spline
  trajectory_generator::AngularStateConstraint start_o_c{
    0.0, start_quat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  trajectory_generator::AngularStateConstraint end_o_c{
    duration_, target_quat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  auto ori_constraints = trajectory_generator::createBoundaryConditions(start_o_c, end_o_c);
  ori_spline_ = std::make_unique<trajectory_generator::OrientationSpline>(ori_constraints);

  trajectory_start_time_ = this->now().seconds();
  is_trajectory_active_ = true;
  RCLCPP_INFO(this->get_logger(), "Trajectory generation completed. Started tracking.");

  return true;
}

Eigen::VectorXd RAMPControl::computeCommandStep()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!is_trajectory_active_) {
    return Eigen::VectorXd::Zero(num_joints_);
  }

  double current_time = this->now().seconds() - trajectory_start_time_;

  double t_eval = std::min(current_time, duration_);

  Eigen::Vector3d p_planned, p_target;
  if (use_lrst_) {
    p_planned = lrst_optimizer_->computeBezierPosition(t_eval, optimized_bezier_P_);
    p_target = lrst_optimizer_->computeBezierPosition(duration_, optimized_bezier_P_);
  } else {
    p_planned = pos_spline_->getPosition(t_eval);
    p_target = pos_spline_->getPosition(duration_);
  }

  Eigen::Quaterniond q_planned = ori_spline_->getOrientation(t_eval);
  Eigen::Quaterniond q_target = ori_spline_->getOrientation(duration_);

  this->publishTargetTF(p_planned, q_planned, "planned/" + ee_frames_[1]);
  this->publishTargetTF(p_target, q_target, "target/" + ee_frames_[1]);

  if (current_time > duration_) {
    is_trajectory_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Trajectory finished.");
    return Eigen::VectorXd::Zero(num_joints_);
  }

  int nq = robot_->getModel().nq;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
  q.head(7) = current_base_pose_;
  for (int i = 0; i < num_joints_; ++i) {
    q(7 + i) = current_joint_pos_[i];
  }

  Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_world = Eigen::Vector3d::Zero();

  if (use_lrst_) {
    v_world = lrst_optimizer_->computeBezierVelocity(current_time, optimized_bezier_P_);
    w_world = ori_spline_->getAngularVelocity(current_time);
  } else {
    v_world = pos_spline_->getVelocity(current_time);
    w_world = ori_spline_->getAngularVelocity(current_time);
  }

  // 2. 遊脚手先(ee_frames_[1])の現在の姿勢(FK)を取得
  pinocchio::SE3 pose_swing = kinematics_->solveFK(q, ee_frames_[1]);
  // World -> Local への回転変換行列（現在の回転行列の転置）
  Eigen::Matrix3d R_world_to_local = pose_swing.rotation().transpose();

  // 3. 速度をLocal座標系に変換
  Eigen::VectorXd v_ee_sw_des = Eigen::VectorXd::Zero(6);
  v_ee_sw_des.head(3) = R_world_to_local * v_world;
  v_ee_sw_des.tail(3) = R_world_to_local * w_world;

  Eigen::MatrixXd J_sup = kinematics_->computeJacobian(q, ee_frames_[0]);
  Eigen::MatrixXd J_b_sup = J_sup.block(0, 0, 6, 6);
  Eigen::MatrixXd J_m_sup = J_sup.block(0, 6, 6, num_joints_);

  Eigen::MatrixXd J_sw = kinematics_->computeJacobian(q, ee_frames_[1]);
  Eigen::MatrixXd J_b_sw = J_sw.block(0, 0, 6, 6);
  Eigen::MatrixXd J_m_sw = J_sw.block(0, 6, 6, num_joints_);

  Eigen::MatrixXd H_b, H_bm;
  dynamics_->computePartitionedMassMatrices(q, H_b, H_bm);

  // --- RAMP-MD による計算フロー ---

  auto md_cmd = md_solver_->computeVelocities(
    H_b, H_bm, J_b_sup, J_m_sup, J_b_sw, J_m_sw, v_ee_sw_des, momentum_distribution_factor_);

  Eigen::VectorXd dq_ref = md_cmd.joint_velocities;

  // --- 制御コマンドの生成とパブリッシュ ---

  double dt = 0.01;
  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = this->now();

  cmd_msg.position.resize(num_joints_);
  cmd_msg.velocity.resize(num_joints_);

  for (int i = 0; i < num_joints_; ++i) {
    // 速度指令値からオイラー積分して目標角度を更新
    target_joint_pos_[i] += dq_ref(i) * dt;

    cmd_msg.name.push_back(robot_->getModel().names[i + 2]);
    cmd_msg.position[i] = target_joint_pos_[i];
    cmd_msg.velocity[i] = dq_ref(i);
    cmd_msg.effort.push_back(0.0);
  }

  cmd_pub_->publish(cmd_msg);

  return Eigen::VectorXd::Zero(1);
}

std::vector<Eigen::Vector3d> RAMPControl::getPlannedPath()
{
  std::vector<Eigen::Vector3d> path;
  double dt = 0.05;

  if (use_lrst_) {
    if (!lrst_optimizer_ || optimized_bezier_P_.cols() == 0) {
      return path;
    }

    for (double t = 0; t <= duration_; t += dt) {
      path.push_back(lrst_optimizer_->computeBezierPosition(t, optimized_bezier_P_));
    }
    path.push_back(lrst_optimizer_->computeBezierPosition(duration_, optimized_bezier_P_));
  } else {
    if (!pos_spline_) {
      return path;
    }

    for (double t = 0; t <= duration_; t += dt) {
      path.push_back(pos_spline_->getPosition(t));
    }
  }

  return path;
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::RAMPControl)
