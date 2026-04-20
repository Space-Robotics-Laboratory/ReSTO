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

  md_solver_ = std::make_unique<ramp::md::MomentumDistribution>(num_joints_, ee_frames_.size());
  lrst_optimizer_ = std::make_unique<ramp::lrst::LowReactionSwingTrajectory>(
    kinematics_.get(), dynamics_.get(), num_joints_, ee_frames_.size());

  target_joint_pos_.resize(num_joints_, 0.0);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

// bool RAMPControl::generateTrajectory()
// {
//   int nq = robot_->getModel().nq;
//   Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
//   q.head(7) = current_base_pose_;
//   for (int i = 0; i < num_joints_; ++i) {
//     q(7 + i) = current_joint_pos_[i];
//     target_joint_pos_[i] = current_joint_pos_[i];
//   }

//   pinocchio::SE3 pose_R = kinematics_->solveFK(q, ee_frames_[1]);

//   Eigen::Vector3d start_pos = pose_R.translation();
//   Eigen::Quaterniond start_quat(pose_R.rotation());

//   auto displacement = Eigen::Vector3d(0.0, -0.2, 0.0);
//   Eigen::Vector3d target_pos = start_pos + displacement;
//   Eigen::Quaterniond target_quat = start_quat;

//   Eigen::Vector3d swing_height = Eigen::Vector3d(0.0, 0.0, 0.05);
//   Eigen::Vector3d mid_pos = start_pos + displacement / 2.0 + swing_height;
//   Eigen::Quaterniond mid_quat = start_quat;

//   duration_ = 10.0;

//   trajectory_generator::VectorStateConstraint start_p_c{
//     0.0, start_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
//   trajectory_generator::VectorStateConstraint end_p_c{
//     duration_, target_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
//   auto pos_constraints = trajectory_generator::createBoundaryConditions(start_p_c, end_p_c);

//   trajectory_generator::VectorStateConstraint mid_p_c{duration_ / 2.0, mid_pos};
//   trajectory_generator::addConstraint(pos_constraints, mid_p_c);

//   trajectory_generator::AngularStateConstraint start_o_c{
//     0.0, start_quat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
//   trajectory_generator::AngularStateConstraint end_o_c{
//     duration_, target_quat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
//   auto ori_constraints = trajectory_generator::createBoundaryConditions(start_o_c, end_o_c);

//   trajectory_generator::AngularStateConstraint mid_o_c{duration_ / 2.0, mid_quat};
//   trajectory_generator::addConstraint(ori_constraints, mid_o_c);

//   pos_spline_ = std::make_unique<trajectory_generator::VectorSpline>(pos_constraints, 3);
//   ori_spline_ = std::make_unique<trajectory_generator::OrientationSpline>(ori_constraints);

//   trajectory_start_time_ = this->now().seconds();
//   is_trajectory_active_ = true;
//   RCLCPP_INFO(this->get_logger(), "Trajectory generation completed. Started tracking.");

//   return true;
// }

bool RAMPControl::generateTrajectory()
{
  RCLCPP_INFO(this->get_logger(), "Start trajectory generation.");

  int nq = robot_->getModel().nq;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
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

  duration_ = 10.0;

  // ====================================================
  // ▼ LRST による軌道の最適化
  // ====================================================
  RCLCPP_INFO(this->get_logger(), "Set boundary conditions.");
  lrst_optimizer_->setBoundaryConditions(start_pos, target_pos);

  // 動かす遊脚(ee_frames_[1])の関節名を自動抽出
  std::vector<std::string> swing_joint_names;
  int joints_per_limb = num_joints_ / ee_frames_.size();
  for (int i = 0; i < joints_per_limb; ++i) {
    // pinocchioのモデルに合わせてインデックスを計算 (ベースの分+2を考慮)
    swing_joint_names.push_back(robot_->getModel().names[2 + 1 * joints_per_limb + i]);
  }

  RCLCPP_INFO(this->get_logger(), "Set robot state.");
  lrst_optimizer_->setRobotState(q, ee_frames_[1], swing_joint_names);

  ramp::lrst::SolverParams current_solver_params;
  current_solver_params.dt = 0.01;
  current_solver_params.step_duration = duration_;
  current_solver_params.step_height = 0.05;

  ramp::lrst::WeightParams current_weight_params;
  current_weight_params.force_max = 1.0;
  current_weight_params.moment_max = 1.0;
  current_weight_params.step_height_max = 100.0;
  current_weight_params.step_height_ave = 100.0;

  optimized_bezier_P_ =
    lrst_optimizer_->optimizeTrajectory(current_solver_params, current_weight_params);

  // ====================================================
  // 姿勢(Orientation)の軌道生成はそのまま使用
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

  // Eigen::Vector3d v_world = pos_spline_->getVelocity(current_time);
  // Eigen::Vector3d w_world = ori_spline_->getAngularVelocity(current_time);
  Eigen::Vector3d v_world =
    lrst_optimizer_->computeBezierVelocity(current_time, optimized_bezier_P_);
  Eigen::Vector3d w_world = ori_spline_->getAngularVelocity(current_time);

  // 2. 遊脚手先(ee_frames_[1])の現在の姿勢(FK)を取得
  pinocchio::SE3 pose_swing = kinematics_->solveFK(q, ee_frames_[1]);
  // World -> Local への回転変換行列（現在の回転行列の転置）
  Eigen::Matrix3d R_world_to_local = pose_swing.rotation().transpose();

  // 3. 速度をLocal座標系に変換
  Eigen::VectorXd x_dot_swing_des = Eigen::VectorXd::Zero(6);
  x_dot_swing_des.head(3) = R_world_to_local * v_world;
  x_dot_swing_des.tail(3) = R_world_to_local * w_world;

  Eigen::MatrixXd J_sup = kinematics_->computeJacobian(q, ee_frames_[0]);
  Eigen::MatrixXd J_swing = kinematics_->computeJacobian(q, ee_frames_[1]);

  Eigen::MatrixXd J_b_sup = J_sup.block(0, 0, 6, 6);
  Eigen::MatrixXd J_m_sup = J_sup.block(0, 6, 6, num_joints_);

  Eigen::MatrixXd J_b_swing = J_swing.block(0, 0, 6, 6);
  Eigen::MatrixXd J_m_swing = J_swing.block(0, 6, 6, num_joints_);

  Eigen::MatrixXd H_b, H_bm;
  dynamics_->computePartitionedMassMatrices(q, H_b, H_bm);

  // --- RAMP-MD による計算フロー ---

  // ① ベース固定と仮定した「ノミナル」な遊脚の関節速度と運動量を計算
  Eigen::MatrixXd J_m_swing_pinv = J_m_swing.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::VectorXd phi_dot_swing_nom = J_m_swing_pinv * x_dot_swing_des;
  Eigen::VectorXd L_swing_nom =
    H_bm * phi_dot_swing_nom;  // 1ステップ前ではなく、今のノミナル値を使う

  double alpha = 1.0;  // FMD

  // ② 【重要】遊脚の連成運動量を考慮して、ベースの慣性行列を補正する
  Eigen::MatrixXd H_b_modified = H_b - alpha * H_bm * J_m_swing_pinv * J_b_swing;

  // ③ 補正した慣性行列 H_b_modified と ノミナル運動量 L_swing_nom を使ってMDソルバーを呼ぶ
  auto md_cmd =
    md_solver_->computeVelocities(H_b_modified, H_bm, J_b_sup, J_m_sup, L_swing_nom, alpha);

  // ④ 算出されたベース速度を使って、遊脚の「実際の」関節角速度を計算
  Eigen::VectorXd phi_dot_swing_real =
    J_m_swing_pinv * (x_dot_swing_des - J_b_swing * md_cmd.base_velocity);

  // ⑤ 最終的な全身の関節角速度ベクトルを合成
  Eigen::VectorXd phi_dot_total = md_cmd.support_limb_joint_velocities + phi_dot_swing_real;

  // For debug (確認用)
  // 元々の H_b を使って、実際のシステム全体の運動量を再計算
  // Eigen::VectorXd L = H_b * md_cmd.base_velocity + H_bm * phi_dot_total;
  // std::cout << "L = " << L.transpose() << std::endl;

  // --- 制御コマンドの生成とパブリッシュ ---

  double dt = 0.01;
  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = this->now();

  cmd_msg.position.resize(num_joints_);
  cmd_msg.velocity.resize(num_joints_);

  for (int i = 0; i < num_joints_; ++i) {
    // 速度指令値からオイラー積分して目標角度を更新
    target_joint_pos_[i] += phi_dot_total(i) * dt;

    cmd_msg.name.push_back(robot_->getModel().names[i + 2]);
    cmd_msg.position[i] = target_joint_pos_[i];
    cmd_msg.velocity[i] = phi_dot_total(i);
    // cmd_msg.velocity.push_back(0.0);
    cmd_msg.effort.push_back(0.0);
  }

  // JointStateパブリッシュ
  cmd_pub_->publish(cmd_msg);

  // return phi_dot_total;
  return Eigen::VectorXd::Zero(1);
}

// std::vector<Eigen::Vector3d> RAMPControl::getPlannedPath()
// {
//   std::vector<Eigen::Vector3d> path;
//   if (!pos_spline_) return path;

//   double dt = 0.05;
//   for (double t = 0; t <= duration_; t += dt) {
//     path.push_back(pos_spline_->getPosition(t));
//   }
//   return path;
// }

std::vector<Eigen::Vector3d> RAMPControl::getPlannedPath()
{
  std::vector<Eigen::Vector3d> path;

  if (!lrst_optimizer_ || optimized_bezier_P_.cols() == 0) {
    return path;
  }

  double dt = 0.05;
  for (double t = 0; t <= duration_; t += dt) {
    path.push_back(lrst_optimizer_->computeBezierPosition(t, optimized_bezier_P_));
  }
  path.push_back(lrst_optimizer_->computeBezierPosition(duration_, optimized_bezier_P_));

  return path;
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::RAMPControl)
