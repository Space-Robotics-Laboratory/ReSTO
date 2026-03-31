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

#include "mlivr_control/gj_control.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace mlivr_control
{

GJControl::GJControl(const rclcpp::NodeOptions & options) : Node("gj_control", options)
{
  cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_cmds", 10);

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&GJControl::jointStateCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&GJControl::controlLoop, this));

  std::string urdf_path =
    ament_index_cpp::get_package_share_directory("mlivr_description") + "/urdf/mlivr.urdf";
  robot_core_ = std::make_unique<mlivr_model::RobotCore>(urdf_path);
  kinematics_ = std::make_unique<mlivr_model::Kinematics>(*robot_core_);
  dynamics_ = std::make_unique<mlivr_model::Dynamics>(*robot_core_);

  current_q_.resize(kTotalNumJoints, 0.0);
  current_v_.resize(kTotalNumJoints, 0.0);
  target_q_.resize(kTotalNumJoints, 0.0);
  is_initialized_ = false;

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

void GJControl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < static_cast<size_t>(kTotalNumJoints) && i < msg->position.size(); ++i) {
    current_q_[i] = msg->position[i];

    if (i < msg->velocity.size()) {
      current_v_[i] = msg->velocity[i];
    } else {
      current_v_[i] = 0.0;
    }
  }

  if (!is_initialized_) {
    target_q_ = current_q_;
    is_initialized_ = true;
  }
}

// void GJControl::controlLoop()
// {
//   std::lock_guard<std::mutex> lock(state_mutex_);
//   if (!is_initialized_) return;

//   // --- 1. 一般化座標 q の構築 (ベースは固定 [0,0,0, 1] とする) ---
//   Eigen::VectorXd q = Eigen::VectorXd::Zero(21);
//   q(6) = 1.0;
//   for (int i = 0; i < 14; ++i) {
//     q(7 + i) = current_q_[i];
//   }

//   // --- 2. 順運動学(FK)と標準ヤコビアンの計算 ---
//   std::string target_frame = "limb_1_link_ee";  // ※URDFの手先リンク名
//   pinocchio::SE3 ee_pose;
//   Eigen::MatrixXd J_full;
//   try {
//     ee_pose = kinematics_->solveFK(q, target_frame);
//     J_full = kinematics_->computeJacobian(q, target_frame);
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR_ONCE(this->get_logger(), "Error: %s", e.what());
//     return;
//   }

//   // --- 3. 左腕のヤコビアンの抽出 ---
//   Eigen::MatrixXd J_m = J_full.rightCols(14);
//   Eigen::MatrixXd J_left = J_m.middleCols(0, 7);

//   // --- 4. 目標手先速度 (ベース座標系基準) の設定 ---
//   static double t = 0.0;
//   t += 0.01;  // 100Hz
//   Eigen::VectorXd v_target_base = Eigen::VectorXd::Zero(6);
//   // Z方向(腕の長手方向)に振幅5cmでゆっくり往復運動させる
//   v_target_base(2) = 0.5 * std::sin(2.0 * M_PI * 0.5 * t);

//   // --- 5. 目標速度をLOCAL座標系に変換 (★爆発回避の要) ---
//   // PinocchioのLOCALヤコビアンと整合させるため、ワールド目標速度を回転行列でローカルに変換
//   Eigen::Matrix3d R = ee_pose.rotation();
//   Eigen::VectorXd v_target_local = Eigen::VectorXd::Zero(6);
//   v_target_local.head<3>() = R.transpose() * v_target_base.head<3>();
//   v_target_local.tail<3>() = R.transpose() * v_target_base.tail<3>();

//   // --- 6. 逆運動学 (IK) の計算 (DLS法) ---
//   double lambda = 0.0;  // 0.1のダンピングで特異点付近の計算爆発を防ぐ
//   Eigen::MatrixXd A =
//     J_left * J_left.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
//   Eigen::VectorXd q_dot_cmd_left = J_left.transpose() * A.inverse() * v_target_local;

//   // --- 7. 指令角度の更新 (積分) ---
//   double dt = 0.01;
//   for (int i = 0; i < 7; ++i) {
//     // 速度を ±2.0 rad/s にクリップして異常な飛びを防止
//     double safe_cmd = std::clamp(q_dot_cmd_left(i), -200.0, 200.0);
//     // current_q_ ではなく target_q_ に足し込むことでチャタリングを防止
//     target_q_[i] += safe_cmd * dt;
//   }

//   // --- 8. MuJoCoへPublish ---
//   std_msgs::msg::Float64MultiArray cmd_msg;
//   cmd_msg.data = target_q_;
//   cmd_pub_->publish(cmd_msg);
// }

void GJControl::controlLoop()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!is_initialized_) return;

  // --- 1. 一般化座標 q の構築 ---
  Eigen::VectorXd q = Eigen::VectorXd::Zero(21);
  q(6) = 1.0;
  for (int i = 0; i < 14; ++i) {
    q(7 + i) = current_q_[i];
  }

  // --- 2. 順運動学(FK)と両腕の一般化ヤコビアンの計算 ---
  std::string frame_L = "limb_1_link_ee";  // 左腕の手先リンク名
  std::string frame_R = "limb_2_link_ee";  // 右腕の手先リンク名 (※要確認)

  pinocchio::SE3 pose_L, pose_R;
  Eigen::MatrixXd J_gen_L, J_gen_R;
  try {
    pose_L = kinematics_->solveFK(q, frame_L);
    J_gen_L = dynamics_->computeGeneralizedJacobian(q, frame_L);

    pose_R = kinematics_->solveFK(q, frame_R);
    J_gen_R = dynamics_->computeGeneralizedJacobian(q, frame_R);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Error: %s", e.what());
    return;
  }

  // --- 3. ヤコビアンの結合 (12行 x 14列) ---
  Eigen::MatrixXd J_stacked(12, 14);
  J_stacked << J_gen_L, J_gen_R;

  // --- 4. 目標速度 (ワールド基準) の設定 ---
  static double t = 0.0;
  t += 0.01;  // 100Hz

  // 左腕：Z方向に振幅5cmの往復運動
  Eigen::VectorXd v_target_base_L = Eigen::VectorXd::Zero(6);
  v_target_base_L(2) = 0.5 * std::sin(2.0 * M_PI * 0.5 * t);

  // 右腕：空間上で「停止」させたいのでゼロ
  Eigen::VectorXd v_target_base_R = Eigen::VectorXd::Zero(6);

  // --- 5. 目標速度を各ローカル座標系に変換 ---
  Eigen::Matrix3d R_L = pose_L.rotation();
  Eigen::VectorXd v_target_local_L = Eigen::VectorXd::Zero(6);
  v_target_local_L.head<3>() = R_L.transpose() * v_target_base_L.head<3>();
  v_target_local_L.tail<3>() = R_L.transpose() * v_target_base_L.tail<3>();

  // ※ゼロベクトルなので変換しなくてもゼロですが、将来的な拡張のために型を揃えます
  Eigen::Matrix3d R_R = pose_R.rotation();
  Eigen::VectorXd v_target_local_R = Eigen::VectorXd::Zero(6);
  v_target_local_R.head<3>() = R_R.transpose() * v_target_base_R.head<3>();
  v_target_local_R.tail<3>() = R_R.transpose() * v_target_base_R.tail<3>();

  // --- 6. 目標速度ベクトルの結合 (12次元) ---
  Eigen::VectorXd v_stacked = Eigen::VectorXd::Zero(12);
  v_stacked.head<6>() = v_target_local_L;
  v_stacked.tail<6>() = v_target_local_R;

  // --- 7. 全身逆運動学 (IK) の計算 (DLS法) ---
  // J_stacked(12x14) を使って、一気に14関節の指令速度 q_dot_cmd_all を解く
  double lambda = 0.1;
  Eigen::MatrixXd A =
    J_stacked * J_stacked.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(12, 12);
  Eigen::VectorXd q_dot_cmd_all = J_stacked.transpose() * A.inverse() * v_stacked;

  // --- 8. 指令角度の更新 (全14関節) ---
  double dt = 0.01;
  for (int i = 0; i < 14; ++i) {  // 0~6: 左腕, 7~13: 右腕
    double safe_cmd = std::clamp(q_dot_cmd_all(i), -20.0, 20.0);
    target_q_[i] += safe_cmd * dt;
  }

  // --- 9. MuJoCoへPublish ---
  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data = target_q_;
  cmd_pub_->publish(cmd_msg);
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::GJControl)
