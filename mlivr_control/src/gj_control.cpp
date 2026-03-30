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

//   // --- 1. 一般化座標 q (サイズ21: 浮遊ベース7 + 関節14) の構築 ---
//   Eigen::VectorXd q = Eigen::VectorXd::Zero(7 + kTotalNumJoints);
//   q(6) = 1.0;  // w
//   for (int i = 0; i < 14; ++i) {
//     q(7 + i) = current_q_[i];
//   }

//   // // --- 2. 一般化ヤコビアンの計算 ---
//   // // ※ "left_end_effector" の部分は実際のURDFのフレーム名に合わせて変更してください
//   // std::string target_frame = "limb_1_link_gripper";
//   // Eigen::MatrixXd J_gen;
//   // try {
//   //   J_gen = dynamics_->computeGeneralizedJacobian(q, target_frame);
//   // } catch (const std::exception & e) {
//   //   RCLCPP_ERROR_ONCE(this->get_logger(), "Error: %s", e.what());
//   //   return;
//   // }

//   // // --- 3. ヤコビアンの分割 (左腕 0~6, 右腕 7~13) ---
//   // Eigen::MatrixXd J_g_left = J_gen.middleCols(0, 7);
//   // Eigen::MatrixXd J_g_right = J_gen.middleCols(7, 7);

//   // // --- 4. 他腕 (右腕) の速度の影響を計算 ---
//   // Eigen::VectorXd q_dot_right = Eigen::Map<Eigen::VectorXd>(current_v_.data() + 7, 7);
//   // Eigen::VectorXd v_disturb = J_g_right * q_dot_right;

//   // // --- 5. 目標手先速度 (Spatial Velocity) の設定 ---
//   // // テストとして、手先をローカルのZ方向に 0.05 m/s で動かす指令
//   // Vector6d v_target = Vector6d::Zero();
//   // v_target(2) = 0.05;  // [vx, vy, vz, wx, wy, wz]

//   // // 他腕の影響を差し引いた、真に必要な手先速度
//   // Eigen::VectorXd modified_v_target = v_target - v_disturb;

//   // // --- 6. 分解速度制御 (IK) の計算 (Damped Least Squares) ---
//   // double lambda = 0.01;  // ダンピング係数
//   // Eigen::MatrixXd A =
//   //   J_g_left * J_g_left.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
//   // Eigen::VectorXd q_dot_cmd_left = J_g_left.transpose() * A.inverse() * modified_v_target;

//   // // --- 7. 指令角度の更新 (積分) ---
//   // double dt = 0.01;  // 100Hz
//   // for (int i = 0; i < 7; ++i) {
//   //   target_q_[i] += q_dot_cmd_left(i) * dt;  // 左腕を更新
//   //   // target_q_[7+i] は初期位置をキープ (右腕)
//   // }

//   // -- -2. 標準ヤコビアン(J_full) の計算-- -
//   std::string target_frame = "limb_1_link_gripper";
//   Eigen::MatrixXd J_full;
//   try {
//     // dynamics_ ではなく kinematics_ を使って空間ヤコビアン(6 x nv)を取得
//     J_full = kinematics_->computeJacobian(q, target_frame);
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR_ONCE(this->get_logger(), "Error: %s", e.what());
//     return;
//   }

//   // --- 3. ヤコビアンの抽出と分割 ---
//   // Pinocchioのヤコビアンは 6 x nv (浮遊ベース6自由度 + 関節14自由度 = 20列)
//   // 関節部分 (J_m) は右側の14列
//   Eigen::MatrixXd J_m = J_full.rightCols(14);

//   // 左腕 (0~6列目) の標準ヤコビアン
//   Eigen::MatrixXd J_left = J_m.middleCols(0, 7);

//   // --- 4. 目標手先速度 (Spatial Velocity) の設定 ---
//   static double t = 0.0;
//   t += 0.01;  // 100Hzの制御周期
//   Eigen::VectorXd v_target = Eigen::VectorXd::Zero(6);
//   v_target(2) = 0.05 * std::sin(2.0 * M_PI * 0.5 * t);  // 振幅0.05m/s, 0.5HzでZ方向を往復

//   // 標準ヤコビアンでは他腕の影響(v_disturb)は考慮しないため、そのまま使う

//   // --- 5. 分解速度制御 (IK) の計算 (Damped Least Squares) ---
//   double lambda = 0.05;  // ダンピング係数
//   Eigen::MatrixXd A =
//     J_left * J_left.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
//   Eigen::VectorXd q_dot_cmd_left = J_left.transpose() * A.inverse() * v_target;

//   // --- 6. 指令角度の更新 (積分) ---
//   double dt = 0.01;  // 100Hz
//   for (int i = 0; i < 7; ++i) {
//     // 速度クリップ
//     double safe_cmd = std::clamp(q_dot_cmd_left(i), -2.0, 2.0);
//     // 現在位置を基準に積分
//     target_q_[i] = current_q_[i] + safe_cmd * dt;
//   }

//   // // --- 5. 目標手先速度 (Spatial Velocity) の設定 ---
//   // // 常に一定方向に進むとすぐ関節の限界（特異点）に到達するため、サイン波で往復運動させます
//   // static double t = 0.0;
//   // t += 0.01;  // 100Hzの制御周期
//   // Eigen::VectorXd v_target = Eigen::VectorXd::Zero(6);
//   // v_target(2) = 0.05 * std::sin(2.0 * M_PI * 0.5 * t);  // 振幅0.05m/s, 0.5HzでZ方向を往復

//   // // 他腕の影響を差し引いた、真に必要な手先速度
//   // Eigen::VectorXd modified_v_target = v_target - v_disturb;

//   // // --- 6. 分解速度制御 (IK) の計算 (Damped Least Squares) ---
//   // double lambda = 0.05;  // ダンピング係数を少し強め(0.01 -> 0.05)にして計算をさらに安定化
//   // Eigen::MatrixXd A =
//   //   J_g_left * J_g_left.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
//   // Eigen::VectorXd q_dot_cmd_left = J_g_left.transpose() * A.inverse() * modified_v_target;

//   // // --- 7. 指令角度の更新 (積分) ---
//   // double dt = 0.01;  // 100Hz
//   // for (int i = 0; i < 7; ++i) {
//   //   // 安全装置1: 速度が大きすぎる場合はクリップ（±2.0 rad/s）
//   //   double safe_cmd = std::clamp(q_dot_cmd_left(i), -2.0, 2.0);

//   //   // 安全装置2: 積分ワインドアップ（誤差の蓄積）を防ぐため、常に「現在位置」を基準に指令
//   //   target_q_[i] = current_q_[i] + safe_cmd * dt;
//   // }

//   // --- 8. MuJoCoへPublish ---
//   std_msgs::msg::Float64MultiArray cmd_msg;
//   cmd_msg.data = target_q_;
//   cmd_pub_->publish(cmd_msg);
// }

void GJControl::controlLoop()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!is_initialized_) return;

  // 経過時間の更新 (100Hz = 0.01秒)
  static double t = 0.0;
  t += 0.01;

  // 【フェーズ1のテスト内容】
  // 左腕の最初の関節（インデックス0）だけを、ゆっくりとしたサイン波で動かす
  // 振幅: 0.2 rad (約11度), 周波数: 0.5 Hz (2秒で1往復)
  double target_angle = 1.0 * std::sin(2.0 * M_PI * 0.5 * t);

  // 初回に target_q_ = current_q_ されているため、
  // 他の関節(1〜13)はそのまま初期姿勢をキープする指令になります
  target_q_[0] += target_angle *
    0.01;  // 少しずつ角度を足し込む(あるいは直接代入: target_q_[0] = init_q + target_angle;)

  // ※より安全にするため、今回は直接代入方式にします
  // target_q_[0] は current_q_ で初期化された初期角度を中心に振幅します
  static double initial_q0 = current_q_[0];
  target_q_[0] = initial_q0 + target_angle;

  // MuJoCoへPublish
  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data = target_q_;
  cmd_pub_->publish(cmd_msg);
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::GJControl)
