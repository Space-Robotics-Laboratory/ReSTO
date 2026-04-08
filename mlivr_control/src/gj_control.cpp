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

  ee_frames_ = this->declare_parameter<std::vector<std::string>>("ee_frames");

  std::string urdf_path =
    ament_index_cpp::get_package_share_directory("mlivr_description") + "/urdf/mlivr.urdf";
  robot_core_ = std::make_unique<mlivr_model::RobotCore>(urdf_path);
  kinematics_ = std::make_unique<mlivr_model::Kinematics>(*robot_core_);
  dynamics_ = std::make_unique<mlivr_model::Dynamics>(*robot_core_);

  num_joints_ = robot_core_->getModel().nv - 6;

  current_q_.resize(num_joints_, 0.0);
  current_v_.resize(num_joints_, 0.0);
  target_q_.resize(num_joints_, 0.0);
  is_initialized_ = false;

  tf_transformer_ = std::make_unique<coordinate_transformer::CoordinateTransformer>(this);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

void GJControl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < static_cast<size_t>(num_joints_) && i < msg->position.size(); ++i) {
    current_q_[i] = msg->position[i];

    if (i < msg->velocity.size()) {
      current_v_[i] = msg->velocity[i];
    } else {
      current_v_[i] = 0.0;
    }
  }

  if (!is_initialized_) {
    target_q_ = current_q_;

    if (this->startTrajectory()) {
      is_initialized_ = true;
    }
  }
}

void GJControl::controlLoop()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!is_initialized_) return;

  // --- 1. 一般化座標 q の構築 ---
  int nq = robot_core_->getModel().nq;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
  q(6) = 1.0;
  for (int i = 0; i < num_joints_; ++i) {
    q(7 + i) = current_q_[i];
  }

  // --- 2. 順運動学(FK)と両腕の一般化ヤコビアンの計算 ---
  std::string frame_L = ee_frames_[0];  // 左腕の手先リンク名
  std::string frame_R = ee_frames_[1];  // 右腕の手先リンク名 (※要確認)

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

  pinocchio::SE3 current_pose_R_in_L = pose_L.actInv(pose_R);
  Eigen::Vector3d p = current_pose_R_in_L.translation();

  // 1秒に1回くらいプリントして確認
  // static int log_counter = 0;
  // if (log_counter++ % 100 == 0) {
  //   RCLCPP_INFO(
  //     this->get_logger(), "Right Arm Position from Left Arm: [x: %.3f, y: %.3f, z: %.3f]", p.x(),
  //     p.y(), p.z());
  // }

  // --- 3. ヤコビアンの結合 (12行 x num_joints_列) ---
  Eigen::MatrixXd J_stacked(12, num_joints_);
  J_stacked << J_gen_L, J_gen_R;

  // =======================================

  // --- 目標速度の計算 (フィードバックなし・開ループ) ---
  Eigen::VectorXd v_target_local_L = Eigen::VectorXd::Zero(6);  // 左腕は動かさない
  Eigen::VectorXd v_target_local_R = Eigen::VectorXd::Zero(6);

  if (is_trajectory_active_) {
    double t = this->now().seconds() - trajectory_start_time_;
    // double duration = 10.0;  // ★ startTrajectory() と時間を合わせる！

    if (t > duration_) {
      // ★ 軌道終了後は確実に速度をゼロにする
      v_target_local_R = Eigen::VectorXd::Zero(6);
    } else {
      // 1. スプラインから現在時刻の【目標速度】を取得 (すべて limb_1 座標系)
      Eigen::VectorXd v_spline = Eigen::VectorXd::Zero(6);
      v_spline.head<3>() = pos_spline_->getVelocity(t);
      v_spline.tail<3>() = ori_spline_->getAngularVelocity(t);

      // PinocchioのMotion型（空間速度）に変換
      pinocchio::Motion v_ff_L1(v_spline);

      // 2. 現在の手先 Pose (limb_1 から見た limb_2)
      // ※pose_L, pose_R は上で計算済みの solveFK の結果
      pinocchio::SE3 L1_to_R = pose_L.actInv(pose_R);

      // 3. 【空間速度の座標変換】 limb_1 座標系の速度を、limb_2 ローカル座標系に直接変換
      // actInv() は回転行列だけでなく、2点間の並進オフセットによる遠心力・コリオリ効果(ω x r)も正しく処理します
      pinocchio::Motion v_ff_local_R = L1_to_R.actInv(v_ff_L1);

      // フィードバックを使わず、純粋なスプライン速度のみを指令値とする
      v_target_local_R = v_ff_local_R.toVector();
    }
  }

  // --- 結合 ---
  Eigen::VectorXd v_stacked = Eigen::VectorXd::Zero(12);
  v_stacked.head<6>() = v_target_local_L;
  v_stacked.tail<6>() = v_target_local_R;

  // ======================================

  // // --- 目標速度と位置フィードバックの計算 ---
  // Eigen::VectorXd v_target_local_L = Eigen::VectorXd::Zero(6);  // 左腕(Weld側)は動かさない
  // Eigen::VectorXd v_target_local_R = Eigen::VectorXd::Zero(6);

  // if (is_trajectory_active_) {
  //   double t = this->now().seconds() - trajectory_start_time_;
  //   if (t > 5.0) t = 5.0;  // 5秒で停止

  //   // 1. スプラインから現在時刻の【目標位置・姿勢・速度】を取得 (すべて limb_1 座標系)
  //   Eigen::Vector3d p_target = pos_spline_->getPosition(t);
  //   Eigen::Quaterniond q_target = ori_spline_->getOrientation(t);
  //   pinocchio::SE3 SE3_target(q_target.toRotationMatrix(), p_target);

  //   // limb_1 座標系における目標空間速度 (Motion)
  //   pinocchio::Motion v_ff_L1(pos_spline_->getVelocity(t), ori_spline_->getAngularVelocity(t));

  //   // 2. 現在の手先 Pose (limb_1 座標系から見た limb_2)
  //   pinocchio::SE3 L1_to_R = pose_L.actInv(pose_R);

  //   // 3. 【空間速度の座標変換】 limb_1 座標系の速度を、limb_2 ローカル座標系に変換
  //   // L1_to_R.actInv() は、空間速度ベクトルを正確に別のフレームに投影します
  //   pinocchio::Motion v_ff_local = L1_to_R.actInv(v_ff_L1);

  //   // 4. 【フィードバック計算】 limb_2 ローカル座標系の誤差空間速度
  //   pinocchio::Motion error_motion = pinocchio::log6(L1_to_R.actInv(SE3_target));

  //   // 5. 最終的な指令速度 = フィードフォワード + フィードバック (Kp)
  //   double Kp = 10.0;  // ゲインを高めに設定して追従性を上げる
  //   v_target_local_R = v_ff_local.toVector() + Kp * error_motion.toVector();
  // }

  // // --- 結合 ---
  // Eigen::VectorXd v_stacked = Eigen::VectorXd::Zero(12);
  // v_stacked.head<6>() = v_target_local_L;
  // v_stacked.tail<6>() = v_target_local_R;

  // =======================================

  // --- IK計算と角度更新 ---
  double lambda = 0.0;
  Eigen::MatrixXd A =
    J_stacked * J_stacked.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(12, 12);
  Eigen::VectorXd q_dot_cmd_all = J_stacked.transpose() * A.inverse() * v_stacked;

  double dt = 0.01;
  for (int i = 0; i < num_joints_; ++i) {
    double safe_cmd = std::clamp(q_dot_cmd_all(i), -200.0, 200.0);
    target_q_[i] += safe_cmd * dt;
  }

  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data = target_q_;
  cmd_pub_->publish(cmd_msg);
}

bool GJControl::startTrajectory()
{
  // --- 1. 目標把持点の取得 (TF使用) ---
  std::string target_site = "seattrack_1_site_1";  // 右手が向かう目標
  std::string base_frame = "limb_1_gripper_site";  // 左手(固定端)を基準座標とする

  // tf_transformer_を使って、左手基準の目標位置を取得
  auto tf_msg_opt = tf_transformer_->getTransformMsg(target_site, base_frame);
  if (!tf_msg_opt) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Waiting for TF: %s -> %s", target_site.c_str(),
      base_frame.c_str());
    return false;
  }
  auto tf_msg = tf_msg_opt.value();
  Eigen::Vector3d target_pos(
    tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z);
  Eigen::Quaterniond target_quat(
    tf_msg.transform.rotation.w, tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
    tf_msg.transform.rotation.z);

  // --- 2. 現在の手先位置の取得 (FK使用) ---
  int nq = robot_core_->getModel().nq;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
  q(6) = 1.0;
  for (int i = 0; i < num_joints_; ++i) q(7 + i) = current_q_[i];

  pinocchio::SE3 pose_L = kinematics_->solveFK(q, ee_frames_[0]);
  pinocchio::SE3 pose_R = kinematics_->solveFK(q, ee_frames_[1]);

  // 左手座標系から見た右手(limb_2)の現在のPose
  pinocchio::SE3 current_pose_R_in_L = pose_L.actInv(pose_R);
  Eigen::Vector3d start_pos = current_pose_R_in_L.translation();
  Eigen::Quaterniond start_quat(current_pose_R_in_L.rotation());

  auto displacement = Eigen::Vector3d(0.0, -0.2, -0.0);

  target_pos = start_pos + displacement;
  target_quat = start_quat;

  Eigen::Vector3d swing_height = Eigen::Vector3d(0.0, 0.0, -0.05);
  Eigen::Vector3d mid_pos = start_pos + displacement / 2.0 + swing_height;
  Eigen::Quaterniond mid_quat = start_quat;

  // --- 3. 自作ライブラリによる軌道制約の作成 ---
  // double duration = 10.0;
  duration_ = 20.0;

  // 位置の制約 (開始0秒で速度0、終了5秒で速度0)
  trajectory_generator::VectorStateConstraint start_p_c{
    0.0, start_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  trajectory_generator::VectorStateConstraint end_p_c{
    duration_, target_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  auto pos_constraints = trajectory_generator::createBoundaryConditions(start_p_c, end_p_c);

  trajectory_generator::VectorStateConstraint mid_p_c{duration_ / 2.0, mid_pos};
  trajectory_generator::addConstraint(pos_constraints, mid_p_c);

  // 姿勢の制約
  trajectory_generator::AngularStateConstraint start_o_c{
    0.0, start_quat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  trajectory_generator::AngularStateConstraint end_o_c{
    duration_, target_quat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  auto ori_constraints = trajectory_generator::createBoundaryConditions(start_o_c, end_o_c);

  trajectory_generator::AngularStateConstraint mid_o_c{duration_ / 2.0, mid_quat};
  trajectory_generator::addConstraint(ori_constraints, mid_o_c);

  // --- 4. スプラインの生成 ---
  pos_spline_ = std::make_unique<trajectory_generator::VectorSpline>(pos_constraints, 3);
  ori_spline_ = std::make_unique<trajectory_generator::OrientationSpline>(ori_constraints);

  trajectory_start_time_ = this->now().seconds();
  is_trajectory_active_ = true;
  RCLCPP_INFO(this->get_logger(), "Trajectory generation completed. Started tracking.");

  return true;  // ★追加: 成功したことをコールバックに伝える
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::GJControl)
