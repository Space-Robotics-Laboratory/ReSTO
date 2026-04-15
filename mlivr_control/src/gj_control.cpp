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
  // Publisher
  cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_cmds", 10);
  ee_path_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("planned_trajectory", 10);

  // Subscriber
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&GJControl::jointStateCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&GJControl::odomCallback, this, std::placeholders::_1));
  trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/start_gj", 10, std::bind(&GJControl::triggerCallback, this, std::placeholders::_1));

  // Timer
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

  current_base_pose_ = Eigen::VectorXd::Zero(7);
  current_base_pose_(6) = 1.0;

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_transformer_ = std::make_unique<coordinate_transformer::CoordinateTransformer>(this);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

void GJControl::publishCommandStep(const std::vector<double> & q)
{
  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = this->now();

  for (int i = 0; i < num_joints_; ++i) {
    cmd_msg.name.push_back(robot_core_->getModel().names[i + 2]);
    cmd_msg.position.push_back(q[i]);
    cmd_msg.velocity.push_back(0.0);
    cmd_msg.effort.push_back(0.0);
  }

  cmd_pub_->publish(cmd_msg);
}

void GJControl::publishTrajectoryMarker()
{
  if (!pos_spline_) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = this->now();
  marker.ns = "gj_planned_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;  // Line width

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  double dt = 0.05;
  for (double t = 0; t <= duration_; t += dt) {
    // ★ 変更: スプラインから取得できる位置が既にワールド座標
    Eigen::Vector3d world_pos = pos_spline_->getPosition(t);

    geometry_msgs::msg::Point p;
    p.x = world_pos.x();
    p.y = world_pos.y();
    p.z = world_pos.z();
    marker.points.push_back(p);
  }

  ee_path_marker_pub_->publish(marker);
  RCLCPP_INFO(this->get_logger(), "Published GJ spline trajectory marker to RViz2.");
}

void GJControl::publishTargetTF(double t)
{
  Eigen::Vector3d p_target = pos_spline_->getPosition(t);
  Eigen::Quaterniond q_target = ori_spline_->getOrientation(t);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = "planned/" + ee_frames_[1];  // wb_controlの命名規則に合わせる

  tf_msg.transform.translation.x = p_target.x();
  tf_msg.transform.translation.y = p_target.y();
  tf_msg.transform.translation.z = p_target.z();
  tf_msg.transform.rotation.x = q_target.x();
  tf_msg.transform.rotation.y = q_target.y();
  tf_msg.transform.rotation.z = q_target.z();
  tf_msg.transform.rotation.w = q_target.w();

  tf_broadcaster_->sendTransform(tf_msg);
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

  if (!is_initialized_ && is_odom_received_) {
    if (is_triggered_) {
      target_q_ = current_q_;
      if (this->startTrajectory()) {
        is_initialized_ = true;
        publishTrajectoryMarker();
      }
    }
  }
}

void GJControl::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_base_pose_(0) = msg->pose.pose.position.x;
  current_base_pose_(1) = msg->pose.pose.position.y;
  current_base_pose_(2) = msg->pose.pose.position.z;
  current_base_pose_(3) = msg->pose.pose.orientation.x;
  current_base_pose_(4) = msg->pose.pose.orientation.y;
  current_base_pose_(5) = msg->pose.pose.orientation.z;
  current_base_pose_(6) = msg->pose.pose.orientation.w;
  is_odom_received_ = true;
}

void GJControl::triggerCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if (!is_triggered_ && !is_initialized_) {
    RCLCPP_INFO(this->get_logger(), "GJ Control Triggered! Generating spline trajectory...");
    is_triggered_ = true;
  }
}

void GJControl::controlLoop()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!is_initialized_) {
    return;
  }

  // --- 1. 一般化座標 q の構築 ---
  int nq = robot_core_->getModel().nq;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
  q.head(7) = current_base_pose_;
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

  // --- 3. ヤコビアンの結合 (12行 x num_joints_列) ---
  Eigen::MatrixXd J_stacked(12, num_joints_);
  J_stacked << J_gen_L, J_gen_R;

  // =======================================

  // --- 目標速度の計算 (フィードバックなし・開ループ) ---
  Eigen::VectorXd v_target_local_L = Eigen::VectorXd::Zero(6);  // 左腕は動かさない
  Eigen::VectorXd v_target_local_R = Eigen::VectorXd::Zero(6);

  if (is_trajectory_active_) {
    double t = this->now().seconds() - trajectory_start_time_;

    double t_eval = (t > duration_) ? duration_ : t;  // 終了後は最終目標値をキープ
    publishTargetTF(t_eval);

    if (t > duration_) {
      v_target_local_R = Eigen::VectorXd::Zero(6);
    } else {
      Eigen::VectorXd v_spline_world = Eigen::VectorXd::Zero(6);
      v_spline_world.head<3>() = pos_spline_->getVelocity(t);
      v_spline_world.tail<3>() = ori_spline_->getAngularVelocity(t);

      pinocchio::Motion v_ff_world(v_spline_world);

      // 2. 【空間速度の座標変換】 World座標系の速度を、limb_2 ローカル座標系に直接変換
      // pose_R はFKで計算済みの World -> limb_2 の姿勢
      pinocchio::Motion v_ff_local_R = pose_R.actInv(v_ff_world);

      v_target_local_R = v_ff_local_R.toVector();
    }
  }

  // --- 結合 ---
  Eigen::VectorXd v_stacked = Eigen::VectorXd::Zero(12);
  v_stacked.head<6>() = v_target_local_L;
  v_stacked.tail<6>() = v_target_local_R;

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

  publishCommandStep(target_q_);
}

bool GJControl::startTrajectory()
{
  int nq = robot_core_->getModel().nq;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
  q.head(7) = current_base_pose_;
  for (int i = 0; i < num_joints_; ++i) q(7 + i) = current_q_[i];

  // pose_L は軌道生成では使わなくなるため削除可能
  pinocchio::SE3 pose_R = kinematics_->solveFK(q, ee_frames_[1]);

  // ★ 変更: World座標系における現在の右手Poseを基準にする
  Eigen::Vector3d start_pos = pose_R.translation();
  Eigen::Quaterniond start_quat(pose_R.rotation());

  // ★ 変更: World座標系でのオフセット（wb_control と同じに合わせる）
  auto displacement = Eigen::Vector3d(0.0, -0.2, 0.0);

  Eigen::Vector3d target_pos = start_pos + displacement;
  Eigen::Quaterniond target_quat = start_quat;

  // ★ 変更: WorldのZ軸(上方向)への持ち上げ高さ
  Eigen::Vector3d swing_height = Eigen::Vector3d(0.0, 0.0, 0.05);
  Eigen::Vector3d mid_pos = start_pos + displacement / 2.0 + swing_height;
  Eigen::Quaterniond mid_quat = start_quat;

  // --- 3. 自作ライブラリによる軌道制約の作成 ---
  // double duration = 10.0;
  duration_ = 10.0;

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
