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

#include "mlivr_control/base_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mlivr_control
{

BaseController::BaseController(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  ee_frames_ = this->declare_parameter<std::vector<std::string>>("ee_frames");

  std::string urdf_path =
    ament_index_cpp::get_package_share_directory("mlivr_description") + "/urdf/mlivr.urdf";
  robot_ = std::make_unique<fbml::RobotCore>(urdf_path, Eigen::Vector3d::Zero());

  robot_->setActuatorParameters(
    this->declare_parameter<double>("mj.armature"), this->declare_parameter<double>("mj.damping"));

  num_joints_ = robot_->getModel().nv - 6;

  // Publisher
  cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_cmds", 10);
  ee_path_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("/planned_trajectory", 10);

  // Subscriber
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&BaseController::jointStateCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&BaseController::odomCallback, this, std::placeholders::_1));
  trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/start_control", 10, std::bind(&BaseController::triggerCallback, this, std::placeholders::_1));

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&BaseController::timerCallback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  current_joint_pos_.resize(num_joints_, 0.0);
  current_base_pose_ = Eigen::VectorXd::Zero(7);
  current_base_pose_(6) = 1.0;  // Quaternion w
  current_base_twist_ = Eigen::VectorXd::Zero(6);
}

void BaseController::publishTargetTF(
  const Eigen::Vector3d & translation, const Eigen::Quaterniond & rotation,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = child_frame_id;

  tf_msg.transform.translation.x = translation.x();
  tf_msg.transform.translation.y = translation.y();
  tf_msg.transform.translation.z = translation.z();

  tf_msg.transform.rotation.x = rotation.x();
  tf_msg.transform.rotation.y = rotation.y();
  tf_msg.transform.rotation.z = rotation.z();
  tf_msg.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(tf_msg);
}

void BaseController::publishTrajectoryMarker(const std::vector<Eigen::Vector3d> & path_points)
{
  if (path_points.empty()) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = this->now();
  marker.ns = this->get_name();
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;  // Line width

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  for (const auto & pos : path_points) {
    geometry_msgs::msg::Point p;
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
    marker.points.push_back(p);
  }

  ee_path_marker_pub_->publish(marker);
}

void BaseController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (current_joint_pos_.size() != msg->position.size()) {
    current_joint_pos_.resize(msg->position.size(), 0.0);
  }

  for (size_t i = 0; i < msg->position.size(); ++i) {
    current_joint_pos_[i] = msg->position[i];
  }
}

void BaseController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_base_pose_(0) = msg->pose.pose.position.x;
  current_base_pose_(1) = msg->pose.pose.position.y;
  current_base_pose_(2) = msg->pose.pose.position.z;
  current_base_pose_(3) = msg->pose.pose.orientation.x;
  current_base_pose_(4) = msg->pose.pose.orientation.y;
  current_base_pose_(5) = msg->pose.pose.orientation.z;
  current_base_pose_(6) = msg->pose.pose.orientation.w;

  current_base_twist_(0) = msg->twist.twist.linear.x;
  current_base_twist_(1) = msg->twist.twist.linear.y;
  current_base_twist_(2) = msg->twist.twist.linear.z;
  current_base_twist_(3) = msg->twist.twist.angular.x;
  current_base_twist_(4) = msg->twist.twist.angular.y;
  current_base_twist_(5) = msg->twist.twist.angular.z;

  is_odom_received_ = true;
}

void BaseController::triggerCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;

  if (!is_triggered_ && !is_initialized_) {
    is_triggered_ = true;

    if (this->generateTrajectory()) {
      is_initialized_ = true;

      auto path_points = this->getPlannedPath();
      publishTrajectoryMarker(path_points);
    }
  }
}

void BaseController::timerCallback()
{
  if (!is_initialized_) {
    this->publishWaitingState();
    return;
  }

  this->computeCommandStep();
}

}  // namespace mlivr_control
