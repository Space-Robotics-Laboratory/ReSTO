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

#ifndef MLIVR_CONTROL__BASE_CONTROLLER_HPP_
#define MLIVR_CONTROL__BASE_CONTROLLER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "mlivr_control/types.hpp"
#include "mlivr_control/visibility_control.hpp"
#include "mlivr_model/core.hpp"

namespace mlivr_control
{

class BaseController : public rclcpp::Node
{
public:
  MLIVR_CONTROL_PUBLIC
  explicit BaseController(const std::string & node_name, const rclcpp::NodeOptions & options);
  virtual ~BaseController() = default;

protected:
  virtual bool generateTrajectory() = 0;

  virtual Eigen::VectorXd computeCommandStep() = 0;

  virtual std::vector<Eigen::Vector3d> getPlannedPath() = 0;

  virtual void publishWaitingState() {}

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ee_path_marker_pub_;
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<mlivr_model::RobotCore> robot_core_;

  Eigen::VectorXd current_base_pose_;   // [x, y, z, qx, qy, qz, qw]
  Eigen::VectorXd current_base_twist_;  // [vx, vy, vz, wx, wy, wz]
  std::vector<double> current_joint_pos_;
  std::mutex state_mutex_;

  int num_joints_;
  std::vector<std::string> ee_frames_;

  bool is_triggered_ = false;
  bool is_initialized_ = false;
  bool is_odom_received_ = false;

  Eigen::Vector3d target_offset_;

private:
  void publishTrajectoryMarker(const std::vector<Eigen::Vector3d> & path_points);

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void triggerCallback(const std_msgs::msg::Empty::SharedPtr msg);

  void timerCallback();
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__BASE_CONTROLLER_HPP_
