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

#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "mlivr_sim/mujoco_engine.hpp"
#include "mlivr_sim/mujoco_renderer.hpp"

namespace mlivr_sim
{

class MujocoRosNode : public rclcpp::Node
{
public:
  explicit MujocoRosNode(const rclcpp::NodeOptions & options);
  ~MujocoRosNode();

private:
  void publishJointStates(const rclcpp::Time & now);
  void publishFTSensorData(const rclcpp::Time & now);
  void broadcastSiteTransforms(const rclcpp::Time & now);

  void jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void simLoop();

  // --- Core Component ---
  std::unique_ptr<MujocoEngine> engine_;
  std::unique_ptr<MujocoRenderer> renderer_;

  std::thread sim_thread_;
  bool is_running_ = false;

  int num_limbs_ = 2;

  // --- ROS 2 Interfaces ---
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> ee_ft_pubs_;
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace mlivr_sim
