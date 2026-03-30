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

#ifndef MLIVR_CONTROL__GJ_CONTROL_HPP_
#define MLIVR_CONTROL__GJ_CONTROL_HPP_

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "mlivr_control/types.hpp"
#include "mlivr_control/visibility_control.hpp"
#include "mlivr_model/core.hpp"
#include "mlivr_model/dynamics.hpp"
#include "mlivr_model/kinematics.hpp"

namespace mlivr_control
{

class GJControl : public rclcpp::Node
{
public:
  MLIVR_CONTROL_PUBLIC
  explicit GJControl(const rclcpp::NodeOptions & options);

  virtual ~GJControl() = default;

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void controlLoop();

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<mlivr_model::RobotCore> robot_core_;
  std::unique_ptr<mlivr_model::Kinematics> kinematics_;
  std::unique_ptr<mlivr_model::Dynamics> dynamics_;

  std::vector<double> current_q_;
  std::vector<double> current_v_;
  std::vector<double> target_q_;
  bool is_initialized_;
  std::mutex state_mutex_;

  const int kNumJoints = 7;
  const int kTotalNumJoints = 14;
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__GJ_CONTROL_HPP_
