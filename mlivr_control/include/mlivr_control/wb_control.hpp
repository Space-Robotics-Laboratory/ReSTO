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

#ifndef MLIVR_CONTROL__WB_CONTROL_HPP_
#define MLIVR_CONTROL__WB_CONTROL_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "mlivr_control/types.hpp"
#include "mlivr_control/visibility_control.hpp"
#include "mlivr_control/wbc_solver.hpp"
#include "mlivr_model/core.hpp"
#include "mlivr_model/dynamics.hpp"
#include "mlivr_model/kinematics.hpp"

namespace mlivr_control
{

class WBControl : public rclcpp::Node
{
public:
  MLIVR_CONTROL_PUBLIC
  explicit WBControl(const rclcpp::NodeOptions & options);
  virtual ~WBControl() = default;

private:
  void publishCommandStep();

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool computeTrajectory();

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<mlivr_model::RobotCore> robot_core_;
  std::shared_ptr<pinocchio::Model> model_ptr_;

  std::unique_ptr<WbcSolver> wbc_solver_;

  std::vector<double> current_q_;

  bool is_initialized_ = false;
  size_t playback_idx_ = 0;
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__WB_CONTROL_HPP_
