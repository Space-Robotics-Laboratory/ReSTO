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

#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/multibody/data.hpp>
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

  std::shared_ptr<crocoddyl::ActionModelAbstract> createActionModel(
    const Eigen::VectorXd & x0, const std::string & fixed_frame, const pinocchio::SE3 & fixed_pose,
    const std::string & swing_frame, const pinocchio::SE3 & target_pose);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<mlivr_model::RobotCore> robot_core_;
  std::shared_ptr<pinocchio::Model> model_ptr_;
  std::shared_ptr<pinocchio::Data> data_ptr_;

  std::shared_ptr<crocoddyl::StateMultibody> state_;
  std::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;

  std::vector<double> current_q_;
  bool is_initialized_ = false;

  std::vector<Eigen::VectorXd> optimized_xs_;
  size_t playback_idx_ = 0;
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__WB_CONTROL_HPP_
