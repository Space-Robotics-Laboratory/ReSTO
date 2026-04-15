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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mlivr_control/base_controller.hpp"
#include "mlivr_control/visibility_control.hpp"
#include "mlivr_control/wbc_solver.hpp"

namespace mlivr_control
{

class WBControl : public BaseController
{
public:
  MLIVR_CONTROL_PUBLIC
  explicit WBControl(const rclcpp::NodeOptions & options);
  virtual ~WBControl() = default;

protected:
  void publishWaitingState() override;

  bool generateTrajectory() override;

  Eigen::VectorXd computeCommandStep() override;

  std::vector<Eigen::Vector3d> getPlannedPath() override;

private:
  void publishPlannedRobotState(const Eigen::VectorXd & q_all);

  std::shared_ptr<pinocchio::Model> model_ptr_;
  std::unique_ptr<WbcSolver> wbc_solver_;

  size_t playback_idx_ = 0;

  pinocchio::SE3 target_ee_pose_se3_;
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__WB_CONTROL_HPP_
