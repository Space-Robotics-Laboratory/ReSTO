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

#include "trajectory_generator/spline.hpp"
#include "trajectory_generator/trajectory_generator.hpp"
#include <rclcpp/rclcpp.hpp>

#include "mlivr_control/base_controller.hpp"
#include "mlivr_control/visibility_control.h"
#include "mlivr_model/dynamics.hpp"
#include "mlivr_model/kinematics.hpp"

namespace mlivr_control
{

class GJControl : public BaseController
{
public:
  MLIVR_CONTROL_PUBLIC
  explicit GJControl(const rclcpp::NodeOptions & options);
  virtual ~GJControl() = default;

protected:
  bool generateTrajectory() override;

  Eigen::VectorXd computeCommandStep() override;

  std::vector<Eigen::Vector3d> getPlannedPath() override;

private:
  std::unique_ptr<mlivr_model::Kinematics> kinematics_;
  std::unique_ptr<mlivr_model::Dynamics> dynamics_;

  std::vector<double> target_joint_pos_;

  std::unique_ptr<trajectory_generator::VectorSpline> pos_spline_;
  std::unique_ptr<trajectory_generator::OrientationSpline> ori_spline_;
  double duration_;
  double trajectory_start_time_ = 0.0;
  bool is_trajectory_active_ = false;
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__GJ_CONTROL_HPP_
