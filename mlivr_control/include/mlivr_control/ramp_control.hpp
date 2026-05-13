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

#ifndef MLIVR_CONTROL__RAMP_CONTROL_HPP_
#define MLIVR_CONTROL__RAMP_CONTROL_HPP_

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ramp/lrst/low_reaction_swing_trajectory.hpp"
#include "ramp/md/momentum_distribution.hpp"
#include "trajectory_generator/spline.hpp"
#include "trajectory_generator/trajectory_generator.hpp"
#include <fbml/dynamics.hpp>
#include <fbml/kinematics.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mlivr_control/base_controller.hpp"
#include "mlivr_control/visibility_control.h"

namespace mlivr_control
{

class RAMPControl : public BaseController
{
public:
  MLIVR_CONTROL_PUBLIC
  explicit RAMPControl(const rclcpp::NodeOptions & options);
  virtual ~RAMPControl() = default;

protected:
  bool generateTrajectory() override;

  Eigen::VectorXd computeCommandStep() override;

  std::vector<Eigen::Vector3d> getPlannedPath() override;

private:
  std::unique_ptr<fbml::Kinematics> kinematics_;
  std::unique_ptr<fbml::Dynamics> dynamics_;

  bool use_lrst_ = true;
  std::unique_ptr<ramp::lrst::LowReactionSwingTrajectory> lrst_optimizer_;
  ramp::lrst::SolverParams default_solver_params_;
  ramp::lrst::WeightParams default_weight_params_;
  Eigen::MatrixXd optimized_bezier_P_;

  std::string current_swing_ee_frame_;
  std::vector<std::string> current_swing_joints_;

  std::unique_ptr<ramp::md::MomentumDistribution> md_solver_;
  double momentum_distribution_factor_ = 0.5;

  std::vector<double> target_joint_pos_;

  std::unique_ptr<trajectory_generator::VectorSpline> pos_spline_;
  std::unique_ptr<trajectory_generator::OrientationSpline> ori_spline_;
  double duration_;
  double trajectory_start_time_ = 0.0;
  bool is_trajectory_active_ = false;
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__RAMP_CONTROL_HPP_
