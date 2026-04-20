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

#ifndef RAMP__MD__LOW_REACTION_SWING_TRAJECTORY_HPP_
#define RAMP__MD__LOW_REACTION_SWING_TRAJECTORY_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "ramp/visibility_control.h"

#include <fbml/dynamics.hpp>
#include <fbml/kinematics.hpp>
#include <nlopt.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace ramp
{
namespace lrst
{

struct SolverParams
{
  double dt = 0.01;  // [s]

  double step_duration = 10.0;  // [s]
  double step_height = 0.05;    // [m]

  double relative_tol = 1e-4;
  int max_iter = 1000;
};

struct WeightParams
{
  double force_max = 1.0;
  double moment_max = 1.0;

  double step_height_max = 1.0;
  double step_height_ave = 1.0;
};

class RAMP_PUBLIC LowReactionSwingTrajectory
{
public:
  explicit LowReactionSwingTrajectory(
    fbml::Kinematics * kinematics, fbml::Dynamics * dynamics, int num_joints, int num_limbs);
  virtual ~LowReactionSwingTrajectory() = default;

  Eigen::MatrixXd optimizeTrajectory(
    const SolverParams & solver_params, const WeightParams & weight_params);

  void setBoundaryConditions(const Eigen::Vector3d & start_pos, const Eigen::Vector3d & end_pos);

  void setRobotState(
    const Eigen::VectorXd & q_init, const std::string & swing_frame_name,
    const std::vector<std::string> & swing_joint_names);

  Eigen::Vector3d computeBezierPosition(double t, const Eigen::MatrixXd & P) const;
  Eigen::Vector3d computeBezierVelocity(double t, const Eigen::MatrixXd & P) const;

private:
  static double objectiveWrapper(
    const std::vector<double> & x, std::vector<double> & grad, void * data);

  double computeCost(const std::vector<double> & x);

  SolverParams solver_params_;
  WeightParams weight_params_;

  const int bezier_order_ = 7;
  Eigen::MatrixXd bezier_base_matrix_;

  int num_joints_;
  int num_limbs_;

  fbml::Kinematics * kinematics_;
  fbml::Dynamics * dynamics_;

  Eigen::VectorXd q_init_;
  std::string swing_frame_name_;
  std::vector<std::string> swing_joint_names_;
};

}  // namespace lrst
}  // namespace ramp

#endif  // RAMP__MD__LOW_REACTION_SWING_TRAJECTORY_HPP_
