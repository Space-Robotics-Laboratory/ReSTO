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

#ifndef RESTO__SWING_OPTIMIZER_HPP_
#define RESTO__SWING_OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace resto
{

struct SolverParams
{
  double dt = 0.01;  // OCP node period [s]; the horizon spans the swing duration
  int max_iter = 500;
};

struct WeightSchedule
{
  double s_accel = 0.1;  // normalized time before which the weight eases down from accel_multi
  double s_decel = 0.7;  // normalized time after which the weight eases up to decel_multi
  double accel_multi = 1.0;
  double decel_multi = 1.0;
};

struct WeightParams
{
  double state_reg = 1e-5;
  double control_reg = 1e2;
  std::array<double, 6> base_pose_reg{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};  // [x, y, z, r, p, y]
  double state_limits = 2e1;
  double control_limits = 1.0;
  double sw_ee_tracking = 1e4;  // terminal only
  double sup_ee_tracking = 1e5;
  double ee_vel_damping = 1e2;
  double momentum_reg = 1e1;
  WeightSchedule control_reg_schedule{0.1, 0.7, 50.0, 200.0};
  WeightSchedule ee_vel_schedule{0.1, 0.7, 50.0, 50.0};
};

struct SwingProblem
{
  Eigen::VectorXd x0;  // [q; v] of the free-floating model
  std::vector<std::string> support_frames;
  std::vector<pinocchio::SE3> support_targets;  // world placements held by the support frames
  std::string swing_frame;
  pinocchio::SE3 swing_target = pinocchio::SE3::Identity();  // world placement at the end
  double duration = 0.0;                                     // [s]
};

struct Result
{
  std::vector<Eigen::VectorXd> xs;  // horizon + 1 states, evenly spaced over the duration
  std::vector<Eigen::VectorXd> us;  // horizon joint torques
  bool converged = false;
  std::size_t iterations = 0;
  double cost = 0.0;
};

/// ReSTO: reaction-suppressing swing of a free-floating multi-limbed robot, solved as a
/// whole-body optimal control problem (Crocoddyl FDDP). Joint position, velocity and effort
/// limits come from the model, so narrow them there if needed.
class SwingOptimizer
{
public:
  SwingOptimizer(
    std::shared_ptr<pinocchio::Model> model, const SolverParams & solver_params,
    const WeightParams & weight_params);

  /// Throws std::invalid_argument on an inconsistent problem.
  [[nodiscard]] Result solve(const SwingProblem & problem) const;

private:
  std::shared_ptr<pinocchio::Model> model_;
  SolverParams solver_params_;
  WeightParams weight_params_;
};

}  // namespace resto

#endif  // RESTO__SWING_OPTIMIZER_HPP_
