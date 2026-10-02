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

#include "resto/swing_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/quadratic.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/residuals/centroidal-momentum.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/multibody.hpp>

namespace resto
{
namespace
{

// Heavier near the start and end of the swing, easing it in and out.
double scheduleMultiplier(double s, const WeightSchedule & schedule)
{
  if (s < schedule.s_accel) {
    const double ratio = (schedule.s_accel - s) / std::max(schedule.s_accel, 1e-6);
    return 1.0 + (schedule.accel_multi - 1.0) * ratio * ratio;
  }
  if (s > schedule.s_decel) {
    const double ratio = (s - schedule.s_decel) / std::max(1.0 - schedule.s_decel, 1e-6);
    return 1.0 + (schedule.decel_multi - 1.0) * ratio * ratio;
  }
  return 1.0;
}

}  // namespace

SwingOptimizer::SwingOptimizer(
  std::shared_ptr<pinocchio::Model> model, const SolverParams & solver_params,
  const WeightParams & weight_params)
: model_(std::move(model)), solver_params_(solver_params), weight_params_(weight_params)
{
  if (
    !model_ || model_->njoints < 2 || model_->joints[1].nq() != 7 || model_->joints[1].nv() != 6) {
    throw std::invalid_argument("resto::SwingOptimizer needs a free-flyer root joint");
  }
  if (
    !std::isfinite(solver_params_.dt) || solver_params_.dt <= 0.0 || solver_params_.max_iter <= 0) {
    throw std::invalid_argument("resto::SwingOptimizer needs a positive dt and max_iter");
  }
}

Result SwingOptimizer::solve(const SwingProblem & problem) const
{
  const pinocchio::Model & model = *model_;
  if (
    problem.x0.size() != model.nq + model.nv || !problem.x0.allFinite() ||
    problem.support_frames.size() != problem.support_targets.size() ||
    !model.existFrame(problem.swing_frame) || !std::isfinite(problem.duration) ||
    problem.duration <= 0.0 ||
    !std::all_of(
      problem.support_frames.begin(), problem.support_frames.end(),
      [&model](const std::string & frame) { return model.existFrame(frame); })) {
    throw std::invalid_argument(
      "resto::SwingProblem needs x0 = [q; v], one target per support frame, known frames and a "
      "positive duration");
  }
  const WeightParams & weights = weight_params_;

  auto state = std::make_shared<crocoddyl::StateMultibody>(model_);
  auto actuation = std::make_shared<crocoddyl::ActuationModelFloatingBase>(state);
  const std::size_t nu = actuation->get_nu();
  const int nv = model.nv;

  // Barrier bounds in the state tangent space: [base 6, joints, base twist 6, joint velocities].
  Eigen::VectorXd x_lb =
    Eigen::VectorXd::Constant(state->get_ndx(), -std::numeric_limits<double>::infinity());
  Eigen::VectorXd x_ub = -x_lb;
  for (pinocchio::JointIndex joint = 2; joint < model.joints.size(); ++joint) {
    const auto & joint_model = model.joints[joint];
    if (joint_model.nq() != 1 || joint_model.nv() != 1) {
      continue;
    }
    x_lb[joint_model.idx_v()] = model.lowerPositionLimit[joint_model.idx_q()];
    x_ub[joint_model.idx_v()] = model.upperPositionLimit[joint_model.idx_q()];
    x_lb[nv + joint_model.idx_v()] = -model.velocityLimit[joint_model.idx_v()];
    x_ub[nv + joint_model.idx_v()] = model.velocityLimit[joint_model.idx_v()];
  }
  const Eigen::VectorXd u_ub = model.effortLimit.tail(static_cast<Eigen::Index>(nu));
  Eigen::VectorXd state_weights = Eigen::VectorXd::Ones(state->get_ndx());
  state_weights.head<6>() =
    Eigen::Map<const Eigen::Matrix<double, 6, 1>>(weights.base_pose_reg.data());
  const pinocchio::FrameIndex swing_id = model.getFrameId(problem.swing_frame);
  std::vector<pinocchio::FrameIndex> support_ids;
  for (const auto & frame : problem.support_frames) {
    support_ids.push_back(model.getFrameId(frame));
  }

  const auto make_node = [&](
                           double control_weight, double velocity_weight, bool terminal,
                           double dt) {
    auto costs = std::make_shared<crocoddyl::CostModelSum>(state, nu);
    const auto add = [&](const std::string & name, auto activation, auto residual, double weight) {
      costs->addCost(
        name, std::make_shared<crocoddyl::CostModelResidual>(state, activation, residual), weight);
    };
    add(
      "state_reg", std::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights),
      std::make_shared<crocoddyl::ResidualModelState>(state, problem.x0, nu), weights.state_reg);
    add(
      "control_reg", std::make_shared<crocoddyl::ActivationModelQuad>(nu),
      std::make_shared<crocoddyl::ResidualModelControl>(state, nu), control_weight);
    add(
      "state_limits",
      std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(
        crocoddyl::ActivationBounds(x_lb, x_ub)),
      std::make_shared<crocoddyl::ResidualModelState>(state, state->zero(), nu),
      weights.state_limits);
    add(
      "control_limits",
      std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(
        crocoddyl::ActivationBounds(-u_ub, u_ub)),
      std::make_shared<crocoddyl::ResidualModelControl>(state, nu), weights.control_limits);
    for (std::size_t support = 0; support < support_ids.size(); ++support) {
      add(
        problem.support_frames[support] + "_tracking",
        std::make_shared<crocoddyl::ActivationModelQuad>(6),
        std::make_shared<crocoddyl::ResidualModelFramePlacement>(
          state, support_ids[support], problem.support_targets[support], nu),
        weights.sup_ee_tracking);
    }
    if (terminal) {
      add(
        problem.swing_frame + "_tracking", std::make_shared<crocoddyl::ActivationModelQuad>(6),
        std::make_shared<crocoddyl::ResidualModelFramePlacement>(
          state, swing_id, problem.swing_target, nu),
        weights.sw_ee_tracking);
    }
    std::vector<pinocchio::FrameIndex> damped_ids = support_ids;
    damped_ids.push_back(swing_id);
    for (const auto id : damped_ids) {
      add(
        model.frames[id].name + "_vel_damping", std::make_shared<crocoddyl::ActivationModelQuad>(6),
        std::make_shared<crocoddyl::ResidualModelFrameVelocity>(
          state, id, pinocchio::Motion::Zero(), pinocchio::LOCAL_WORLD_ALIGNED, nu),
        velocity_weight);
    }
    add(
      "momentum_reg", std::make_shared<crocoddyl::ActivationModelQuad>(6),
      std::make_shared<crocoddyl::ResidualModelCentroidalMomentum>(
        state, Eigen::Matrix<double, 6, 1>::Zero(), nu),
      weights.momentum_reg);
    return std::make_shared<crocoddyl::IntegratedActionModelEuler>(
      std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, costs),
      dt);
  };

  const int horizon =
    std::max(1, static_cast<int>(std::lround(problem.duration / solver_params_.dt)));
  const double dt = problem.duration / horizon;
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running;
  running.reserve(static_cast<std::size_t>(horizon));
  for (int node = 0; node < horizon; ++node) {
    const double s = static_cast<double>(node) / std::max(horizon - 1, 1);
    running.push_back(make_node(
      weights.control_reg * scheduleMultiplier(s, weights.control_reg_schedule),
      weights.ee_vel_damping * scheduleMultiplier(s, weights.ee_vel_schedule), false, dt));
  }
  const auto terminal = make_node(
    weights.control_reg * weights.control_reg_schedule.decel_multi * 2.0,
    weights.ee_vel_damping * weights.ee_vel_schedule.decel_multi * 2.0, true, dt);

  crocoddyl::SolverFDDP solver(
    std::make_shared<crocoddyl::ShootingProblem>(problem.x0, running, terminal));
  const std::vector<Eigen::VectorXd> xs_init(static_cast<std::size_t>(horizon) + 1, problem.x0);
  const std::vector<Eigen::VectorXd> us_init(
    static_cast<std::size_t>(horizon), Eigen::VectorXd::Zero(static_cast<Eigen::Index>(nu)));
  Result result;
  result.converged =
    solver.solve(xs_init, us_init, static_cast<std::size_t>(solver_params_.max_iter), false);
  result.iterations = solver.get_iter();
  result.cost = solver.get_cost();
  result.xs = solver.get_xs();
  result.us = solver.get_us();
  for (const auto & x : result.xs) {
    if (!x.allFinite()) {
      throw std::runtime_error("resto::SwingOptimizer produced a non-finite trajectory");
    }
  }
  return result;
}

}  // namespace resto
