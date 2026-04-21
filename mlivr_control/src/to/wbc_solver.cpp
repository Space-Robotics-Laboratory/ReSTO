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

#include "mlivr_control/to/wbc_solver.hpp"

#include <iostream>

#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/actions/impulse-fwddyn.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/impulses/impulse-6d.hpp>
#include <crocoddyl/multibody/impulses/multiple-impulses.hpp>
#include <crocoddyl/multibody/residuals/centroidal-momentum.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace mlivr_control
{

WbcSolver::WbcSolver(std::shared_ptr<pinocchio::Model> model, const WbcSolverParams & params)
: model_ptr_(model), params_(params)
{
  data_ptr_ = std::make_shared<pinocchio::Data>(*model_ptr_);

  state_ = std::make_shared<crocoddyl::StateMultibody>(model_ptr_);
  actuation_ = std::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);

  std::cout << "[WbcSolver] Crocoddyl State and Actuation models initialized." << std::endl;
}

bool WbcSolver::computeTrajectory(
  const Eigen::VectorXd & base_pose, const Eigen::VectorXd & base_twist,
  const std::vector<double> & current_joint_pos, const std::string & fixed_frame,
  const std::string & swing_frame, const Eigen::Vector3d & world_translation_offset)
{
  std::cout << "[WbcSolver] === Starting Trajectory Optimization ===" << std::endl;

  int num_joints = model_ptr_->nv - 6;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_ptr_->nq);
  q.head(7) = base_pose;
  for (int i = 0; i < num_joints; ++i) {
    q(7 + i) = current_joint_pos[i];
  }

  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_ptr_->nv);
  v.head(6) = base_twist;

  Eigen::VectorXd x0(model_ptr_->nq + model_ptr_->nv);
  x0 << q, v;

  pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, q);
  pinocchio::updateFramePlacements(*model_ptr_, *data_ptr_);

  pinocchio::FrameIndex fixed_id = model_ptr_->getFrameId(fixed_frame);
  pinocchio::FrameIndex swing_id = model_ptr_->getFrameId(swing_frame);

  pinocchio::SE3 start_fixed_pose = data_ptr_->oMf[fixed_id];  // in world frame
  pinocchio::SE3 start_swing_pose = data_ptr_->oMf[swing_id];  // in world frame

  double start_fixed_ee_z = start_fixed_pose.translation().z();
  double start_swing_ee_z = start_swing_pose.translation().z();

  pinocchio::SE3 target_swing_pose = start_swing_pose;  // in world frame
  target_swing_pose.translation() += world_translation_offset;

  double target_swing_ee_z = target_swing_pose.translation().z();

  int T = params_.solver.horizon_steps;

  double ctrl_reg_weight_default = params_.weights.control_reg;
  double s_ctrl_start = 0.1;
  double s_ctrl_brake = 0.9;
  double ctrl_start_mult = 5.0;
  double ctrl_brake_mult = 20.0;

  double ee_vel_weight_default = params_.weights.ee_vel_damping;
  // TODO: Parameterize
  double s_start = 0.2;
  double s_brake = 0.6;
  double start_mult = 5.0;
  double brake_mult = 20.0;

  double max_clearance = 0.05;

  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;

  for (int i = 0; i < T; ++i) {
    TaskPhase phase;

    phase.ee_tracking_targets[fixed_frame] = start_fixed_pose;

    if (i == T - 1) {
      phase.ee_tracking_targets[swing_frame] = target_swing_pose;
    }

    // phase.collision_frames = {fixed_frame, swing_frame};
    phase.collision_frames = {swing_frame};

    phase.support_limbs = {fixed_frame};

    double mult_ctrl = 1.0;
    double s_ctrl = static_cast<double>(i) / std::max(T - 1, 1);
    if (s_ctrl < s_ctrl_start) {
      double ratio = (s_ctrl_start - s_ctrl) / std::max(s_ctrl_start, 1e-6);
      mult_ctrl = 1.0 + (ctrl_start_mult - 1.0) * (ratio * ratio);
    } else if (s_ctrl > s_ctrl_brake) {
      double ratio = (s_ctrl - s_ctrl_brake) / std::max(1.0 - s_ctrl_brake, 1e-6);
      mult_ctrl = 1.0 + (ctrl_brake_mult - 1.0) * (ratio * ratio);
    } else {
      mult_ctrl = 1.0;
    }
    params_.weights.control_reg = ctrl_reg_weight_default * mult_ctrl;

    double multiplier = 1.0;
    double s_vel = static_cast<double>(i) / std::max(T - 1, 1);
    if (s_vel < s_start) {
      double ratio = (s_start - s_vel) / std::max(s_start, 1e-6);
      multiplier = 1.0 + (start_mult - 1.0) * (ratio * ratio);
    } else if (s_vel > s_brake) {
      double ratio = (s_vel - s_brake) / std::max(1.0 - s_brake, 1e-6);
      multiplier = 1.0 + (brake_mult - 1.0) * (ratio * ratio);
    } else {
      multiplier = 1.0;
    }
    params_.weights.ee_vel_damping = ee_vel_weight_default * multiplier;

    double s = static_cast<double>(i) / (T - 1);
    double arch = 4.0 * s * (1.0 - s);

    phase.ee_z_lower_bounds[fixed_frame] = start_fixed_ee_z;
    phase.ee_z_lower_bounds[swing_frame] = start_swing_ee_z + (max_clearance * arch);

    auto model = createActionModel(x0, phase);
    running_models.push_back(model);
  }

  TaskPhase terminal_phase;
  terminal_phase.ee_tracking_targets[fixed_frame] = start_fixed_pose;
  terminal_phase.ee_tracking_targets[swing_frame] = target_swing_pose;
  // terminal_phase.collision_frames = {fixed_frame, swing_frame};
  terminal_phase.collision_frames = {swing_frame};
  terminal_phase.support_limbs = {fixed_frame};
  terminal_phase.ee_z_lower_bounds[fixed_frame] = start_fixed_ee_z;
  terminal_phase.ee_z_lower_bounds[swing_frame] = target_swing_ee_z;
  params_.weights.control_reg = ctrl_reg_weight_default * ctrl_brake_mult * 2.0;
  params_.weights.ee_vel_damping = ee_vel_weight_default * brake_mult * 2.0;

  auto terminal_model = createActionModel(x0, terminal_phase);

  auto problem = std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);
  crocoddyl::SolverFDDP solver(problem);

  std::vector<std::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
  callbacks.push_back(std::make_shared<crocoddyl::CallbackVerbose>());
  solver.setCallbacks(callbacks);

  solver.solve(solver.get_xs(), solver.get_us(), params_.solver.max_iter, false);

  optimized_xs_ = solver.get_xs();
  optimized_us_ = solver.get_us();

  std::cout << "[WbcSolver] Optimization completed! Trajectory length: " << optimized_xs_.size()
            << std::endl;

  return true;
}

std::shared_ptr<crocoddyl::ActionModelAbstract> WbcSolver::createActionModel(
  const Eigen::VectorXd & x0, const TaskPhase & phase)
{
  auto contacts = std::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  // === Costs ===

  addStateAndControlRegularizationCosts(costs, x0);

  addStateAndControlLimitsCost(costs);

  addEndEffectorTrackingCost(costs, phase);

  addEndEffectorVelocityDampingCost(costs);

  addEnvironmentCollisionCost(costs, phase);

  addMomentumRegularizationCost(costs);

  // Differential-Algebraic Model (DAM)
  auto dmodel = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
    state_, actuation_, contacts, costs, 0.0, true);

  // Integrated Atmospheric Model (IAM) (Computing the state one step ahead using Euler integration)
  return std::make_shared<crocoddyl::IntegratedActionModelEuler>(dmodel, params_.solver.dt);
}

void WbcSolver::addStateAndControlRegularizationCosts(
  std::shared_ptr<crocoddyl::CostModelSum> & costs, const Eigen::VectorXd & x0)
{
  // State regularization cost
  auto x_residual =
    std::make_shared<crocoddyl::ResidualModelState>(state_, x0, actuation_->get_nu());
  costs->addCost(
    "state_reg", std::make_shared<crocoddyl::CostModelResidual>(state_, x_residual),
    params_.weights.state_reg);

  // Control regularization cost
  auto u_residual = std::make_shared<crocoddyl::ResidualModelControl>(state_, actuation_->get_nu());
  costs->addCost(
    "control_reg", std::make_shared<crocoddyl::CostModelResidual>(state_, u_residual),
    params_.weights.control_reg);
}

void WbcSolver::addStateAndControlLimitsCost(std::shared_ptr<crocoddyl::CostModelSum> & costs)
{
  // State and Control Limit Constraints (barrier functions)

  // === State Limit (Tangent Space) ===

  int num_joints = model_ptr_->nv - 6;

  Eigen::VectorXd x_lb =
    Eigen::VectorXd::Constant(state_->get_ndx(), -std::numeric_limits<double>::infinity());
  Eigen::VectorXd x_ub =
    Eigen::VectorXd::Constant(state_->get_ndx(), std::numeric_limits<double>::infinity());

  x_lb.segment(6, num_joints) = model_ptr_->lowerPositionLimit.tail(num_joints);
  x_ub.segment(6, num_joints) = model_ptr_->upperPositionLimit.tail(num_joints);

  int vel_start_idx = 12 + num_joints;  // 12: base pose err and vel err
  x_lb.segment(vel_start_idx, num_joints) = -model_ptr_->velocityLimit.tail(num_joints);
  x_ub.segment(vel_start_idx, num_joints) = model_ptr_->velocityLimit.tail(num_joints);

  crocoddyl::ActivationBounds x_bounds(x_lb, x_ub);
  auto x_limit_activation = std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(x_bounds);

  auto zero_state = state_->zero();
  auto x_limit_residual =
    std::make_shared<crocoddyl::ResidualModelState>(state_, zero_state, actuation_->get_nu());

  costs->addCost(
    "state_limits",
    std::make_shared<crocoddyl::CostModelResidual>(state_, x_limit_activation, x_limit_residual),
    params_.weights.state_limits);

  // === Control Limit ===

  Eigen::VectorXd u_ub = model_ptr_->effortLimit.tail(actuation_->get_nu());
  Eigen::VectorXd u_lb = -u_ub;
  crocoddyl::ActivationBounds u_bounds(u_lb, u_ub);
  auto u_limit_activation = std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(u_bounds);

  auto u_limit_residual =
    std::make_shared<crocoddyl::ResidualModelControl>(state_, actuation_->get_nu());

  costs->addCost(
    "control_limits",
    std::make_shared<crocoddyl::CostModelResidual>(state_, u_limit_activation, u_limit_residual),
    params_.weights.control_limits);
}

void WbcSolver::addEndEffectorTrackingCost(
  std::shared_ptr<crocoddyl::CostModelSum> & costs, const TaskPhase & phase)
{
  // Dynamic addition of target-tracking costs
  for (const auto & [frame_name, target_pose] : phase.ee_tracking_targets) {
    pinocchio::FrameIndex frame_id = model_ptr_->getFrameId(frame_name);

    auto placement_residual = std::make_shared<crocoddyl::ResidualModelFramePlacement>(
      state_, frame_id, target_pose, actuation_->get_nu());

    double ee_tracking_weight = params_.weights.ee_tracking;

    auto it = std::find(phase.support_limbs.begin(), phase.support_limbs.end(), frame_name);
    if (it != phase.support_limbs.end()) {
      // ee_tracking_weight *= 2.0;  // TODO: Parameterize
      ee_tracking_weight = params_.weights.sup_ee_tracking;
    }

    costs->addCost(
      frame_name + "_tracking_target",
      std::make_shared<crocoddyl::CostModelResidual>(state_, placement_residual),
      ee_tracking_weight);
  }
}

void WbcSolver::addEndEffectorVelocityDampingCost(std::shared_ptr<crocoddyl::CostModelSum> & costs)
{
  // End-effector velocity damping cost
  for (const auto & frame_name : params_.ee_frames) {
    pinocchio::FrameIndex frame_id = model_ptr_->getFrameId(frame_name);

    auto vel_residual = std::make_shared<crocoddyl::ResidualModelFrameVelocity>(
      state_, frame_id, pinocchio::Motion::Zero(), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
      actuation_->get_nu());

    costs->addCost(
      frame_name + "_vel_damping",
      std::make_shared<crocoddyl::CostModelResidual>(state_, vel_residual),
      params_.weights.ee_vel_damping);
  }
}

void WbcSolver::addEnvironmentCollisionCost(
  std::shared_ptr<crocoddyl::CostModelSum> & costs, const TaskPhase & phase)
{
  for (const auto & frame_name : phase.collision_frames) {
    pinocchio::FrameIndex frame_id = model_ptr_->getFrameId(frame_name);

    double min_z = -1000.0;
    auto it = phase.ee_z_lower_bounds.find(frame_name);
    if (it != phase.ee_z_lower_bounds.end()) {
      min_z = it->second;
    }

    // Barrier model for frames
    Eigen::Vector3d lb_trans(-1000.0, -1000.0, min_z);
    Eigen::Vector3d ub_trans(1000.0, 1000.0, 1000.0);
    crocoddyl::ActivationBounds trans_bounds(lb_trans, ub_trans);
    auto trans_barrier_activation =
      std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(trans_bounds);

    auto translation_residual = std::make_shared<crocoddyl::ResidualModelFrameTranslation>(
      state_, frame_id, Eigen::Vector3d::Zero(), actuation_->get_nu());

    costs->addCost(
      frame_name + "_floor_collision",
      std::make_shared<crocoddyl::CostModelResidual>(
        state_, trans_barrier_activation, translation_residual),
      params_.weights.env_collision);
  }
}

void WbcSolver::addMomentumRegularizationCost(std::shared_ptr<crocoddyl::CostModelSum> & costs)
{
  // Penalty to keep total momentum about the robot's center of mass at zero
  auto momentum_residual = std::make_shared<crocoddyl::ResidualModelCentroidalMomentum>(
    state_, Eigen::VectorXd::Zero(6), actuation_->get_nu());

  costs->addCost(
    "momentum_reg", std::make_shared<crocoddyl::CostModelResidual>(state_, momentum_residual),
    params_.weights.momentum_reg);
}

}  // namespace mlivr_control
