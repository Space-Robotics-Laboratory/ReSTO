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

#include "mlivr_control/wbc_solver.hpp"

#include <iostream>

#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
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
  const std::vector<double> & current_joint_pos, const std::string & support_ee_frame,
  const std::string & swing_ee_frame, const Eigen::Vector3d & world_translation_offset)
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

  pinocchio::FrameIndex sup_ee_id = model_ptr_->getFrameId(support_ee_frame);
  pinocchio::FrameIndex sw_ee_id = model_ptr_->getFrameId(swing_ee_frame);

  pinocchio::SE3 start_sup_ee_pose = data_ptr_->oMf[sup_ee_id];  // in world frame
  pinocchio::SE3 start_sw_ee_pose = data_ptr_->oMf[sw_ee_id];    // in world frame
  pinocchio::SE3 target_swing_pose = start_sw_ee_pose;           // in world frame
  target_swing_pose.translation() += world_translation_offset;

  double start_sup_ee_z = start_sup_ee_pose.translation().z();
  double start_sw_ee_z = start_sw_ee_pose.translation().z();
  double target_sw_ee_z = target_swing_pose.translation().z();

  double ctrl_reg_weight_default = params_.weights.control_reg;
  double sw_ee_tracking_default = params_.weights.ee_tracking;
  double ee_vel_weight_default = params_.weights.ee_vel_damping;

  double max_clearance = 0.05;

  int T = params_.solver.horizon_steps;

  // === Running Model ===

  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;

  for (int i = 0; i < T; ++i) {
    TaskPhase phase;

    double s = static_cast<double>(i) / std::max(T - 1, 1);

    // XY平面(およびZのオフセット)の滑らかな遷移割合 (5次多項式)「最小躍度軌道（Minimum Jerk Trajectory）」
    double s_mj = 10.0 * std::pow(s, 3) - 15.0 * std::pow(s, 4) + 6.0 * std::pow(s, 5);

    // Z軸の持ち上げアーチ (両端で速度・加速度ゼロ)
    double arch = 16.0 * std::pow(s, 2) * std::pow(1.0 - s, 2);

    pinocchio::SE3 step_target_pose = start_sw_ee_pose;

    // 目標位置の計算
    step_target_pose.translation().x() += world_translation_offset.x() * s_mj;
    step_target_pose.translation().y() += world_translation_offset.y() * s_mj;
    step_target_pose.translation().z() +=
      world_translation_offset.z() * s_mj + (max_clearance * arch);

    // 終端だけでなく、道中のすべてのステップでターゲットを与える
    phase.ee_tracking_targets[swing_ee_frame] = step_target_pose;
    phase.ee_tracking_targets[support_ee_frame] = start_sup_ee_pose;

    // phase.collision_frames = {support_ee_frame, swing_ee_frame};
    phase.collision_frames = {swing_ee_frame};
    phase.support_limbs = {support_ee_frame};

    // double s = static_cast<double>(i) / (T - 1);
    // double arch = 4.0 * s * (1.0 - s);

    phase.ee_z_lower_bounds[support_ee_frame] = start_sup_ee_z;
    phase.ee_z_lower_bounds[swing_ee_frame] = start_sw_ee_z + (max_clearance * arch);

    params_.weights.control_reg =
      ctrl_reg_weight_default * computeWeightMultiplier(s, params_.ctrl_reg_schedule);
    params_.weights.ee_tracking = sw_ee_tracking_default * 1e-5;
    params_.weights.ee_vel_damping =
      ee_vel_weight_default * computeWeightMultiplier(s, params_.ee_vel_schedule);

    running_models.push_back(createActionModel(x0, phase));
  }

  // === Terminal Model ===

  TaskPhase terminal_phase;
  terminal_phase.ee_tracking_targets[support_ee_frame] = start_sup_ee_pose;
  terminal_phase.ee_tracking_targets[swing_ee_frame] = target_swing_pose;
  // terminal_phase.collision_frames = {support_ee_frame, swing_ee_frame};
  terminal_phase.collision_frames = {swing_ee_frame};
  terminal_phase.support_limbs = {support_ee_frame};
  terminal_phase.ee_z_lower_bounds[support_ee_frame] = start_sup_ee_z;
  terminal_phase.ee_z_lower_bounds[swing_ee_frame] = target_sw_ee_z;
  params_.weights.control_reg =
    ctrl_reg_weight_default * params_.ctrl_reg_schedule.decel_multi * 2.0;
  params_.weights.ee_tracking = sw_ee_tracking_default;
  params_.weights.ee_vel_damping =
    ee_vel_weight_default * params_.ee_vel_schedule.decel_multi * 2.0;

  auto terminal_model = createActionModel(x0, terminal_phase);

  //=======================

  auto problem = std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);
  crocoddyl::SolverFDDP solver(problem);

  std::vector<Eigen::VectorXd> xs_init(T + 1, x0);
  std::vector<Eigen::VectorXd> us_init(T, Eigen::VectorXd::Zero(actuation_->get_nu()));

  std::vector<std::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
  callbacks.push_back(std::make_shared<crocoddyl::CallbackVerbose>());
  solver.setCallbacks(callbacks);

  solver.solve(xs_init, us_init, params_.solver.max_iter, false);

  optimized_xs_ = solver.get_xs();
  optimized_us_ = solver.get_us();

  std::cout << "[WbcSolver] Optimization completed! Trajectory length: " << optimized_xs_.size()
            << std::endl;

  return true;
}

std::shared_ptr<crocoddyl::ActionModelAbstract> WbcSolver::createActionModel(
  const Eigen::VectorXd & x0, const TaskPhase & phase)
{
  // auto contacts = std::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  // === Costs ===

  addStateAndControlRegularizationCosts(costs, x0);

  addStateAndControlLimitsCost(costs);

  addEndEffectorTrackingCost(costs, phase);

  addEndEffectorVelocityDampingCost(costs);

  addEnvironmentCollisionCost(costs, phase);

  addMomentumRegularizationCost(costs);

  // Differential-Algebraic Model (DAM)
  // auto dmodel = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
  //   state_, actuation_, contacts, costs, 0.0, true);
  auto dmodel =
    std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, costs);

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

double WbcSolver::computeWeightMultiplier(double s, const WeightScheduleParams & sched)
{
  if (s < sched.s_accel) {
    double ratio = (sched.s_accel - s) / std::max(sched.s_accel, 1e-6);
    return 1.0 + (sched.accel_multi - 1.0) * (ratio * ratio);
  } else if (s > sched.s_decel) {
    double ratio = (s - sched.s_decel) / std::max(1.0 - sched.s_decel, 1e-6);
    return 1.0 + (sched.decel_multi - 1.0) * (ratio * ratio);
  }
  return 1.0;
}

}  // namespace mlivr_control
