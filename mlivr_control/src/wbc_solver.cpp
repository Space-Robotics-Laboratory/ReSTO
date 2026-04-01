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

#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
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
  const std::vector<double> & current_q_14, const std::string & fixed_frame,
  const std::string & swing_frame, const Eigen::Vector3d & local_translation_offset)
{
  std::cout << "[WbcSolver] === Starting Trajectory Optimization ===" << std::endl;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_ptr_->nq);
  q(6) = 1.0;
  for (int i = 0; i < 14; ++i) q(7 + i) = current_q_14[i];

  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_ptr_->nv);
  Eigen::VectorXd x0(model_ptr_->nq + model_ptr_->nv);
  x0 << q, v;

  pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, q);
  pinocchio::updateFramePlacements(*model_ptr_, *data_ptr_);

  pinocchio::FrameIndex fixed_id = model_ptr_->getFrameId(fixed_frame);
  pinocchio::FrameIndex swing_id = model_ptr_->getFrameId(swing_frame);

  pinocchio::SE3 fixed_pose = data_ptr_->oMf[fixed_id];
  pinocchio::SE3 start_swing_pose = data_ptr_->oMf[swing_id];

  // ローカル座標系での移動量を適用する処理
  pinocchio::SE3 local_offset = pinocchio::SE3::Identity();
  local_offset.translation() = local_translation_offset;
  pinocchio::SE3 target_swing_pose = start_swing_pose * local_offset;

  int T = params_.solver.horizon_steps;
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;

  for (int i = 0; i < T; ++i) {
    double s = static_cast<double>(i) / T;
    pinocchio::SE3 current_target = start_swing_pose;
    current_target.translation() =
      (1.0 - s) * start_swing_pose.translation() + s * target_swing_pose.translation();

    TaskPhase current_phase;
    current_phase.active_contacts[fixed_frame] = fixed_pose;
    current_phase.swing_targets[swing_frame] = current_target;

    auto model = createActionModel(x0, current_phase);
    running_models.push_back(model);
  }

  TaskPhase terminal_phase;
  terminal_phase.active_contacts[fixed_frame] = fixed_pose;
  terminal_phase.swing_targets[swing_frame] = target_swing_pose;

  auto terminal_model = createActionModel(x0, terminal_phase);

  auto problem = std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);
  crocoddyl::SolverFDDP solver(problem);

  std::vector<std::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
  callbacks.push_back(std::make_shared<crocoddyl::CallbackVerbose>());
  solver.setCallbacks(callbacks);

  solver.solve(solver.get_xs(), solver.get_us(), 100, false);

  optimized_xs_ = solver.get_xs();
  std::cout << "[WbcSolver] Optimization completed! Trajectory length: " << optimized_xs_.size()
            << std::endl;
  return true;
}

std::shared_ptr<crocoddyl::ActionModelAbstract> WbcSolver::createActionModel(
  const Eigen::VectorXd & x0, const TaskPhase & phase)
{
  auto contacts = std::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  // Dynamic addition of contact model
  for (const auto & [frame_name, pose] : phase.active_contacts) {
    pinocchio::FrameIndex frame_id = model_ptr_->getFrameId(frame_name);
    auto contact_6d = std::make_shared<crocoddyl::ContactModel6D>(
      state_, frame_id, pose, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, actuation_->get_nu());

    contacts->addContact(frame_name + "_weld", contact_6d);
  }

  // === Costs ===

  // Dynamic addition of target-tracking costs
  for (const auto & [frame_name, target_pose] : phase.swing_targets) {
    pinocchio::FrameIndex frame_id = model_ptr_->getFrameId(frame_name);
    auto placement_residual = std::make_shared<crocoddyl::ResidualModelFramePlacement>(
      state_, frame_id, target_pose, actuation_->get_nu());

    costs->addCost(
      frame_name + "_swing_goal",
      std::make_shared<crocoddyl::CostModelResidual>(state_, placement_residual),
      params_.weights.swing_goal);
  }

  // State and Control regularization cost
  auto x_residual =
    std::make_shared<crocoddyl::ResidualModelState>(state_, x0, actuation_->get_nu());
  costs->addCost(
    "state_reg", std::make_shared<crocoddyl::CostModelResidual>(state_, x_residual),
    params_.weights.state_reg);

  auto u_residual = std::make_shared<crocoddyl::ResidualModelControl>(state_, actuation_->get_nu());
  costs->addCost(
    "control_reg", std::make_shared<crocoddyl::CostModelResidual>(state_, u_residual),
    params_.weights.control_reg);

  // Differential-Algebraic Model (DAM)
  auto dmodel = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
    state_, actuation_, contacts, costs, 0.0, true);

  // Integrated Atmospheric Model (IAM) (Computing the state one step ahead using Euler integration)
  return std::make_shared<crocoddyl::IntegratedActionModelEuler>(dmodel, params_.solver.dt);
}

}  // namespace mlivr_control
