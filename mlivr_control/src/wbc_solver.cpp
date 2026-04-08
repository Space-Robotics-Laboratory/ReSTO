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
  const std::vector<double> & current_q, const std::string & fixed_frame,
  const std::string & swing_frame, const Eigen::Vector3d & local_translation_offset)
{
  std::cout << "[WbcSolver] === Starting Trajectory Optimization ===" << std::endl;

  int num_joints = model_ptr_->nv - 6;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_ptr_->nq);
  q(6) = 1.0;
  for (int i = 0; i < num_joints; ++i) {
    q(7 + i) = current_q[i];
  }

  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_ptr_->nv);
  Eigen::VectorXd x0(model_ptr_->nq + model_ptr_->nv);
  x0 << q, v;

  pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, q);
  pinocchio::updateFramePlacements(*model_ptr_, *data_ptr_);

  pinocchio::FrameIndex fixed_id = model_ptr_->getFrameId(fixed_frame);
  pinocchio::FrameIndex swing_id = model_ptr_->getFrameId(swing_frame);

  pinocchio::SE3 start_fixed_pose = data_ptr_->oMf[fixed_id];  // in world frame
  pinocchio::SE3 start_swing_pose = data_ptr_->oMf[swing_id];  // in world frame

  pinocchio::SE3 target_swing_pose = start_swing_pose;  // in world frame

  target_swing_pose.translation() += local_translation_offset;

  // ローカル座標系での移動量を適用する処理
  // pinocchio::SE3 local_offset = pinocchio::SE3::Identity();
  // local_offset.translation() = local_translation_offset;
  // target_swing_pose = start_swing_pose * local_offset;

  // ==========================================================
  // ★ 追加: 局所解を突破するための「中間経由点 (Via-point)」
  // ==========================================================
  pinocchio::SE3 via_swing_pose = start_swing_pose;
  via_swing_pose.translation() =
    (start_swing_pose.translation() + target_swing_pose.translation()) / 2.0;

  int T = params_.solver.horizon_steps;

  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;

  // ==========================================================
  // 【究極の無反動軌道】
  // 全ホライズンを単一のフェーズで計算(ハード制約・衝撃モデルなし)
  // ==========================================================
  for (int i = 0; i < T; ++i) {
    TaskPhase phase;

    // limb_1 は常に初期位置をキープする (ソフト制約)
    phase.swing_targets[fixed_frame] = start_fixed_pose;

    if (i == T / 2) {
      // 軌道のちょうど半分の時間で、持ち上げた経由点を通るように誘導
      phase.swing_targets[swing_frame] = via_swing_pose;
    }
    // limb_2 は「道中は自由に動かして良い」とするため、最後のステップのみ目標を与える
    if (i == T - 1) {
      phase.swing_targets[swing_frame] = target_swing_pose;
    }

    // phase.collision_frames = {fixed_frame, swing_frame};
    phase.collision_frames = {swing_frame};

    auto model = createActionModel(x0, phase);
    running_models.push_back(model);
  }

  // ==========================================================
  // [Terminal] 終端モデル
  // ==========================================================
  TaskPhase terminal_phase;
  terminal_phase.swing_targets[fixed_frame] = start_fixed_pose;
  terminal_phase.swing_targets[swing_frame] = target_swing_pose;
  // terminal_phase.collision_frames = {fixed_frame, swing_frame};
  terminal_phase.collision_frames = {swing_frame};

  auto terminal_model = createActionModel(x0, terminal_phase);

  auto problem = std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);
  crocoddyl::SolverFDDP solver(problem);

  std::vector<std::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
  callbacks.push_back(std::make_shared<crocoddyl::CallbackVerbose>());
  solver.setCallbacks(callbacks);

  solver.solve(solver.get_xs(), solver.get_us(), 500, false);

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

    // ==============================================================
    // ★ 追加：手先の空間速度をゼロに近づける（ダンピング）コスト
    // ==============================================================
    // 目標速度をゼロ (Motion::Zero) とし、動くこと自体にペナルティを与える
    auto vel_residual = std::make_shared<crocoddyl::ResidualModelFrameVelocity>(
      state_, frame_id, pinocchio::Motion::Zero(), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
      actuation_->get_nu());

    costs->addCost(
      frame_name + "_vel_damping",
      std::make_shared<crocoddyl::CostModelResidual>(state_, vel_residual),
      params_.weights.ee_vel_damping);
    // ==============================================================
  }

  // ==============================================================
  // ★ 環境との干渉回避コスト（床面バリア）
  // ==============================================================
  double min_z = -0.02;

  Eigen::Vector3d lb_trans(-1000.0, -1000.0, min_z);
  Eigen::Vector3d ub_trans(1000.0, 1000.0, 1000.0);

  crocoddyl::ActivationBounds trans_bounds(lb_trans, ub_trans);
  auto trans_barrier_activation =
    std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(trans_bounds);

  // ★ phase から受け取ったフレームのリストを回す
  for (const auto & frame_name : phase.collision_frames) {
    pinocchio::FrameIndex frame_id = model_ptr_->getFrameId(frame_name);

    auto translation_residual = std::make_shared<crocoddyl::ResidualModelFrameTranslation>(
      state_, frame_id, Eigen::Vector3d::Zero(), actuation_->get_nu());

    costs->addCost(
      frame_name + "_floor_collision",
      std::make_shared<crocoddyl::CostModelResidual>(
        state_, trans_barrier_activation, translation_residual),
      1e3);  // 遊び(min_z=-0.02)があるので、1e4でも暴れないはずです
  }
  // ==============================================================

  // Dynamic addition of limit constraints (barrier functions)

  // ==============================================================
  // ★ 復活＆改良: 状態リミット (Tangent Space 40次元へのスライス適用)
  // ==============================================================
  // state_->get_ndx() は 40次元 (ベース姿勢誤差6 + 関節角度14 + ベース速度6 + 関節速度14)
  int num_joints = model_ptr_->nv - 6;

  Eigen::VectorXd lb =
    Eigen::VectorXd::Constant(state_->get_ndx(), -std::numeric_limits<double>::infinity());
  Eigen::VectorXd ub =
    Eigen::VectorXd::Constant(state_->get_ndx(), std::numeric_limits<double>::infinity());

  // 1. 関節角度の上下限 (インデックス6〜19に格納される)
  lb.segment(6, num_joints) = model_ptr_->lowerPositionLimit.tail(num_joints);
  ub.segment(6, num_joints) = model_ptr_->upperPositionLimit.tail(num_joints);

  // 2. 関節速度の上下限 (インデックス26〜39に格納される)
  int vel_start_idx = 12 + num_joints;
  lb.segment(vel_start_idx, num_joints) = -model_ptr_->velocityLimit.tail(num_joints);
  ub.segment(vel_start_idx, num_joints) = model_ptr_->velocityLimit.tail(num_joints);

  crocoddyl::ActivationBounds x_bounds(lb, ub);
  auto x_limit_activation = std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(x_bounds);

  // xrefをゼロ状態(関節角0)にすることで、誤差ベクトルの関節部分が「絶対角度」と一致する
  auto x_limit_residual =
    std::make_shared<crocoddyl::ResidualModelState>(state_, state_->zero(), actuation_->get_nu());

  costs->addCost(
    "state_limits",
    std::make_shared<crocoddyl::CostModelResidual>(state_, x_limit_activation, x_limit_residual),
    params_.weights.state_limits);  // 非常に強い重みで絶対に限界を超えさせない
  // ==============================================================

  // Control limit
  Eigen::VectorXd u_max = model_ptr_->effortLimit.tail(actuation_->get_nu());
  Eigen::VectorXd u_min = -u_max;
  crocoddyl::ActivationBounds u_bounds(u_min, u_max);
  auto u_limit_activation = std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(u_bounds);

  auto u_limit_residual =
    std::make_shared<crocoddyl::ResidualModelControl>(state_, actuation_->get_nu());
  costs->addCost(
    "control_limits",
    std::make_shared<crocoddyl::CostModelResidual>(state_, u_limit_activation, u_limit_residual),
    params_.weights.control_limits);

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

  // ベースと全関節の動きによって生じる空間全体の運動量(h = [linear, angular])を計算し、
  // それがゼロ(Force::Zero)から変動しないようにペナルティを与える
  auto momentum_residual = std::make_shared<crocoddyl::ResidualModelCentroidalMomentum>(
    state_, Eigen::VectorXd::Zero(6), actuation_->get_nu());
  costs->addCost(
    "momentum_reg", std::make_shared<crocoddyl::CostModelResidual>(state_, momentum_residual),
    params_.weights.momentum_reg);

  // Differential-Algebraic Model (DAM)
  auto dmodel = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
    state_, actuation_, contacts, costs, 0.0, true);

  // Integrated Atmospheric Model (IAM) (Computing the state one step ahead using Euler integration)
  return std::make_shared<crocoddyl::IntegratedActionModelEuler>(dmodel, params_.solver.dt);
}

std::shared_ptr<crocoddyl::ActionModelAbstract> WbcSolver::createImpulseModel(
  const Eigen::VectorXd & x0, const std::string & impact_frame)
{
  auto impulses = std::make_shared<crocoddyl::ImpulseModelMultiple>(state_);
  pinocchio::FrameIndex impact_id = model_ptr_->getFrameId(impact_frame);

  // 衝突の瞬間、対象フレームを完全に固定（LOCAL_WORLD_ALIGNED）するインパルス
  auto impulse_6d = std::make_shared<crocoddyl::ImpulseModel6D>(
    state_, impact_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  impulses->addImpulse(impact_frame + "_impulse", impulse_6d);

  // インパルスモデルでは瞬間的な状態遷移のみを扱うため、制御入力(u)はゼロ(nu=0)
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_, 0);

  // =================================================================
  // ★ 衝撃吸収（Impact Minimization）コストの追加
  // =================================================================

  // 1. Soft Landing コスト: 衝突直前の手先速度(Linear/Angular)をゼロに近づける
  auto vel_residual = std::make_shared<crocoddyl::ResidualModelFrameVelocity>(
    state_, impact_id, pinocchio::Motion::Zero(), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
    0);
  costs->addCost(
    "impact_vel_min", std::make_shared<crocoddyl::CostModelResidual>(state_, vel_residual),
    1e3);  // 衝撃をどれだけ嫌がるかの重み

  // 2. Base Stabilization コスト: 衝突時の撃力がベースの運動量変動に伝わるのを防ぐ（姿勢で吸収させる）
  auto momentum_residual = std::make_shared<crocoddyl::ResidualModelCentroidalMomentum>(
    state_, Eigen::VectorXd::Zero(6), 0);  // インパルスモデルなので nu = 0
  costs->addCost(
    "impact_momentum_min",
    std::make_shared<crocoddyl::CostModelResidual>(state_, momentum_residual), 1e-2);

  // =================================================================

  // 状態正則化（衝突の瞬間に姿勢が極端に崩れないようにする）
  auto x_residual = std::make_shared<crocoddyl::ResidualModelState>(state_, x0, 0);
  costs->addCost(
    "state_reg", std::make_shared<crocoddyl::CostModelResidual>(state_, x_residual),
    params_.weights.state_reg);

  // 反発係数 r_coeff = 0.0 (完全非弾性衝突 = 把持・Weld)
  double r_coeff = 0.0;

  return std::make_shared<crocoddyl::ActionModelImpulseFwdDynamics>(
    state_, impulses, costs, r_coeff, 0.0, true);
}

}  // namespace mlivr_control
