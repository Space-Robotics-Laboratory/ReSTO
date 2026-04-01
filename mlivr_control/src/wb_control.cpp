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

#include "mlivr_control/wb_control.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mlivr_control
{

WBControl::WBControl(const rclcpp::NodeOptions & options) : Node("wb_control", options)
{
  std::string urdf_path =
    ament_index_cpp::get_package_share_directory("mlivr_description") + "/urdf/mlivr.urdf";

  robot_core_ = std::make_unique<mlivr_model::RobotCore>(urdf_path);

  model_ptr_ = std::make_shared<pinocchio::Model>(robot_core_->getModel());
  data_ptr_ = std::make_shared<pinocchio::Data>(*model_ptr_);

  RCLCPP_INFO(
    this->get_logger(), "Pinocchio model loaded. nq: %d, nv: %d", model_ptr_->nq, model_ptr_->nv);

  state_ = std::make_shared<crocoddyl::StateMultibody>(model_ptr_);
  actuation_ = std::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);

  RCLCPP_INFO(this->get_logger(), "Crocoddyl State and Actuation models initialized successfully.");

  current_q_.resize(14, 0.0);

  cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_cmds", 10);

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&WBControl::jointStateCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&WBControl::publishCommandStep, this));

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

void WBControl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (is_initialized_) return;

  for (size_t i = 0; i < 14 && i < msg->position.size(); ++i) {
    current_q_[i] = msg->position[i];
  }

  if (this->computeTrajectory()) {
    is_initialized_ = true;
  }
}

void WBControl::publishCommandStep()
{
  if (!is_initialized_ || optimized_xs_.empty() || playback_idx_ >= optimized_xs_.size()) {
    return;  // 再生完了、または未計算
  }

  // x = [q, v] のうち、qのサイズは21 (nq)
  Eigen::VectorXd q_opt = optimized_xs_[playback_idx_].head(model_ptr_->nq);

  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data.resize(14);
  for (int i = 0; i < 14; ++i) {
    cmd_msg.data[i] = q_opt(7 + i);  // 関節角度を抽出
  }
  cmd_pub_->publish(cmd_msg);

  playback_idx_++;
}

bool WBControl::computeTrajectory()
{
  RCLCPP_INFO(this->get_logger(), "=== Starting Trajectory Optimization ===");

  // 1. 初期状態 x0 の構築
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_ptr_->nq);
  q(6) = 1.0;  // クォータニオン w
  for (int i = 0; i < 14; ++i) q(7 + i) = current_q_[i];

  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_ptr_->nv);
  Eigen::VectorXd x0(model_ptr_->nq + model_ptr_->nv);
  x0 << q, v;

  // 2. 現在の Pose の取得
  pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, q);
  pinocchio::updateFramePlacements(*model_ptr_, *data_ptr_);

  std::string fixed_frame = "limb_1_link_gripper";
  std::string swing_frame = "limb_2_link_gripper";

  pinocchio::FrameIndex fixed_id = model_ptr_->getFrameId(fixed_frame);
  pinocchio::FrameIndex swing_id = model_ptr_->getFrameId(swing_frame);

  pinocchio::SE3 fixed_pose = data_ptr_->oMf[fixed_id];
  pinocchio::SE3 start_swing_pose = data_ptr_->oMf[swing_id];

  // 3. 目標の Pose を設定
  // ローカル座標系での移動量 (SE3) を作成
  pinocchio::SE3 local_offset = pinocchio::SE3::Identity();
  // 例: limb_2 の手先ローカル座標系の Z方向に 0.2m 移動
  local_offset.translation() = Eigen::Vector3d(0.0, -0.5, -0.2);
  // 現在のPoseに対して、右側から掛けることでローカル座標系での移動になる
  pinocchio::SE3 target_swing_pose = start_swing_pose * local_offset;

  // 4. シューティング問題の構築 (ホライズン: 100ステップ = 1.0秒)
  int T = 100;
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;

  for (int i = 0; i < T; ++i) {
    // ターゲットを初期位置から最終位置まで線形補間する (滑らかな誘導)
    double s = static_cast<double>(i) / T;
    pinocchio::SE3 current_target = start_swing_pose;
    current_target.translation() =
      (1.0 - s) * start_swing_pose.translation() + s * target_swing_pose.translation();

    auto model = createActionModel(x0, fixed_frame, fixed_pose, swing_frame, current_target);
    running_models.push_back(model);
  }

  // 終端モデル (最終的な目標地点)
  auto terminal_model =
    createActionModel(x0, fixed_frame, fixed_pose, swing_frame, target_swing_pose);

  auto problem = std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);
  crocoddyl::SolverFDDP solver(problem);

  // ターミナルに最適化のイテレーションログを出力する
  std::vector<std::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
  callbacks.push_back(std::make_shared<crocoddyl::CallbackVerbose>());
  solver.setCallbacks(callbacks);

  // 5. ソルバーの実行 (最大100イテレーション)
  solver.solve(solver.get_xs(), solver.get_us(), 100, false);

  // 結果の保存と再生準備
  optimized_xs_ = solver.get_xs();
  playback_idx_ = 0;

  RCLCPP_INFO(
    this->get_logger(), "Optimization completed! Trajectory length: %zu", optimized_xs_.size());
  return true;
}

std::shared_ptr<crocoddyl::ActionModelAbstract> WBControl::createActionModel(
  const Eigen::VectorXd & x0, const std::string & fixed_frame, const pinocchio::SE3 & fixed_pose,
  const std::string & swing_frame, const pinocchio::SE3 & target_pose)
{
  // 1. 接触モデル (Weld拘束) の設定
  auto contacts = std::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());

  pinocchio::FrameIndex fixed_id = model_ptr_->getFrameId(fixed_frame);

  // ContactModel6D で位置と姿勢の両方を完全に固定(Weld)する
  auto contact_6d = std::make_shared<crocoddyl::ContactModel6D>(
    state_, fixed_id, fixed_pose, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
    actuation_->get_nu());
  contacts->addContact(fixed_frame + "_weld_contact", contact_6d);

  // 2. コストモデル (目的関数) の設定
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  // [コストA: 目標追従] 動かす腕(limb_2等)を目標のPoseに到達させる (重み: 1e4)
  pinocchio::FrameIndex swing_id = model_ptr_->getFrameId(swing_frame);
  auto placement_residual = std::make_shared<crocoddyl::ResidualModelFramePlacement>(
    state_, swing_id, target_pose, actuation_->get_nu());
  costs->addCost(
    "swing_goal_cost", std::make_shared<crocoddyl::CostModelResidual>(state_, placement_residual),
    1e4);

  // [コストB: 状態の正則化] 姿勢や速度の暴走を防ぎ、初期状態 x0 付近を維持する (重み: 1e-1)
  auto x_residual =
    std::make_shared<crocoddyl::ResidualModelState>(state_, x0, actuation_->get_nu());
  costs->addCost(
    "state_reg_cost", std::make_shared<crocoddyl::CostModelResidual>(state_, x_residual), 1e-1);

  // [コストC: 制御入力の最小化] トルクの二乗和を小さくする (重み: 1e-4)
  auto u_residual = std::make_shared<crocoddyl::ResidualModelControl>(state_, actuation_->get_nu());
  costs->addCost(
    "control_reg_cost", std::make_shared<crocoddyl::CostModelResidual>(state_, u_residual), 1e-4);

  // // 今後の実装イメージ（※今はまだ追加しなくてOKです）
  // auto force_residual = std::make_shared<crocoddyl::ResidualModelContactForce>(
  //   state_, fixed_id, pinocchio::Force::Zero(), 6, actuation_->get_nu());
  // // 反力を極力ゼロに近づけるように重み（Weight）をつける
  // costs->addCost(
  //   "contact_force_cost", std::make_shared<crocoddyl::CostModelResidual>(state_, force_residual),
  //   1e-2);

  // 3. 微分動的モデル(DAM)の構築 (状態、駆動、接触、コストを統合)
  auto dmodel = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
    state_, actuation_, contacts, costs, 0.0, true);

  // 4. 積分モデル(IAM)の構築 (オイラー積分で1ステップ先の状態を計算、dt = 10ms)
  double dt = 0.01;
  return std::make_shared<crocoddyl::IntegratedActionModelEuler>(dmodel, dt);
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::WBControl)
