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

#include "mlivr_model/kinematics.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace mlivr_model
{

Kinematics::Kinematics(const RobotCore & core)
: model_(core.getModel()), data_(pinocchio::Data(model_))
{
}

Eigen::MatrixXd Kinematics::computeJacobian(
  const Eigen::VectorXd & q, const std::string & frame_name)
{
  if (!model_.existFrame(frame_name)) {
    throw std::invalid_argument("Frame '" + frame_name + "' does not exist in the model.");
  }

  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

  pinocchio::Data::Matrix6x J(6, model_.nv);
  J.setZero();

  pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::LOCAL, J);

  return J;
}

pinocchio::SE3 Kinematics::solveFK(const Eigen::VectorXd & q, const std::string & frame_name)
{
  if (!model_.existFrame(frame_name)) {
    throw std::invalid_argument("Frame '" + frame_name + "' does not exist in the model.");
  }

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
  return data_.oMf[frame_id];
}

bool Kinematics::solveNumericalIK(
  Eigen::VectorXd & q, const std::string & frame_name, const std::vector<std::string> & joint_names,
  const pinocchio::SE3 & desired_pose, double tolerance, int max_iterations, double damping_factor)
{
  if (!model_.existFrame(frame_name)) {
    throw std::invalid_argument("Frame '" + frame_name + "' does not exist.");
  }
  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

  // 動かす対象の関節IDを取得し、必要なヤコビアンの列数（sub_nv）を計算する
  std::vector<pinocchio::JointIndex> joint_ids;
  int sub_nv = 0;
  for (const auto & name : joint_names) {
    if (!model_.existJointName(name)) {
      throw std::invalid_argument("Joint '" + name + "' does not exist.");
    }
    pinocchio::JointIndex jid = model_.getJointId(name);
    joint_ids.push_back(jid);
    sub_nv += model_.joints[jid].nv();
  }

  // 抽出用の部分ヤコビアンと、Pinocchioから取得する全体ヤコビアン
  pinocchio::Data::Matrix6x J_sub(6, sub_nv);
  pinocchio::Data::Matrix6x J_full(6, model_.nv);

  for (int iter = 0; iter < max_iterations; ++iter) {
    // 現在の状態における順運動学を計算
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    // エンドエフェクタの現在姿勢と、目標姿勢との誤差（Local座標系）を計算
    const pinocchio::SE3 T_cur = data_.oMf[frame_id];
    pinocchio::SE3 T_err = T_cur.inverse() * desired_pose;
    Eigen::VectorXd error = pinocchio::log6(T_err).toVector();

    // 誤差が許容範囲内なら収束とみなす
    if (error.norm() < tolerance) {
      return true;
    }

    // ヤコビアンの計算 (Local座標系)
    J_full.setZero();
    pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::LOCAL, J_full);

    // 全体ヤコビアンから、動かす関節（joint_names）の列だけを抽出して J_sub を作る
    int col_offset = 0;
    for (const auto & jid : joint_ids) {
      int nv_i = model_.joints[jid].nv();
      int idx_v = model_.joints[jid].idx_v();
      J_sub.middleCols(col_offset, nv_i) = J_full.middleCols(idx_v, nv_i);
      col_offset += nv_i;
    }

    // Damped Least Squares (DLS) による関節速度の計算
    // A = J * J^T + lambda * I
    Eigen::MatrixXd A = J_sub * J_sub.transpose();
    A.diagonal().array() += damping_factor;  // 対角成分にダンピング係数を足す

    // dq_sub = J^T * A^-1 * error
    // (ldlt().solve() は inverse() よりも高速かつ数値的に安定です)
    Eigen::VectorXd dq_sub = J_sub.transpose() * A.ldlt().solve(error);

    // 計算された部分的な関節速度(dq_sub)を、全体サイズの速度ベクトル(v_full)にマッピングする
    Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model_.nv);
    col_offset = 0;
    for (const auto & jid : joint_ids) {
      int nv_i = model_.joints[jid].nv();
      int idx_v = model_.joints[jid].idx_v();
      v_full.segment(idx_v, nv_i) = dq_sub.segment(col_offset, nv_i);
      col_offset += nv_i;
    }

    // 状態 q の更新（オイラー積分だけでなくクォータニオン等の多様体も正しく積分する）
    q = pinocchio::integrate(model_, q, v_full);
  }

  // 最大ループ回数に達しても収束しなかった場合
  return false;
}

}  // namespace mlivr_model
