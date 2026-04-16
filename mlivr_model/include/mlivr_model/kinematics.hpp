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

#ifndef MLIVR_MODEL__KINEMATICS_HPP_
#define MLIVR_MODEL__KINEMATICS_HPP_

#include <Eigen/Dense>
#include <string>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "mlivr_model/core.hpp"

namespace mlivr_model
{

class Kinematics
{
public:
  explicit Kinematics(const RobotCore & core);

  virtual ~Kinematics() = default;

  Eigen::MatrixXd computeJacobian(const Eigen::VectorXd & q, const std::string & frame_name);

  pinocchio::SE3 solveFK(const Eigen::VectorXd & q, const std::string & frame_name);

  /**
   * @brief 指定した関節のみを動かして、目標フレームを目標姿勢に一致させる数値的逆運動学(IK)
   * @param q 現在の一般化座標（計算結果で上書きされます）
   * @param frame_name 目標とするエンドエフェクタ等のフレーム名
   * @param joint_names IKで動かすことを許可する関節の名前リスト
   * @param desired_pose 目標となるSE3姿勢
   * @param tolerance 収束判定の許容誤差
   * @param max_iterations 最大ループ回数
   * @param damping_factor DLSのダンピング係数（特異点付近の安定化用）
   * @return bool 収束した場合は true
   */
  bool solveNumericalIK(
    Eigen::VectorXd & q, const std::string & frame_name,
    const std::vector<std::string> & joint_names, const pinocchio::SE3 & desired_pose,
    double tolerance = 1e-4, int max_iterations = 100, double damping_factor = 1e-2);

private:
  const pinocchio::Model & model_;
  pinocchio::Data data_;
};

}  // namespace mlivr_model

#endif  // MLIVR_MODEL__KINEMATICS_HPP_
