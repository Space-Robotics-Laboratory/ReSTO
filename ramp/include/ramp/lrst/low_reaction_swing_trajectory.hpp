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
#include <vector>

#include "ramp/visibility_control.h"

#include <nlopt.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "mlivr_model/dynamics.hpp"
#include "mlivr_model/kinematics.hpp"

namespace ramp
{
namespace lrst
{

struct OptimizationWeights
{
  double k_mom_max = 1.0;
  double k_mom_ave = 1.0;
  double k_height_max = 1.0;
  double step_height = 0.05;
  double dt = 0.01;  // 軌道計算のタイムステップ [s]
  double tf = 2.0;   // 遊脚の移動時間 [s]
};

class LowReactionSwingTrajectory
{
public:
  RAMP_PUBLIC
  explicit LowReactionSwingTrajectory(
    mlivr_model::Kinematics * kinematics, mlivr_model::Dynamics * dynamics, int num_joints,
    int num_limbs);
  virtual ~LowReactionSwingTrajectory() = default;

  RAMP_PUBLIC
  /**
   * @brief 遊脚の反力（運動量変化）が最小となるベジェ曲線の制御点を計算する
   * @param initial_guess 最適化変数の初期値（ベジェ曲線のフリーな制御点パラメータ等）
   * @return 最適化された変数の配列
   */
  std::vector<double> optimizeTrajectory(const OptimizationWeights & weights);

  RAMP_PUBLIC
  void setBoundaryConditions(const Eigen::Vector3d & start_pos, const Eigen::Vector3d & end_pos);

  RAMP_PUBLIC
  void setRobotState(
    const Eigen::VectorXd & q_init, const std::string & swing_frame_name,
    const std::vector<std::string> & swing_joint_names);

private:
  // --- NLoptに渡すためのstaticラッパー関数 ---
  static double objectiveWrapper(
    const std::vector<double> & x, std::vector<double> & grad, void * data);

  // 7次ベジェ曲線の位置を計算するヘルパー関数
  Eigen::Vector3d computeBezierPosition(double t, const Eigen::MatrixXd & P);

  // --- 実際の評価関数（コスト計算） ---
  double computeCost(const std::vector<double> & x);

  OptimizationWeights current_weights_;

  Eigen::MatrixXd bezier_base_matrix_;

  int num_joints_;
  int num_limbs_;

  mlivr_model::Kinematics * kinematics_;
  mlivr_model::Dynamics * dynamics_;

  Eigen::VectorXd q_init_;
  std::string swing_frame_name_;
  std::vector<std::string> swing_joint_names_;
};

}  // namespace lrst
}  // namespace ramp

#endif  // RAMP__MD__LOW_REACTION_SWING_TRAJECTORY_HPP_
