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

#include "ramp/lrst/low_reaction_swing_trajectory.hpp"

#include <iostream>
#include <stdexcept>

namespace ramp
{
namespace lrst
{

// 階乗(nCr)を計算するヘルパー関数（nchoosekに相当）
double nChoosek(int n, int k)
{
  if (k > n) return 0;
  if (k * 2 > n) k = n - k;
  if (k == 0) return 1;
  double result = n;
  for (int i = 2; i <= k; ++i) {
    result *= (n - i + 1);
    result /= i;
  }
  return result;
}

LowReactionSwingTrajectory::LowReactionSwingTrajectory(
  fbml::Kinematics * kinematics, fbml::Dynamics * dynamics, int num_joints, int num_limbs)
: num_joints_(num_joints), num_limbs_(num_limbs), kinematics_(kinematics), dynamics_(dynamics)
{
}

std::vector<double> LowReactionSwingTrajectory::optimizeTrajectory(
  const OptimizationWeights & weights)
{
  current_weights_ = weights;

  // 始点と終点から、中間点の初期推定値(initial_guess)を自動計算する
  Eigen::Vector3d start_pos = bezier_base_matrix_.col(0);
  Eigen::Vector3d end_pos = bezier_base_matrix_.col(7);
  Eigen::Vector3d mid_pos = (start_pos + end_pos) / 2.0;

  // Z方向（高さ）に step_height を足す
  mid_pos.z() += current_weights_.step_height;

  // MATLABの x0 (要素数6) を作成
  std::vector<double> initial_guess = {
    mid_pos.x(), mid_pos.y(), mid_pos.z(),  // 制御点4の初期値
    mid_pos.x(), mid_pos.y(), mid_pos.z()   // 制御点5の初期値
  };

  unsigned int num_vars = initial_guess.size();

  // NLoptの初期化
  // LN_BOBYQA は境界制約付きの勾配不要(Derivative-Free)最適化アルゴリズムで、
  // MATLABのfminconの代替として非常に優秀です。
  nlopt::opt opt(nlopt::LN_BOBYQA, num_vars);

  // 評価関数と、それに渡すデータ（thisポインタ）をセット
  opt.set_min_objective(LowReactionSwingTrajectory::objectiveWrapper, this);

  // 最適化変数の上下限（Bounds）の設定（必要に応じて調整）
  std::vector<double> lower_bounds(num_vars, -10.0);
  std::vector<double> upper_bounds(num_vars, 10.0);
  opt.set_lower_bounds(lower_bounds);
  opt.set_upper_bounds(upper_bounds);

  // 終了条件の設定（許容誤差や最大評価回数）
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(1000);  // 無限ループを防ぐため、最大評価回数を設定

  std::vector<double> x_opt = initial_guess;
  double min_cost = 0.0;

  try {
    // ▼ 修正: 未使用の result 変数を削除して直接実行する
    opt.optimize(x_opt, min_cost);
    std::cout << "[LRST] Optimization successful. Minimum cost: " << min_cost << std::endl;
  } catch (std::exception & e) {
    std::cerr << "[LRST] NLopt failed: " << e.what() << std::endl;
  }

  return x_opt;
}

void LowReactionSwingTrajectory::setBoundaryConditions(
  const Eigen::Vector3d & start_pos, const Eigen::Vector3d & end_pos)
{
  // 3x8 のベジェ曲線制御点行列を初期化 (MATLABの AA)
  bezier_base_matrix_ = Eigen::MatrixXd::Zero(3, 8);

  // 始点での位置・速度・加速度がゼロ（滑らかな動き出し）になるよう、
  // 最初の4つの制御点を始点に一致させる
  bezier_base_matrix_.col(0) = start_pos;
  bezier_base_matrix_.col(1) = start_pos;
  bezier_base_matrix_.col(2) = start_pos;
  bezier_base_matrix_.col(3) = start_pos;  // ※ここは後で最適化で動かします

  // 終点での位置・速度・加速度がゼロになるよう、
  // 最後の4つの制御点を終点に一致させる
  bezier_base_matrix_.col(4) = end_pos;  // ※ここは後で最適化で動かします
  bezier_base_matrix_.col(5) = end_pos;
  bezier_base_matrix_.col(6) = end_pos;
  bezier_base_matrix_.col(7) = end_pos;
}

void LowReactionSwingTrajectory::setRobotState(
  const Eigen::VectorXd & q_init, const std::string & swing_frame_name,
  const std::vector<std::string> & swing_joint_names)
{
  q_init_ = q_init;
  swing_frame_name_ = swing_frame_name;
  swing_joint_names_ = swing_joint_names;
}

// staticラッパー関数
double LowReactionSwingTrajectory::objectiveWrapper(
  const std::vector<double> & x, std::vector<double> & grad, void * data)
{
  // 今回使用するアルゴリズム（LN_BOBYQA）は勾配を必要としないため、
  // grad は空（計算不要）の状態で呼ばれます。
  if (!grad.empty()) {
    // もし勾配が必要なアルゴリズム（LD_SLSQPなど）に変更した場合は、
    // ここに自前で数値微分を計算する処理を書く必要があります。
  }

  // voidポインタをLowReactionSwingTrajectoryのインスタンスにキャストして、実際の計算関数を呼ぶ
  LowReactionSwingTrajectory * optimizer = static_cast<LowReactionSwingTrajectory *>(data);
  return optimizer->computeCost(x);
}

Eigen::Vector3d LowReactionSwingTrajectory::computeBezierPosition(
  double t, const Eigen::MatrixXd & P)
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double tf = current_weights_.tf;
  int m = 7;  // 7次ベジェ曲線

  for (int i = 0; i <= m; ++i) {
    double b = nChoosek(m, i) * std::pow(t / tf, i) * std::pow((tf - t) / tf, m - i);
    pos += b * P.col(i);
  }
  return pos;
}

// 実際の評価関数（MATLABの opt_function_low_reaction_bez に相当）
double LowReactionSwingTrajectory::computeCost(const std::vector<double> & x)
{
  // 1. 最適化変数 x (要素数6) を使って、ベジェ曲線の制御点行列 P (3x8) を完成させる
  Eigen::MatrixXd P = bezier_base_matrix_;
  P.col(3) = Eigen::Vector3d(x[0], x[1], x[2]);  // MATLABの AA(:,4) に相当
  P.col(4) = Eigen::Vector3d(x[3], x[4], x[5]);  // MATLABの AA(:,5) に相当

  double tf = current_weights_.tf;
  double dt = current_weights_.dt;
  int num_steps = static_cast<int>(tf / dt) + 1;

  double max_force = 0.0;
  double sum_force = 0.0;
  double max_height = -1e9;

  Eigen::VectorXd q_prev = q_init_;
  Eigen::VectorXd q_dot_prev;
  Eigen::VectorXd L_prev = Eigen::VectorXd::Zero(6);

  pinocchio::SE3 initial_pose = kinematics_->solveFK(q_init_, swing_frame_name_);
  Eigen::Matrix3d R_des = initial_pose.rotation();

  // 2. 離散時間ループ (MATLABの t = t0:dt:tf に相当)
  for (int i = 0; i < num_steps; ++i) {
    double t = i * dt;
    if (t > tf) t = tf;

    // ベジェ曲線から目標手先位置を取得
    Eigen::Vector3d x_des = computeBezierPosition(t, P);
    max_height = std::max(max_height, x_des.z());

    // ▼ 修正: 回転は初期姿勢のまま維持し、位置だけをベジェ曲線に従わせる
    pinocchio::SE3 pose_des(R_des, x_des);

    // IKを解く（q_prev を初期値として渡し、結果で上書きされる）
    Eigen::VectorXd q = q_prev;
    bool ik_success =
      kinematics_->solveNumericalIK(q, swing_frame_name_, pose_des, swing_joint_names_);

    if (!ik_success) {
      return 1e9;  // ペナルティ
    }

    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(q.size());
    if (i > 0) {
      q_dot = (q - q_prev) / dt;  // 微分で関節速度を算出
    }

    Eigen::MatrixXd H_b, H_bm;
    dynamics_->computePartitionedMassMatrices(q, H_b, H_bm);

    // 遊脚の運動量 L を計算
    Eigen::VectorXd L = H_bm * q_dot;

    if (i > 0) {
      // 運動量の微分 L_dot (＝反力/モーメント) を計算
      Eigen::VectorXd L_dot = (L - L_prev) / dt;

      double force_norm = L_dot.head<3>().norm();  // 並進反力のノルム (Ld_lin)

      max_force = std::max(max_force, force_norm);
      sum_force += force_norm;
    }

    // 次のステップのための更新
    q_prev = q;
    q_dot_prev = q_dot;
    L_prev = L;
  }

  double mean_force = sum_force / (num_steps - 1);

  // 3. 評価関数の計算 (ペナルティの合算)
  double cost = current_weights_.k_mom_max * max_force + current_weights_.k_mom_ave * mean_force;
  // + current_weights_.k_height_max * std::abs(current_weights_.step_height - (max_height - P(2,0)));

  return cost;
}

}  // namespace lrst
}  // namespace ramp
