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

LowReactionSwingTrajectory::LowReactionSwingTrajectory(int num_joints, int num_limbs)
{
  // KinematicsやDynamicsの初期化処理
  (void)num_joints;
  (void)num_limbs;
}

std::vector<double> LowReactionSwingTrajectory::optimizeTrajectory(
  const std::vector<double> & initial_guess, const OptimizationWeights & weights)
{
  current_weights_ = weights;
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
    // 最適化の実行
    nlopt::result result = opt.optimize(x_opt, min_cost);
    std::cout << "[LRST] Optimization successful. Minimum cost: " << min_cost << std::endl;
  } catch (std::exception & e) {
    std::cerr << "[LRST] NLopt failed: " << e.what() << std::endl;
  }

  return x_opt;
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

// 実際の評価関数（MATLABの opt_function_low_reaction_bez に相当）
double LowReactionSwingTrajectory::computeCost(const std::vector<double> & x)
{
  double cost = 0.0;

  // ==============================================================
  // ここにMATLABコードの計算フローを移植します。
  // 1. 変数 `x` （ベジェ曲線の制御点）を用いて、軌道を離散時間(t=0~T)で生成
  // 2. 各時刻のCartesian手先位置に対して IK (数値解法) を解き、関節角度・速度を計算
  // 3. 連成慣性行列(H_bm)と関節速度から、各時刻の運動量(L)を計算
  // 4. 運動量の最大値(max)や平均値(mean)、および床衝突ペナルティを cost に足し込む
  // ==============================================================

  // 仮の計算（xの二乗和を最小化するダミー処理）
  for (double val : x) {
    cost += val * val;
  }

  return cost;
}
}  // namespace lrst
}  // namespace ramp
