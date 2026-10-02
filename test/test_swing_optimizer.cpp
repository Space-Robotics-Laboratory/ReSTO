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

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/multibody/sample-models.hpp>

#include "resto/swing_optimizer.hpp"

namespace
{

TEST(SwingOptimizer, HoldsSupportsAndMovesTheSwingFrameTowardItsTarget)
{
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::buildModels::humanoidRandom(*model, true);
  model->gravity.setZero();
  pinocchio::Data data(*model);
  const Eigen::VectorXd q0 = pinocchio::neutral(*model);
  pinocchio::framesForwardKinematics(*model, data, q0);

  resto::SwingProblem problem;
  problem.x0.resize(model->nq + model->nv);
  problem.x0 << q0, Eigen::VectorXd::Zero(model->nv);
  problem.support_frames = {"lleg6_joint", "rleg6_joint"};
  for (const auto & frame : problem.support_frames) {
    problem.support_targets.push_back(data.oMf[model->getFrameId(frame)]);
  }
  problem.swing_frame = "rarm6_joint";
  const pinocchio::SE3 swing_start = data.oMf[model->getFrameId(problem.swing_frame)];
  problem.swing_target = swing_start;
  problem.swing_target.translation().y() += 0.02;
  problem.duration = 1.0;

  resto::SolverParams solver_params;
  solver_params.dt = 0.1;
  solver_params.max_iter = 20;
  const resto::SwingOptimizer optimizer(model, solver_params, resto::WeightParams());
  const resto::Result result = optimizer.solve(problem);

  ASSERT_EQ(result.xs.size(), 11U);
  ASSERT_EQ(result.us.size(), 10U);
  EXPECT_TRUE(result.xs.front().isApprox(problem.x0));
  pinocchio::framesForwardKinematics(*model, data, result.xs.back().head(model->nq));
  const pinocchio::SE3 swing_end = data.oMf[model->getFrameId(problem.swing_frame)];
  EXPECT_LT(
    (swing_end.translation() - problem.swing_target.translation()).norm(),
    (swing_start.translation() - problem.swing_target.translation()).norm());

  problem.support_targets.pop_back();
  EXPECT_THROW((void)optimizer.solve(problem), std::invalid_argument);
}

}  // namespace
