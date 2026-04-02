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

#ifndef MLIVR_CONTROL__WBC_SOLVER_HPP_
#define MLIVR_CONTROL__WBC_SOLVER_HPP_

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace mlivr_control
{

struct SolverParams
{
  int horizon_steps = 100;
  double dt = 0.01;
};

struct WeightParams
{
  double swing_goal = 1e4;
  double state_reg = 1e-1;
  double control_reg = 1e-4;

  double state_limits = 1e3;
  double control_limits = 1e3;
};

struct WbcSolverParams
{
  SolverParams solver;
  WeightParams weights;
};

struct TaskPhase
{
  // 把持(Weld)するフレームとその姿勢
  std::map<std::string, pinocchio::SE3> active_contacts;

  // 目標追従させるフレームとその目標姿勢
  std::map<std::string, pinocchio::SE3> swing_targets;
};

class WbcSolver
{
public:
  explicit WbcSolver(std::shared_ptr<pinocchio::Model> model, const WbcSolverParams & params);
  ~WbcSolver() = default;

  bool computeTrajectory(
    const std::vector<double> & current_q_14, const std::string & fixed_frame,
    const std::string & swing_frame, const Eigen::Vector3d & local_translation_offset);

  void setParams(const WbcSolverParams & params) { params_ = params; }

  const std::vector<Eigen::VectorXd> & getOptimizedXs() const { return optimized_xs_; }

private:
  std::shared_ptr<crocoddyl::ActionModelAbstract> createActionModel(
    const Eigen::VectorXd & x0, const TaskPhase & phase);

  std::shared_ptr<crocoddyl::ActionModelAbstract> createImpulseModel(
    const Eigen::VectorXd & x0, const std::string & impact_frame);

  std::shared_ptr<pinocchio::Model> model_ptr_;
  std::shared_ptr<pinocchio::Data> data_ptr_;

  std::shared_ptr<crocoddyl::StateMultibody> state_;
  std::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;

  std::vector<Eigen::VectorXd> optimized_xs_;

  WbcSolverParams params_;
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__WBC_SOLVER_HPP_
