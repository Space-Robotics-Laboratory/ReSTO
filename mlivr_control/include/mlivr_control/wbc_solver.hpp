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
  int max_iter = 500;
};

struct WeightParams
{
  double state_reg = 0.0;
  double control_reg = 0.0;

  double state_limits = 0.0;
  double control_limits = 0.0;

  double sw_ee_tracking = 0.0;
  double sup_ee_tracking = 0.0;
  double ee_vel_damping = 0.0;

  double env_collision = 0.0;

  double momentum_reg = 0.0;
};

struct WeightScheduleParams
{
  double s_accel = 0.2;
  double s_decel = 0.8;
  double accel_multi = 1.0;
  double decel_multi = 1.0;
};

struct WbcSolverParams
{
  SolverParams solver;
  WeightParams weights;

  WeightScheduleParams ctrl_reg_schedule;
  WeightScheduleParams ee_vel_schedule;

  std::vector<std::string> ee_frames;
};

struct TaskPhase
{
  // Frame name and target ee pose list for target-tracking
  std::map<std::string, pinocchio::SE3> ee_tracking_targets;

  // Frame name list for environment collision
  std::vector<std::string> collision_frames;

  // Selection vector of support limbs
  std::vector<std::string> support_limbs;

  std::map<std::string, double> ee_z_lower_bounds;

  bool is_terminal = false;
};

class WbcSolver
{
public:
  explicit WbcSolver(std::shared_ptr<pinocchio::Model> model, const WbcSolverParams & params);
  ~WbcSolver() = default;

  bool computeTrajectory(
    const Eigen::VectorXd & base_pose, const Eigen::VectorXd & base_twist,
    const std::vector<double> & current_joint_pos, const std::string & support_ee_frame,
    const std::string & swing_ee_frame, const Eigen::Vector3d & world_translation_offset);

  void setParams(const WbcSolverParams & params) { params_ = params; }

  const std::vector<Eigen::VectorXd> & getOptimizedXs() const { return optimized_xs_; }
  const std::vector<Eigen::VectorXd> & getOptimizedUs() const { return optimized_us_; }

private:
  std::shared_ptr<crocoddyl::ActionModelAbstract> createActionModel(
    const WeightParams & weights, const Eigen::VectorXd & x0, const TaskPhase & phase);

  void addStateAndControlRegularizationCosts(
    std::shared_ptr<crocoddyl::CostModelSum> & costs, const WeightParams & weights,
    const Eigen::VectorXd & x0);

  void addStateAndControlLimitsCost(
    std::shared_ptr<crocoddyl::CostModelSum> & costs, const WeightParams & weights);

  void addEndEffectorTrackingCost(
    std::shared_ptr<crocoddyl::CostModelSum> & costs, const WeightParams & weights,
    const TaskPhase & phase);

  void addEndEffectorVelocityDampingCost(
    std::shared_ptr<crocoddyl::CostModelSum> & costs, const WeightParams & weights);

  void addEnvironmentCollisionCost(
    std::shared_ptr<crocoddyl::CostModelSum> & costs, const WeightParams & weights,
    const TaskPhase & phase);

  void addMomentumRegularizationCost(
    std::shared_ptr<crocoddyl::CostModelSum> & costs, const WeightParams & weights);

  double computeWeightMultiplier(double s, const WeightScheduleParams & sched);

  std::shared_ptr<pinocchio::Model> model_ptr_;
  std::shared_ptr<pinocchio::Data> data_ptr_;

  std::shared_ptr<crocoddyl::StateMultibody> state_;
  std::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;

  std::vector<Eigen::VectorXd> optimized_xs_;
  std::vector<Eigen::VectorXd> optimized_us_;

  WbcSolverParams params_;
};

}  // namespace mlivr_control

#endif  // MLIVR_CONTROL__WBC_SOLVER_HPP_
