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

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace ramp
{
namespace lrst
{

// Helper function for calculating the factorial (nCr)
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

Eigen::MatrixXd LowReactionSwingTrajectory::optimizeTrajectory(
  const SolverParams & solver_params, const WeightParams & weight_params)
{
  std::cout << "[LRST] Optimizing trajectory..." << std::endl;

  solver_params_ = solver_params;
  weight_params_ = weight_params;

  // Compute initial guess of mid-point based on start and end pos
  Eigen::Vector3d start_pos = bezier_base_matrix_.col(0);
  Eigen::Vector3d end_pos = bezier_base_matrix_.col(7);
  Eigen::Vector3d mid_pos = (start_pos + end_pos) / 2.0;

  mid_pos.z() += solver_params_.step_height;

  // Initial optimization variable x0
  std::vector<double> initial_guess = {mid_pos.x(), mid_pos.y(), mid_pos.z(),
                                       mid_pos.x(), mid_pos.y(), mid_pos.z()};

  unsigned int num_vars = initial_guess.size();

  // Initialize NLopt
  nlopt::opt opt(nlopt::LN_BOBYQA, num_vars);

  // Set the evaluation function and the data (this pointer) passed to it
  opt.set_min_objective(LowReactionSwingTrajectory::objectiveWrapper, this);

  // Bounds of optimization variables
  std::vector<double> lower_bounds(num_vars, -10.0);
  std::vector<double> upper_bounds(num_vars, 10.0);
  opt.set_lower_bounds(lower_bounds);
  opt.set_upper_bounds(upper_bounds);

  // Termination conditions
  opt.set_xtol_rel(solver_params_.relative_tol);
  opt.set_maxeval(solver_params_.max_iter);

  std::vector<double> x_opt = initial_guess;
  double min_cost = 0.0;

  try {
    opt.optimize(x_opt, min_cost);
    std::cout << "[LRST] Optimization successful. Minimum cost: " << min_cost << std::endl;
  } catch (std::exception & e) {
    std::cerr << "[LRST] NLopt failed: " << e.what() << std::endl;
  }

  Eigen::MatrixXd P_opt = bezier_base_matrix_;
  P_opt.col(3) = Eigen::Vector3d(x_opt[0], x_opt[1], x_opt[2]);
  P_opt.col(4) = Eigen::Vector3d(x_opt[3], x_opt[4], x_opt[5]);

  return P_opt;
}

void LowReactionSwingTrajectory::setBoundaryConditions(
  const Eigen::Vector3d & start_pos, const Eigen::Vector3d & end_pos)
{
  constexpr int kDof = 3;
  bezier_base_matrix_ = Eigen::MatrixXd::Zero(kDof, bezier_order_ + 1);

  // Constraints for start point
  bezier_base_matrix_.col(0) = start_pos;
  bezier_base_matrix_.col(1) = start_pos;
  bezier_base_matrix_.col(2) = start_pos;

  // Initialize mid-point constraints (subject to change during optimization)
  bezier_base_matrix_.col(3) = start_pos;
  bezier_base_matrix_.col(4) = end_pos;

  // Constraints for end point
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

double LowReactionSwingTrajectory::objectiveWrapper(
  const std::vector<double> & x, std::vector<double> & grad, void * data)
{
  // In the case of algorithms that do not require gradients (LN_BOBYQA), grad is called in an empty state (no computation required).
  if (!grad.empty()) {
    // For algorithms that require a gradient (such as LD_SLSQP), you need to write code here to compute the numerical derivative yourself.
  }

  // Cast void pointer to instance of LowReactionSwingTrajectory and call actual calculation function
  LowReactionSwingTrajectory * optimizer = static_cast<LowReactionSwingTrajectory *>(data);
  return optimizer->computeCost(x);
}

Eigen::Vector3d LowReactionSwingTrajectory::computeBezierPosition(
  double t, const Eigen::MatrixXd & P) const
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double tf = solver_params_.step_duration;
  int m = bezier_order_;

  for (int i = 0; i <= m; ++i) {
    double b = nChoosek(m, i) * std::pow(t / tf, i) * std::pow((tf - t) / tf, m - i);
    pos += b * P.col(i);
  }
  return pos;
}

Eigen::Vector3d LowReactionSwingTrajectory::computeBezierVelocity(
  double t, const Eigen::MatrixXd & P) const
{
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double tf = solver_params_.step_duration;
  int m = bezier_order_;
  if (t >= tf) return vel;

  // Differentiation formula for Bézier curves
  for (int i = 0; i <= m - 1; ++i) {
    double b = nChoosek(m - 1, i) * std::pow(t / tf, i) * std::pow((tf - t) / tf, m - 1 - i);
    vel += b * (static_cast<double>(m) / tf) * (P.col(i + 1) - P.col(i));
  }
  return vel;
}

double LowReactionSwingTrajectory::computeCost(const std::vector<double> & x)
{
  // Complete the Bézier curve control point matrix P (3x8) using the optimization variable x (6 elements)
  Eigen::MatrixXd P = bezier_base_matrix_;
  P.col(3) = Eigen::Vector3d(x[0], x[1], x[2]);
  P.col(4) = Eigen::Vector3d(x[3], x[4], x[5]);

  double tf = solver_params_.step_duration;
  double dt = solver_params_.dt;
  int num_steps = static_cast<int>(tf / dt) + 1;

  double max_force = 0.0;
  double max_moment = 0.0;
  double sum_force = 0.0;
  double max_height = -1e9;
  double sum_height = 0.0;

  double ground_z = P(2, 0);

  Eigen::VectorXd q_prev = q_init_;
  Eigen::VectorXd L_prev = Eigen::VectorXd::Zero(6);

  pinocchio::SE3 initial_pose = kinematics_->solveFK(q_init_, swing_frame_name_);
  Eigen::Matrix3d R_des = initial_pose.rotation();

  // Discrete-time loop
  for (int i = 0; i < num_steps; ++i) {
    double t = i * dt;
    if (t > tf) t = tf;

    Eigen::Vector3d x_des = computeBezierPosition(t, P);

    double current_height = x_des.z() - ground_z;
    max_height = std::max(max_height, current_height);
    sum_height += current_height;

    pinocchio::SE3 pose_des(R_des, x_des);  // Maintain rotation in the starting position

    Eigen::VectorXd q = q_prev;
    bool ik_success =
      kinematics_->solveNumericalIK(q, swing_frame_name_, pose_des, swing_joint_names_);

    if (!ik_success) {
      return 1e9;  // High penalty for kinematic infeasible pose
    }

    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(num_joints_);
    if (i > 0) {
      q_dot = (q.tail(num_joints_) - q_prev.tail(num_joints_)) / dt;
    }

    Eigen::MatrixXd H_b, H_bm;
    dynamics_->computePartitionedMassMatrices(q, H_b, H_bm);

    // HACK: Momentum of the swing limb
    Eigen::VectorXd L = H_bm * q_dot;

    if (i > 0) {
      // Differential of momentum
      Eigen::VectorXd L_dot = (L - L_prev) / dt;

      double force_norm = L_dot.head<3>().norm();   // Linear component of L_dot
      double moment_norm = L_dot.tail<3>().norm();  // Angular component of L_dot

      max_force = std::max(max_force, force_norm);
      max_moment = std::max(max_moment, moment_norm);
      sum_force += force_norm;
    }

    q_prev = q;
    L_prev = L;
  }

  double mean_force = sum_force / (num_steps - 1);
  double mean_height = sum_height / num_steps;

  // === Costs ===

  double force_cost = weight_params_.force_max * max_force;
  double moment_cost = weight_params_.moment_max * max_moment;

  double max_step_height_cost =
    weight_params_.step_height_max * std::abs(solver_params_.step_height - max_height);
  double ave_step_height_cost =
    weight_params_.step_height_ave * std::abs(solver_params_.step_height - mean_height);

  double cost = force_cost + moment_cost + max_step_height_cost + ave_step_height_cost;

  return cost;
}

}  // namespace lrst
}  // namespace ramp
