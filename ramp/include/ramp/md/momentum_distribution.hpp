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

#ifndef RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_
#define RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "ramp/visibility_control.h"

namespace ramp
{
namespace md
{

struct VelocityCommand
{
  Eigen::VectorXd base_velocity;
  Eigen::VectorXd support_limb_joint_velocities;
};

class MomentumDistribution
{
public:
  RAMP_PUBLIC
  explicit MomentumDistribution(int num_joints, int num_limbs);
  virtual ~MomentumDistribution() = default;

  RAMP_PUBLIC
  VelocityCommand computeVelocities(
    const Eigen::MatrixXd & H_b, const Eigen::MatrixXd & H_bm_sup, const Eigen::MatrixXd & J_b_sup,
    const Eigen::MatrixXd & J_m_sup, const Eigen::VectorXd & L_swing, double alpha = 1.0,
    double max_lambda = 0.1, double epsilon = 0.05);

private:
  Eigen::MatrixXd computePseudoInverseAdaptiveDLS(
    const Eigen::MatrixXd & J, double max_lambda, double epsilon);

  int num_joints_;
  int num_limbs_;

  Eigen::MatrixXd A_matrix_;
  Eigen::MatrixXd S_inv_buffer_;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
};

}  // namespace md
}  // namespace ramp

#endif  // RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_
