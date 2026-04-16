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

#include "ramp/md/momentum_distribution.hpp"

namespace ramp
{
namespace md
{

MomentumDistribution::MomentumDistribution(int num_joints, int num_limbs)
: num_joints_(num_joints), num_limbs_(num_limbs)
{
  int max_rows = 6 * num_limbs_;

  A_matrix_.resize(6, 6);
  S_inv_buffer_.resize(num_joints_, max_rows);

  // Force the SVD solver to reserve internal buffers using dummy matrices
  Eigen::MatrixXd dummy_J = Eigen::MatrixXd::Zero(max_rows, num_joints_);
  svd_.compute(dummy_J, Eigen::ComputeThinU | Eigen::ComputeThinV);
}

Eigen::VectorXd MomentumDistribution::computeBaseVelocity(
  const Eigen::MatrixXd & H_b, const Eigen::MatrixXd & H_bm_sup, const Eigen::MatrixXd & J_b_sup,
  const Eigen::MatrixXd & J_m_sup, const Eigen::VectorXd & L_swing, double alpha, double max_lambda,
  double epsilon)
{
  Eigen::MatrixXd J_m_pinv = computePseudoInverseAdaptiveDLS(J_m_sup, max_lambda, epsilon);

  // A = H_b - H_bm_sup * J_m_sup^+ * J_b_sup
  A_matrix_ = H_b - H_bm_sup * J_m_pinv * J_b_sup;

  // b = -alpha * L_swing
  Eigen::VectorXd b = -alpha * L_swing;

  // (A * dot{x}_b = b)
  return A_matrix_.colPivHouseholderQr().solve(b);
}

Eigen::MatrixXd MomentumDistribution::computePseudoInverseAdaptiveDLS(
  const Eigen::MatrixXd & J, double max_lambda, double epsilon)
{
  // Compute the SVD of the current Jacobian
  svd_.compute(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const Eigen::VectorXd & singular_values = svd_.singularValues();

  // Clear only the necessary area to zero and use the buffer
  S_inv_buffer_.setZero();

  for (int i = 0; i < singular_values.size(); ++i) {
    double sigma = singular_values(i);
    double lambda = 0.0;

    if (sigma < epsilon) {
      double ratio = sigma / epsilon;
      lambda = max_lambda * (1.0 - ratio * ratio);
    }

    // sigma / (sigma^2 + lambda^2)
    S_inv_buffer_(i, i) = sigma / (sigma * sigma + lambda * lambda);
  }

  // J^+ = V * S_{inv} * U^T
  return svd_.matrixV() * S_inv_buffer_.block(0, 0, J.cols(), J.rows()) *
    svd_.matrixU().transpose();
}

}  // namespace md
}  // namespace ramp
