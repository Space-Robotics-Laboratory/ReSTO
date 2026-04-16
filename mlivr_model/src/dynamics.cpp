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

#include "mlivr_model/dynamics.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace mlivr_model
{

Dynamics::Dynamics(const RobotCore & core) : model_(core.getModel()), data_(pinocchio::Data(model_))
{
}

Eigen::MatrixXd Dynamics::computeGeneralizedJacobian(
  const Eigen::VectorXd & q, const std::string & frame_name)
{
  if (!model_.existFrame(frame_name)) {
    throw std::invalid_argument("Frame '" + frame_name + "' does not exist.");
  }
  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

  // Compute inertial matrix M (Composite Rigid Body Algorithm)
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::Data::Matrix6x J_full(6, model_.nv);
  J_full.setZero();
  pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::LOCAL, J_full);

  const int njoints = model_.nv - 6;

  Eigen::MatrixXd M_b = data_.M.block(0, 0, 6, 6);
  Eigen::MatrixXd M_bm = data_.M.block(0, 6, 6, njoints);

  Eigen::MatrixXd J_b = J_full.block(0, 0, 6, 6);
  Eigen::MatrixXd J_m = J_full.block(0, 6, 6, njoints);

  // Compute generalized jacobian (J* = J_m - J_b * M_b^{-1} * M_bm)
  Eigen::MatrixXd J_g = J_m - J_b * M_b.llt().solve(M_bm);

  return J_g;
}

void Dynamics::computeInertiaMatrices(
  const Eigen::VectorXd & q, Eigen::MatrixXd & H_b, Eigen::MatrixXd & H_bm)
{
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  const int njoints = model_.nv - 6;

  H_b = data_.M.block(0, 0, 6, 6);
  H_bm = data_.M.block(0, 6, 6, njoints);
}

}  // namespace mlivr_model
