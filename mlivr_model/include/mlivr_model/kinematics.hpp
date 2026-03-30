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

#ifndef MLIVR_MODEL__KINEMATICS_HPP_
#define MLIVR_MODEL__KINEMATICS_HPP_

#include <Eigen/Dense>
#include <string>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "mlivr_model/core.hpp"

namespace mlivr_model
{

class Kinematics
{
public:
  explicit Kinematics(const RobotCore & core);

  virtual ~Kinematics() = default;

  pinocchio::SE3 solveFK(const Eigen::VectorXd & q, const std::string & frame_name);

  Eigen::MatrixXd computeJacobian(const Eigen::VectorXd & q, const std::string & frame_name);

private:
  const pinocchio::Model & model_;
  pinocchio::Data data_;
};

}  // namespace mlivr_model

#endif  // MLIVR_MODEL__KINEMATICS_HPP_
