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

#include "mlivr_model/core.hpp"

#include <iostream>
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>

namespace mlivr_model
{

RobotCore::RobotCore(const std::string & urdf_path)
{
  try {
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);

    model_.gravity.linear() = Eigen::Vector3d::Zero();

    // === Synchronize the physical parameters on the MuJoCo side with Pinocchio ===
    const int num_actuated_joints = model_.nv - 6;
    model_.rotorInertia.tail(num_actuated_joints).setConstant(0.5);  // armature
    model_.friction.tail(num_actuated_joints).setConstant(0.5);      // damping

    // For debug
    std::cout << "[mlivr_model] Successfully loaded URDF." << std::endl;
    std::cout << "[mlivr_model] - Number of joints: " << model_.njoints << std::endl;
    std::cout << "[mlivr_model] - Degrees of freedom (nv): " << model_.nv << std::endl;
    std::cout << "[mlivr_model] - Configuration size (nq): " << model_.nq << std::endl;

  } catch (const std::exception & e) {
    std::cerr << "[mlivr_model] Failed to build Pinocchio model from URDF: " << urdf_path
              << std::endl;
    std::cerr << e.what() << std::endl;
    throw;
  }
}

}  // namespace mlivr_model
