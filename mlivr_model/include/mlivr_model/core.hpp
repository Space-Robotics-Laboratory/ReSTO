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

#ifndef MLIVR_MODEL__CORE_HPP_
#define MLIVR_MODEL__CORE_HPP_

#include <string>

#include <pinocchio/multibody/model.hpp>

namespace mlivr_model
{

class RobotCore
{
public:
  explicit RobotCore(const std::string & urdf_path);

  virtual ~RobotCore() = default;

  const pinocchio::Model & getModel() const { return model_; }

private:
  pinocchio::Model model_;
};

}  // namespace mlivr_model

#endif  // MLIVR_MODEL__CORE_HPP_
