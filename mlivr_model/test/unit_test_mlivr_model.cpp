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

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mlivr_model/core.hpp"

namespace mlivr_model
{

TEST(RobotCoreTest, LoadValidUrdf)
{
  std::string desc_share = ament_index_cpp::get_package_share_directory("mlivr_description");
  std::string urdf_path = desc_share + "/urdf/mlivr.urdf";

  EXPECT_NO_THROW({
    RobotCore core(urdf_path);
    const pinocchio::Model & model = core.getModel();

    EXPECT_EQ(model.name, "MLIVR");

    EXPECT_GT(model.njoints, 0);
  });
}

}  // namespace mlivr_model

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
