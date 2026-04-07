cmake_minimum_required(VERSION 3.8)
project(mlivr_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(glfw3 REQUIRED)

ament_auto_add_library(mlivr_mj_sim
  src/mlivr_mj_sim.cpp
)

set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.4.0")

target_include_directories(mlivr_mj_sim PUBLIC
  ${MUJOCO_DIR}/include
)
target_link_directories(mlivr_mj_sim PUBLIC
  ${MUJOCO_DIR}/lib
)
target_link_libraries(mlivr_mj_sim
  mujoco
  glfw
)

set_target_properties(mlivr_mj_sim PROPERTIES INSTALL_RPATH "${MUJOCO_DIR}/lib")

rclcpp_components_register_nodes(mlivr_mj_sim
  "mlivr_mj_sim::MujocoSim"
)

ament_auto_add_executable(mlivr_mj_sim_node
  src/mlivr_mj_sim_node.cpp
)

install(
  DIRECTORY
    config
    launch
    models
  DESTINATION
    share/${PROJECT_NAME}/
)

# gtest
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
