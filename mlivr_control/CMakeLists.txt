cmake_minimum_required(VERSION 3.8)
project(mlivr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
ament_auto_find_build_dependencies()

ament_auto_add_library(base_controller
  src/base_controller.cpp
)

# Kinematic Jacobian Controller
ament_auto_add_library(kj_control
  src/kj/kj_control.cpp
)
target_link_libraries(kj_control
  base_controller
  trajectory_generator::trajectory_generator
)
rclcpp_components_register_nodes(kj_control
  "mlivr_control::PDControl"
)
ament_auto_add_executable(kj_control_node
  src/kj/kj_control_node.cpp
)

# Generalized Jacobian Controller
ament_auto_add_library(gj_control
  src/gj/gj_control.cpp
)
target_link_libraries(gj_control
  base_controller
  trajectory_generator::trajectory_generator
)
rclcpp_components_register_nodes(gj_control
  "mlivr_control::GJControl"
)
ament_auto_add_executable(gj_control_node
  src/gj/gj_control_node.cpp
)

# Generalized Jacobian Controller
ament_auto_add_library(ramp_control
  src/ramp/ramp_control.cpp
)
target_link_libraries(ramp_control
  base_controller
  trajectory_generator::trajectory_generator
)
rclcpp_components_register_nodes(ramp_control
  "mlivr_control::RAMPControl"
)
ament_auto_add_executable(ramp_control_node
  src/ramp/ramp_control_node.cpp
)

# Whole-Body Controller (Trajectory Optimization)
ament_auto_add_library(wb_control
  src/to/wb_control.cpp
  src/to/wbc_solver.cpp
)
target_link_libraries(wb_control
  base_controller
  trajectory_generator::trajectory_generator
)
rclcpp_components_register_nodes(wb_control
  "mlivr_control::WBControl"
)
ament_auto_add_executable(wb_control_node
  src/to/wb_control_node.cpp
)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
