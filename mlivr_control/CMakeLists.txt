cmake_minimum_required(VERSION 3.8)
project(mlivr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
ament_auto_find_build_dependencies()

ament_auto_add_library(gj_control
  src/gj_control.cpp
)
target_link_libraries(gj_control
  trajectory_generator::trajectory_generator
)
rclcpp_components_register_nodes(gj_control
  "mlivr_control::GJControl"
)
ament_auto_add_executable(gj_control_node
  src/gj_control_node.cpp
)

ament_auto_add_library(wb_control
  src/wb_control.cpp
  src/wbc_solver.cpp
)
target_link_libraries(wb_control
  trajectory_generator::trajectory_generator
)
rclcpp_components_register_nodes(wb_control
  "mlivr_control::WBControl"
)
ament_auto_add_executable(wb_control_node
  src/wb_control_node.cpp
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
