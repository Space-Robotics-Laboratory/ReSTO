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
rclcpp_components_register_nodes(gj_control
  "mlivr_control::GJControl"
)
ament_auto_add_executable(gj_control_node
  src/gj_control_node.cpp
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
