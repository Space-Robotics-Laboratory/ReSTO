cmake_minimum_required(VERSION 3.8)
project(mlivr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

install(
  DIRECTORY
    config
    meshes
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

add_executable(env_visualizer src/env_visualizer.cpp)
ament_target_dependencies(env_visualizer
  rclcpp
  visualization_msgs
)

install(TARGETS
  env_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
