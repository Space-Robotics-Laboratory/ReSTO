cmake_minimum_required(VERSION 3.8)
project(mlivr_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS
    ${MUJOCO_PREFIX}/include
    $ENV{MUJOCO_PREFIX}/include
    $ENV{HOME}/.mujoco/mujoco-3.4.0/include
  PATHS
    /usr/local/include
    /usr/include
    /opt/mujoco/include
)

find_library(MUJOCO_LIBRARY
  NAMES mujoco
  HINTS
    ${MUJOCO_PREFIX}/lib
    $ENV{MUJOCO_PREFIX}/lib
    $ENV{HOME}/.mujoco/mujoco-3.4.0/lib
  PATHS
    /usr/local/lib
    /usr/lib
    /opt/mujoco/lib
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo not found! Please specify its path using -DMUJOCO_PREFIX=/path/to/mujoco")
endif()

message(STATUS "Found MuJoCo Include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "Found MuJoCo Library: ${MUJOCO_LIBRARY}")

if(NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco UNKNOWN IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
  )
endif()

# === C++ Library ===

add_library(mujoco_engine_lib SHARED
  src/mujoco_engine.cpp
  src/mujoco_renderer.cpp
)

target_include_directories(mujoco_engine_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mujoco_engine_lib
  mujoco::mujoco
  glfw
  Eigen3::Eigen
)

set_target_properties(mujoco_engine_lib PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# === ROS 2 Node Plugin ===

add_library(mlivr_sim_node SHARED
  src/mujoco_ros_node.cpp
)

target_include_directories(mlivr_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mlivr_sim_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)

target_link_libraries(mlivr_sim_node
  mujoco_engine_lib
)

rclcpp_components_register_node(mlivr_sim_node
  PLUGIN "mlivr_sim::MujocoRosNode"
  EXECUTABLE mlivr_sim_node_exec
)

# === Install ===

install(TARGETS
  mujoco_engine_lib
  mlivr_sim_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    config
    launch
    models
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)
ament_package()
