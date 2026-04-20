cmake_minimum_required(VERSION 3.8)
project(ramp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(NLopt REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)

set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
find_package(pinocchio REQUIRED)
find_package(fbml REQUIRED)

# add library
add_library(${PROJECT_NAME} SHARED
  src/lrst/low_reaction_swing_trajectory.cpp
  src/md/momentum_distribution.cpp
)

# target include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  ${NLOPT_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# link libraries
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${NLOPT_LIBRARIES}
  pinocchio::pinocchio
)

ament_target_dependencies(${PROJECT_NAME} fbml)

# export dependencies
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  Eigen3
  pinocchio
  fbml
)

# install rules
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
