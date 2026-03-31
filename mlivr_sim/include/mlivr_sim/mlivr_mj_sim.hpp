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

#ifndef MLIVR_SIM__MLIVR_MJ_SIM_HPP_
#define MLIVR_SIM__MLIVR_MJ_SIM_HPP_

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "mlivr_sim/visibility_control.hpp"

namespace mlivr_mj_sim
{

class MujocoSim : public rclcpp::Node
{
public:
  MLIVR_SIM_PUBLIC
  explicit MujocoSim(const rclcpp::NodeOptions & options);

  virtual ~MujocoSim();

private:
  void jointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // MuJoCo Control Callback (Static wrapper)
  static void mjcbControlWrapper(const mjModel * m, mjData * d);

  void computePDControl(const mjModel * m, mjData * d);

  void simLoop();

  void broadcastSiteTransforms();

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // MuJoCo Variables
  mjModel * m_ = nullptr;
  mjData * d_ = nullptr;
  GLFWwindow * window_ = nullptr;
  mjvScene scn_;
  mjvCamera cam_;
  mjvOption opt_;
  mjrContext con_;

  // Simulation Thread
  std::thread sim_thread_;
  std::atomic<bool> is_running_;

  // Control Data & Mutex (Thread Safety)
  std::mutex target_mutex_;
  std::vector<double> target_qpos_;

  // Singleton instance pointer for the static callback
  static MujocoSim * instance_;

  // Variables/Callbacks for mouse control
  bool button_left_ = false;
  bool button_middle_ = false;
  bool button_right_ = false;
  double lastx_ = 0.0;
  double lasty_ = 0.0;
  static void scrollCallback(GLFWwindow * window, double xoffset, double yoffset);
  static void mouseButtonCallback(GLFWwindow * window, int button, int act, int mods);
  static void mouseMoveCallback(GLFWwindow * window, double xpos, double ypos);
};

}  // namespace mlivr_mj_sim

#endif  // MLIVR_SIM__MLIVR_MJ_SIM_HPP_
