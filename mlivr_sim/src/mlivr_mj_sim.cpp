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

#include "mlivr_sim/mlivr_mj_sim.hpp"

// #include <algorithm>
#include <cstdlib>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#define DEBUG false

namespace mlivr_mj_sim
{

// Static instance pointer initialization
MujocoSim * MujocoSim::instance_ = nullptr;

MujocoSim::MujocoSim(const rclcpp::NodeOptions & options) : rclcpp::Node("mlivr_mj_sim", options)
{
  instance_ = this;

  kNumLimbs_ = 2;

  // Publisher
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  for (int limb_id = 0; limb_id < kNumLimbs_; ++limb_id) {
    ee_ft_pubs_.push_back(this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/limb_" + std::to_string(limb_id + 1) + "/ee_ft_sensor", 10));
  }

  // Subscriber
  cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "joint_cmds", 10, std::bind(&MujocoSim::jointCmdCallback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // === Initialize MuJoCo ===

  std::string sim_share = ament_index_cpp::get_package_share_directory("mlivr_sim");
  std::string desc_share = ament_index_cpp::get_package_share_directory("mlivr_description");
  std::string symlink_cmd = "ln -snf " + desc_share + " " + sim_share + "/../mlivr_description";
  int ret = system(symlink_cmd.c_str());
  (void)ret;

  std::string kXmlDir = sim_share + "/models/world.xml";

  char error_msg[1000] = "";
  m_ = mj_loadXML(kXmlDir.c_str(), nullptr, error_msg, sizeof(error_msg));
  if (!m_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load MuJoCo model: %s", error_msg);
    return;
  }
  d_ = mj_makeData(m_);

  bool extract_keyframe = this->declare_parameter<bool>("extract_keyframe", false);

  if (extract_keyframe) {
    // Pose Extraction  // TODO: Activate only pose extraction mode set in yaml
    d_->qpos[8] = -1.57;
    d_->qpos[14] = -1.57;
    for (int i = 0; i < 50000; i++) {
      mj_step(m_, d_);

      // Overwrite the base pose at every step forcefully
      d_->qpos[0] = 0.0;  // x
      d_->qpos[1] = 0.1;  // y
      d_->qpos[2] = 1.0;  // z
      d_->qpos[3] = 1.0;  // qw
      d_->qpos[4] = 0.0;  // qx
      d_->qpos[5] = 0.0;  // qy
      d_->qpos[6] = 0.0;  // qz

      // Reduce the overall system speed by 90%
      for (int j = 0; j < m_->nv; j++) {
        d_->qvel[j] *= 0.1;
      }
    }
    std::cout << "\n\n";
    std::cout << "<keyframe>\n  <key name=\"init_grasp\" qpos=\"";
    for (int i = 0; i < m_->nq; i++) {
      std::cout << d_->qpos[i] << " ";
    }
    std::cout << "\"/>\n</keyframe>\n\n";
  } else {
    // Load keyframe
    int key_id = mj_name2id(m_, mjOBJ_KEY, "init_grasp");
    if (key_id >= 0) {
      mj_resetDataKeyframe(m_, d_, key_id);
      mj_forward(m_, d_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Keyframe 'init_grasp' not found. Using default posture.");
    }
  }

  int num_joints = m_->nv - 6;
  target_qpos_.resize(num_joints, 0.0);

  // Set target joint pos as initial joint pos
  for (int i = 0; i < num_joints; i++) {
    target_qpos_[i] = d_->qpos[7 + i];
  }

  // Register MuJoCo control callbacks
  mjcb_control = MujocoSim::mjcbControlWrapper;

  // Launch sim thread
  is_running_ = true;
  sim_thread_ = std::thread(&MujocoSim::simLoop, this);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

MujocoSim::~MujocoSim()
{
  is_running_ = false;
  if (sim_thread_.joinable()) {
    sim_thread_.join();
  }

  mjcb_control = nullptr;
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);

  instance_ = nullptr;

  RCLCPP_INFO(this->get_logger(), "/%s node is destructed.", this->get_name());
}

void MujocoSim::publishFTSensorData(const rclcpp::Time & now)
{
  if (!m_ || !d_) {
    return;
  }

  for (int limb_id = 0; limb_id < kNumLimbs_; ++limb_id) {
    std::string force_sensor_name = "limb_" + std::to_string(limb_id + 1) + "_ee_force";
    std::string torque_sensor_name = "limb_" + std::to_string(limb_id + 1) + "_ee_torque";

    int force_id = mj_name2id(m_, mjOBJ_SENSOR, force_sensor_name.c_str());
    int torque_id = mj_name2id(m_, mjOBJ_SENSOR, torque_sensor_name.c_str());

    if (force_id >= 0 && torque_id >= 0) {
      geometry_msgs::msg::WrenchStamped msg;
      msg.header.stamp = now;
      msg.header.frame_id = "limb_" + std::to_string(limb_id + 1) + "_gripper_site";

      int f_adr = m_->sensor_adr[force_id];
      int t_adr = m_->sensor_adr[torque_id];

      msg.wrench.force.x = d_->sensordata[f_adr + 0];
      msg.wrench.force.y = d_->sensordata[f_adr + 1];
      msg.wrench.force.z = d_->sensordata[f_adr + 2];
      msg.wrench.torque.x = d_->sensordata[t_adr + 0];
      msg.wrench.torque.y = d_->sensordata[t_adr + 1];
      msg.wrench.torque.z = d_->sensordata[t_adr + 2];

      ee_ft_pubs_[limb_id]->publish(msg);
    }
  }
}

void MujocoSim::jointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  for (size_t i = 0; i < target_qpos_.size() && i < msg->data.size(); i++) {
    target_qpos_[i] = msg->data[i];
  }
}

void MujocoSim::mjcbControlWrapper(const mjModel * m, mjData * d)
{
  if (instance_) {
    instance_->computePDControl(m, d);
  }
}

void MujocoSim::computePDControl(const mjModel * m, mjData * d)
{
  (void)m;
  double kp = 50.0;
  double kd = 10.0;

  int num_joints = m->nv - 6;

  std::lock_guard<std::mutex> lock(target_mutex_);
  for (int i = 0; i < num_joints; i++) {
    double error = target_qpos_[i] - d->qpos[7 + i];
    double error_dot = 0.0 - d->qvel[6 + i];

    double tau = (kp * error) + (kd * error_dot);

    // // ★ 物理演算の爆発を防ぐためのトルク制限（クランプ）
    // // -150.0 Nm 〜 150.0 Nm の範囲に強制的に収める
    // tau = std::clamp(tau, -150.0, 150.0);

    d->qfrc_applied[6 + i] = tau;
  }
}

void MujocoSim::simLoop()
{
  if (!glfwInit()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLFW in sim thread.");
    return;
  }
  window_ = glfwCreateWindow(1200, 900, "MLIVR ROS 2 Sim", NULL, NULL);
  if (!window_) {
    glfwTerminate();
    return;
  }
  // Initialize rendering related
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  glfwSetScrollCallback(window_, MujocoSim::scrollCallback);
  glfwSetMouseButtonCallback(window_, MujocoSim::mouseButtonCallback);
  glfwSetCursorPosCallback(window_, MujocoSim::mouseMoveCallback);

  mjv_defaultScene(&scn_);
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjr_defaultContext(&con_);

  // Visualization in sim window
  opt_.frame = mjFRAME_SITE;
  opt_.flags[mjVIS_CONTACTFORCE] = 1;
  opt_.flags[mjVIS_CONTACTPOINT] = 1;

  mjv_makeScene(m_, &scn_, 2000);
  mjr_makeContext(m_, &con_, mjFONTSCALE_150);

  cam_.azimuth = 0.0;      // deg
  cam_.elevation = -10.0;  // deg
  cam_.distance = 3.0;
  cam_.lookat[0] = 0.5;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 1.0;

  sensor_msgs::msg::JointState joint_msg;
  // joint_msg.name = {"left_joint1", ...}; // TODO Set joint names

  auto last_pub_time = this->now();
  const double kPubFrequency = 60.0;  // Hz
  const double kPubRate = 1.0 / kPubFrequency;

  int num_joints = m_->nv - 6;

  while (is_running_ && !glfwWindowShouldClose(window_) && rclcpp::ok()) {
    mj_step(m_, d_);

    auto current_time = this->now();
    if ((current_time - last_pub_time).seconds() >= kPubRate) {
      joint_msg.header.stamp = current_time;
      joint_msg.position.clear();
      for (int i = 0; i < num_joints; i++) {
        joint_msg.position.push_back(d_->qpos[7 + i]);
        joint_msg.velocity.push_back(d_->qvel[6 + i]);
      }
      joint_state_pub_->publish(joint_msg);
      publishFTSensorData(current_time);

      last_pub_time = current_time;
    }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);

    glfwSwapBuffers(window_);
    glfwPollEvents();

    broadcastSiteTransforms();
  }

  mjr_freeContext(&con_);
  mjv_freeScene(&scn_);
  glfwDestroyWindow(window_);
  glfwTerminate();
}

void MujocoSim::scrollCallback(GLFWwindow * window, double xoffset, double yoffset)
{
  (void)window;
  (void)xoffset;
  if (!instance_ || !instance_->m_) return;
  mjv_moveCamera(
    instance_->m_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &instance_->scn_, &instance_->cam_);
}

void MujocoSim::mouseButtonCallback(GLFWwindow * window, int button, int act, int mods)
{
  (void)button;
  (void)act;
  (void)mods;
  if (!instance_) return;

  // Update the button's state
  instance_->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  instance_->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  instance_->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // Save the mouse coordinates at the moment of the click
  glfwGetCursorPos(window, &instance_->lastx_, &instance_->lasty_);
}

void MujocoSim::mouseMoveCallback(GLFWwindow * window, double xpos, double ypos)
{
  if (!instance_ || !instance_->m_) return;

  if (!instance_->button_left_ && !instance_->button_middle_ && !instance_->button_right_) {
    return;
  }

  // Mouse movement distance
  double dx = xpos - instance_->lastx_;
  double dy = ypos - instance_->lasty_;
  instance_->lastx_ = xpos;
  instance_->lasty_ = ypos;

  // Get window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // Check if Shift key is pressed
  bool mod_shift =
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // Determine the camera operation (action) based on the button
  mjtMouse action;
  if (instance_->button_right_) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (instance_->button_left_) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(
    instance_->m_, action, dx / height, dy / height, &instance_->scn_, &instance_->cam_);
}

void MujocoSim::broadcastSiteTransforms()
{
  if (!m_ || !d_) return;

  rclcpp::Time now = this->now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(m_->nsite);

  for (int i = 0; i < m_->nsite; ++i) {
    const char * site_name = mj_id2name(m_, mjOBJ_SITE, i);

    if (!site_name) continue;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = std::string(site_name);

    // --- Get translation ---
    // d_->site_xpos = [x0, y0, z0, x1, y1, z1, ...]
    tf_msg.transform.translation.x = d_->site_xpos[3 * i + 0];
    tf_msg.transform.translation.y = d_->site_xpos[3 * i + 1];
    tf_msg.transform.translation.z = d_->site_xpos[3 * i + 2];

    // --- Get rotation and convert to quaternion ---
    // d_->site_xmat = [R00, R01, R02, R10... ]
    int mat_offset = 9 * i;
    Eigen::Matrix3d R;
    R << d_->site_xmat[mat_offset + 0], d_->site_xmat[mat_offset + 1],
      d_->site_xmat[mat_offset + 2], d_->site_xmat[mat_offset + 3], d_->site_xmat[mat_offset + 4],
      d_->site_xmat[mat_offset + 5], d_->site_xmat[mat_offset + 6], d_->site_xmat[mat_offset + 7],
      d_->site_xmat[mat_offset + 8];

    Eigen::Quaterniond q(R);
    tf_msg.transform.rotation.w = q.w();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();

    transforms.push_back(tf_msg);
  }

  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

}  // namespace mlivr_mj_sim

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_mj_sim::MujocoSim)
