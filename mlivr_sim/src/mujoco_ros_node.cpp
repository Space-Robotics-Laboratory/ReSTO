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

#include "mlivr_sim/mujoco_ros_node.hpp"

#include <cstdlib>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mlivr_sim
{

MujocoRosNode::MujocoRosNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mlivr_sim", options)
{
  // --- ROS 2 parameters ---
  bool extract_keyframe = this->declare_parameter<bool>("extract_keyframe", false);
  double kp = this->declare_parameter<double>("gains.kp", 500.0);
  double kd = this->declare_parameter<double>("gains.kd", 30.0);

  // --- Publisher ---
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  for (int i = 0; i < num_limbs_; ++i) {
    ee_ft_pubs_.push_back(this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/limb_" + std::to_string(i + 1) + "/ee_ft_sensor", 10));
  }

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  // --- Subscriber ---
  cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_cmds", 10, std::bind(&MujocoRosNode::jointCmdCallback, this, std::placeholders::_1));

  // --- TF Broadcaster ---
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // --- Engine ---
  std::string sim_share = ament_index_cpp::get_package_share_directory("mlivr_sim");
  std::string xml_path = sim_share + "/models/world.xml";

  // Symbolic link for description
  std::string desc_share = ament_index_cpp::get_package_share_directory("mlivr_description");
  std::string symlink_cmd = "ln -snf " + desc_share + " " + sim_share + "/../mlivr_description";
  if (system(symlink_cmd.c_str()) != 0) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to create symlink for mlivr_description. Meshes might not load.");
  }

  engine_ = std::make_unique<MujocoEngine>(xml_path, extract_keyframe);
  engine_->setPDGains(kp, kd);

  // --- Launch Sim Thread ---
  is_running_ = true;
  sim_thread_ = std::thread(&MujocoRosNode::simLoop, this);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

MujocoRosNode::~MujocoRosNode()
{
  is_running_ = false;
  if (sim_thread_.joinable()) {
    sim_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "/%s node is destructed.", this->get_name());
}

void MujocoRosNode::publishJointStates(const rclcpp::Time & now)
{
  mjModel * m = engine_->getModel();
  mjData * d = engine_->getData();
  int num_joints = m->nv - 6;

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now;

  for (int i = 1; i < m->njnt; i++) {
    const char * jnt_name = mj_id2name(m, mjOBJ_JOINT, i);

    if (jnt_name) {
      msg.name.push_back(jnt_name);
      msg.position.push_back(d->qpos[m->jnt_qposadr[i]]);  // HACK: No need 7 if use jnt_qposadr
      msg.velocity.push_back(d->qvel[m->jnt_dofadr[i]]);   // HACK: No need 6 if use jnt_dofadr
    }
  }

  joint_state_pub_->publish(msg);
}

void MujocoRosNode::publishFTSensorData(const rclcpp::Time & now)
{
  mjModel * m = engine_->getModel();
  mjData * d = engine_->getData();

  for (int limb_id = 0; limb_id < num_limbs_; ++limb_id) {
    std::string f_name = "limb_" + std::to_string(limb_id + 1) + "_ee_force";
    std::string t_name = "limb_" + std::to_string(limb_id + 1) + "_ee_torque";

    int f_id = mj_name2id(m, mjOBJ_SENSOR, f_name.c_str());
    int t_id = mj_name2id(m, mjOBJ_SENSOR, t_name.c_str());

    if (f_id >= 0 && t_id >= 0) {
      geometry_msgs::msg::WrenchStamped msg;
      msg.header.stamp = now;
      msg.header.frame_id = "limb_" + std::to_string(limb_id + 1) + "_gripper_site";

      int f_adr = m->sensor_adr[f_id];
      int t_adr = m->sensor_adr[t_id];

      msg.wrench.force.x = d->sensordata[f_adr + 0];
      msg.wrench.force.y = d->sensordata[f_adr + 1];
      msg.wrench.force.z = d->sensordata[f_adr + 2];
      msg.wrench.torque.x = d->sensordata[t_adr + 0];
      msg.wrench.torque.y = d->sensordata[t_adr + 1];
      msg.wrench.torque.z = d->sensordata[t_adr + 2];

      ee_ft_pubs_[limb_id]->publish(msg);
    }
  }
}

void MujocoRosNode::publishOdometry(const rclcpp::Time & now)
{
  mjData * d = engine_->getData();

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now;
  msg.header.frame_id = "world";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = d->qpos[0];
  msg.pose.pose.position.y = d->qpos[1];
  msg.pose.pose.position.z = d->qpos[2];
  // HACK: MuJoCo quaternion [w, x, y, z]
  msg.pose.pose.orientation.w = d->qpos[3];
  msg.pose.pose.orientation.x = d->qpos[4];
  msg.pose.pose.orientation.y = d->qpos[5];
  msg.pose.pose.orientation.z = d->qpos[6];

  msg.twist.twist.linear.x = d->qvel[0];
  msg.twist.twist.linear.y = d->qvel[1];
  msg.twist.twist.linear.z = d->qvel[2];
  msg.twist.twist.angular.x = d->qvel[3];
  msg.twist.twist.angular.y = d->qvel[4];
  msg.twist.twist.angular.z = d->qvel[5];

  odom_pub_->publish(msg);
}

void MujocoRosNode::jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  engine_->setControlCommand(msg->position, msg->velocity, msg->effort);
}

void MujocoRosNode::broadcastTransforms(const rclcpp::Time & now)
{
  mjModel * m = engine_->getModel();
  mjData * d = engine_->getData();

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(1 + m->nsite);

  geometry_msgs::msg::TransformStamped base_tf_msg;
  base_tf_msg.header.stamp = now;
  base_tf_msg.header.frame_id = "world";
  base_tf_msg.child_frame_id = "base_link";

  base_tf_msg.transform.translation.x = d->qpos[0];
  base_tf_msg.transform.translation.y = d->qpos[1];
  base_tf_msg.transform.translation.z = d->qpos[2];
  base_tf_msg.transform.rotation.w = d->qpos[3];
  base_tf_msg.transform.rotation.x = d->qpos[4];
  base_tf_msg.transform.rotation.y = d->qpos[5];
  base_tf_msg.transform.rotation.z = d->qpos[6];

  transforms.push_back(base_tf_msg);

  for (int i = 0; i < m->nsite; ++i) {
    const char * site_name = mj_id2name(m, mjOBJ_SITE, i);
    if (!site_name) continue;

    geometry_msgs::msg::TransformStamped site_tf_msg;
    site_tf_msg.header.stamp = now;
    site_tf_msg.header.frame_id = "world";
    site_tf_msg.child_frame_id = std::string(site_name);

    site_tf_msg.transform.translation.x = d->site_xpos[3 * i + 0];
    site_tf_msg.transform.translation.y = d->site_xpos[3 * i + 1];
    site_tf_msg.transform.translation.z = d->site_xpos[3 * i + 2];

    int mat_offset = 9 * i;
    Eigen::Matrix3d R;
    R << d->site_xmat[mat_offset + 0], d->site_xmat[mat_offset + 1], d->site_xmat[mat_offset + 2],
      d->site_xmat[mat_offset + 3], d->site_xmat[mat_offset + 4], d->site_xmat[mat_offset + 5],
      d->site_xmat[mat_offset + 6], d->site_xmat[mat_offset + 7], d->site_xmat[mat_offset + 8];

    Eigen::Quaterniond q(R);
    site_tf_msg.transform.rotation.w = q.w();
    site_tf_msg.transform.rotation.x = q.x();
    site_tf_msg.transform.rotation.y = q.y();
    site_tf_msg.transform.rotation.z = q.z();

    transforms.push_back(site_tf_msg);
  }

  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

void MujocoRosNode::simLoop()
{
  renderer_ = std::make_unique<MujocoRenderer>(engine_.get());

  auto last_pub_time = this->now();
  const double kPubRate = 1.0 / 60.0;  // 60Hz

  // Real-time at the start of the simulation
  auto start_real_time = this->now();

  while (is_running_ && rclcpp::ok() && (!renderer_ || !renderer_->isWindowClosed())) {
    // HACK: Advance the time in MuJoCo in sync with real-time
    double elapsed_real_time = (this->now() - start_real_time).seconds();

    // HACK: Run the simulation at high speed until MuJoCo's internal time (d_->time) catches up with real time
    while (engine_->getData()->time < elapsed_real_time) {
      engine_->step();
    }

    auto current_time = this->now();
    if ((current_time - last_pub_time).seconds() >= kPubRate) {
      publishJointStates(current_time);
      publishFTSensorData(current_time);
      publishOdometry(current_time);

      last_pub_time = current_time;
    }

    broadcastTransforms(current_time);

    if (renderer_) {
      renderer_->render();
    }
  }

  renderer_.reset();
}

}  // namespace mlivr_sim

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_sim::MujocoRosNode)
