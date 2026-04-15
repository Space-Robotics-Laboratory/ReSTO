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

#include "mlivr_control/kj/kj_control.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace mlivr_control
{

KJControl::KJControl(const rclcpp::NodeOptions & options) : BaseController("kj_control", options)
{
  kinematics_ = std::make_unique<mlivr_model::Kinematics>(*robot_core_);

  target_joint_pos_.resize(num_joints_, 0.0);

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

bool KJControl::generateTrajectory()
{
  int nq = robot_core_->getModel().nq;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
  q.head(7) = current_base_pose_;
  for (int i = 0; i < num_joints_; ++i) {
    q(7 + i) = current_joint_pos_[i];
    target_joint_pos_[i] = current_joint_pos_[i];
  }

  pinocchio::SE3 pose_R = kinematics_->solveFK(q, ee_frames_[1]);

  Eigen::Vector3d start_pos = pose_R.translation();
  Eigen::Quaterniond start_quat(pose_R.rotation());

  auto displacement = Eigen::Vector3d(0.0, -0.2, 0.0);
  Eigen::Vector3d target_pos = start_pos + displacement;
  Eigen::Quaterniond target_quat = start_quat;

  Eigen::Vector3d swing_height = Eigen::Vector3d(0.0, 0.0, 0.05);
  Eigen::Vector3d mid_pos = start_pos + displacement / 2.0 + swing_height;
  Eigen::Quaterniond mid_quat = start_quat;

  duration_ = 10.0;

  trajectory_generator::VectorStateConstraint start_p_c{
    0.0, start_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  trajectory_generator::VectorStateConstraint end_p_c{
    duration_, target_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  auto pos_constraints = trajectory_generator::createBoundaryConditions(start_p_c, end_p_c);

  trajectory_generator::VectorStateConstraint mid_p_c{duration_ / 2.0, mid_pos};
  trajectory_generator::addConstraint(pos_constraints, mid_p_c);

  trajectory_generator::AngularStateConstraint start_o_c{
    0.0, start_quat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  trajectory_generator::AngularStateConstraint end_o_c{
    duration_, target_quat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  auto ori_constraints = trajectory_generator::createBoundaryConditions(start_o_c, end_o_c);

  trajectory_generator::AngularStateConstraint mid_o_c{duration_ / 2.0, mid_quat};
  trajectory_generator::addConstraint(ori_constraints, mid_o_c);

  pos_spline_ = std::make_unique<trajectory_generator::VectorSpline>(pos_constraints, 3);
  ori_spline_ = std::make_unique<trajectory_generator::OrientationSpline>(ori_constraints);

  trajectory_start_time_ = this->now().seconds();
  is_trajectory_active_ = true;
  RCLCPP_INFO(this->get_logger(), "Trajectory generation completed. Started tracking.");

  return true;
}

Eigen::VectorXd KJControl::computeCommandStep()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  int nq = robot_core_->getModel().nq;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
  q.head(7) = current_base_pose_;
  for (int i = 0; i < num_joints_; ++i) {
    q(7 + i) = current_joint_pos_[i];
  }

  std::string frame_L = ee_frames_[0];
  std::string frame_R = ee_frames_[1];

  pinocchio::SE3 pose_L, pose_R;
  Eigen::MatrixXd J_full_L, J_full_R;

  try {
    pose_L = kinematics_->solveFK(q, frame_L);
    pose_R = kinematics_->solveFK(q, frame_R);

    J_full_L = kinematics_->computeJacobian(q, frame_L);
    J_full_R = kinematics_->computeJacobian(q, frame_R);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Error: %s", e.what());
    return Eigen::VectorXd::Zero(1);
  }

  Eigen::MatrixXd J_std_L = J_full_L.block(0, 6, 6, num_joints_);
  Eigen::MatrixXd J_std_R = J_full_R.block(0, 6, 6, num_joints_);

  Eigen::MatrixXd J_stacked(12, num_joints_);
  J_stacked << J_std_L, J_std_R;

  Eigen::VectorXd v_target_local_L = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd v_target_local_R = Eigen::VectorXd::Zero(6);

  if (is_trajectory_active_) {
    double t = this->now().seconds() - trajectory_start_time_;
    double t_eval = (t > duration_) ? duration_ : t;

    // Planned ee pose on trajectory
    Eigen::Vector3d p_planned = pos_spline_->getPosition(t_eval);
    Eigen::Quaterniond q_planned = ori_spline_->getOrientation(t_eval);
    this->publishTargetTF(p_planned, q_planned, "planned/" + ee_frames_[1]);
    // Target ee pose
    Eigen::Vector3d p_target = pos_spline_->getPosition(duration_);
    Eigen::Quaterniond q_target = ori_spline_->getOrientation(duration_);
    this->publishTargetTF(p_target, q_target, "target/" + ee_frames_[1]);

    if (t > duration_) {
      v_target_local_R = Eigen::VectorXd::Zero(6);
    } else {
      Eigen::VectorXd v_spline_world = Eigen::VectorXd::Zero(6);
      v_spline_world.head<3>() = pos_spline_->getVelocity(t);
      v_spline_world.tail<3>() = ori_spline_->getAngularVelocity(t);

      pinocchio::Motion v_ff_world(v_spline_world);
      pinocchio::Motion v_ff_local_R = pose_R.actInv(v_ff_world);
      v_target_local_R = v_ff_local_R.toVector();
    }
  }

  Eigen::VectorXd v_stacked = Eigen::VectorXd::Zero(12);
  v_stacked.head<6>() = v_target_local_L;
  v_stacked.tail<6>() = v_target_local_R;

  // 3. 疑似逆行列 (Damped Least Squares) による関節速度の計算
  // double lambda = 0.0;  // Damping term for singularity avoidance
  // Eigen::MatrixXd A =
  //   J_stacked * J_stacked.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(12, 12);
  // Eigen::VectorXd q_dot_cmd_all = J_stacked.transpose() * A.inverse() * v_stacked;
  Eigen::VectorXd q_dot_cmd_all = J_stacked.inverse() * v_stacked;

  double dt = 0.01;
  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = this->now();

  for (int i = 0; i < num_joints_; ++i) {
    double safe_cmd = std::clamp(q_dot_cmd_all(i), -200.0, 200.0);
    target_joint_pos_[i] += safe_cmd * dt;

    cmd_msg.name.push_back(robot_core_->getModel().names[i + 2]);
    cmd_msg.position.push_back(target_joint_pos_[i]);
    cmd_msg.velocity.push_back(0.0);
    cmd_msg.effort.push_back(0.0);
  }
  cmd_pub_->publish(cmd_msg);

  return Eigen::VectorXd::Zero(1);
}

std::vector<Eigen::Vector3d> KJControl::getPlannedPath()
{
  std::vector<Eigen::Vector3d> path;
  if (!pos_spline_) return path;

  double dt = 0.05;
  for (double t = 0; t <= duration_; t += dt) {
    path.push_back(pos_spline_->getPosition(t));
  }
  return path;
}

}  // namespace mlivr_control

RCLCPP_COMPONENTS_REGISTER_NODE(mlivr_control::KJControl)
