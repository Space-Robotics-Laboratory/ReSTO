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

#include "mlivr_sim/mujoco_engine.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace mlivr_sim
{

MujocoEngine * MujocoEngine::instance_ = nullptr;

MujocoEngine::MujocoEngine(const std::string & xml_path, bool extract_keyframe)
{
  char error_msg[1000] = "";
  m_ = mj_loadXML(xml_path.c_str(), nullptr, error_msg, sizeof(error_msg));
  if (!m_) {
    throw std::runtime_error("Failed to load MuJoCo model: " + std::string(error_msg));
  }
  d_ = mj_makeData(m_);

  if (extract_keyframe) {
    // Pose Extraction
    // d_->qpos[2] = 1.5;
    // d_->qpos[7] = -0.239565;
    d_->qpos[8] = -0.861929;
    d_->qpos[9] = 1.65969;
    d_->qpos[10] = -0.797827;
    d_->qpos[11] = 1.33203;
    d_->qpos[12] = 1.57081;
    // d_->qpos[13] = -0.23803;
    d_->qpos[14] = -0.949546;
    d_->qpos[15] = 1.63655;
    d_->qpos[16] = -0.686937;
    d_->qpos[17] = 1.33196;
    d_->qpos[18] = -1.57082;
    for (int i = 0; i < 1000000; i++) {
      mj_step(m_, d_);

      // Overwrite the base pose at every step forcefully
      // d_->qpos[0] = 0.02;  // x
      // d_->qpos[1] = 0.1;   // y
      // d_->qpos[2] = 1.0;   // z
      // d_->qpos[3] = 1.0;   // qw
      // d_->qpos[4] = 0.0;   // qx
      // d_->qpos[5] = 0.0;   // qy
      // d_->qpos[6] = 0.0;   // qz
      d_->qpos[0] = 0.52;            // x
      d_->qpos[1] = -0.25;           // y
      d_->qpos[2] = 1.0;             // z
      d_->qpos[3] = 1.0 / sqrt(2);   // qw
      d_->qpos[4] = 0.0;             // qx
      d_->qpos[5] = 0.0;             // qy
      d_->qpos[6] = -1.0 / sqrt(2);  // qz

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
    int key_id = mj_name2id(m_, mjOBJ_KEY, "init_grasp");
    if (key_id >= 0) {
      mj_resetDataKeyframe(m_, d_, key_id);
      mj_forward(m_, d_);
    }
  }

  num_joints_ = m_->nv - 6;
  target_qpos_.resize(num_joints_, 0.0);
  target_qvel_.resize(num_joints_, 0.0);
  target_tau_ff_.resize(num_joints_, 0.0);
  for (int i = 0; i < num_joints_; i++) {
    target_qpos_[i] = d_->qpos[7 + i];
  }

  instance_ = this;
  mjcb_control = MujocoEngine::mjcbControlWrapper;

  std::cout << "[MujocoEngine] Successfully initialized." << std::endl;
}

MujocoEngine::~MujocoEngine()
{
  mjcb_control = nullptr;
  instance_ = nullptr;

  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

void MujocoEngine::setTargetJointPos(const std::vector<double> & target_qpos)
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  for (size_t i = 0; i < target_qpos_.size() && i < target_qpos.size(); i++) {
    target_qpos_[i] = target_qpos[i];
  }
}

void MujocoEngine::setControlCommand(
  const std::vector<double> & q, const std::vector<double> & v, const std::vector<double> & tau)
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  for (int i = 0; i < num_joints_; i++) {
    target_qpos_[i] = (i < (int)q.size()) ? q[i] : target_qpos_[i];
    target_qvel_[i] = (i < (int)v.size()) ? v[i] : 0.0;
    target_tau_ff_[i] = (i < (int)tau.size()) ? tau[i] : 0.0;
  }
}

void MujocoEngine::setPDGains(double kp, double kd)
{
  kp_ = kp;
  kd_ = kd;
}

void MujocoEngine::step()
{
  mj_step(m_, d_);
}

void MujocoEngine::computePDControl(const mjModel * m, mjData * d)
{
  (void)m;
  std::lock_guard<std::mutex> lock(target_mutex_);
  for (int i = 0; i < num_joints_; i++) {
    double error_q = target_qpos_[i] - d->qpos[7 + i];
    double error_v = target_qvel_[i] - d->qvel[6 + i];

    double tau = target_tau_ff_[i] + (kp_ * error_q) + (kd_ * error_v);

    d->qfrc_applied[6 + i] = tau;
  }
}

void MujocoEngine::mjcbControlWrapper(const mjModel * m, mjData * d)
{
  if (instance_) {
    instance_->computePDControl(m, d);
  }
}

}  // namespace mlivr_sim
