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

#include "mlivr_sim/mujoco_renderer.hpp"

#include <iostream>
#include <stdexcept>

namespace mlivr_sim
{

MujocoRenderer::MujocoRenderer(MujocoEngine * engine) : engine_(engine)
{
  if (!glfwInit()) {
    throw std::runtime_error("Failed to initialize GLFW.");
  }

  window_ = glfwCreateWindow(1200, 900, "MLIVR MuJoCo Sim", NULL, NULL);
  if (!window_) {
    glfwTerminate();
    throw std::runtime_error("Failed to create GLFW window.");
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);  // VSync (60fps制限など)

  glfwSetWindowUserPointer(window_, this);
  glfwSetScrollCallback(window_, scrollCallback);
  glfwSetMouseButtonCallback(window_, mouseButtonCallback);
  glfwSetCursorPosCallback(window_, mouseMoveCallback);

  mjv_defaultScene(&scn_);
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjr_defaultContext(&con_);

  // Visualization options
  opt_.frame = mjFRAME_SITE;
  opt_.flags[mjVIS_CONTACTFORCE] = 1;
  opt_.flags[mjVIS_CONTACTPOINT] = 1;

  mjModel * m = engine_->getModel();
  mjv_makeScene(m, &scn_, 2000);
  mjr_makeContext(m, &con_, mjFONTSCALE_150);

  // Camera init config
  cam_.azimuth = 0.0;      // deg
  cam_.elevation = -10.0;  // deg
  cam_.distance = 3.0;
  cam_.lookat[0] = 0.5;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 1.0;

  std::cout << "[MujocoRenderer] Successfully initialized." << std::endl;
}

MujocoRenderer::~MujocoRenderer()
{
  mjr_freeContext(&con_);
  mjv_freeScene(&scn_);
  if (window_) {
    glfwDestroyWindow(window_);
  }
  glfwTerminate();
}

bool MujocoRenderer::isWindowClosed() const
{
  return glfwWindowShouldClose(window_);
}

void MujocoRenderer::render()
{
  if (!window_ || isWindowClosed()) return;

  mjModel * m = engine_->getModel();
  mjData * d = engine_->getData();

  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

  // シーンの更新
  mjv_updateScene(m, d, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);

  // ==========================================================
  // もし将来、C++側で直接軌道の線(カプセル)を描画したい場合は、
  // ここに mjv_connector 等の処理を記述します。
  // ==========================================================

  // レンダリングとバッファのスワップ
  mjr_render(viewport, &scn_, &con_);
  glfwSwapBuffers(window_);
  glfwPollEvents();
}

void MujocoRenderer::scrollCallback(GLFWwindow * window, double xoffset, double yoffset)
{
  (void)xoffset;
  auto * renderer = static_cast<MujocoRenderer *>(glfwGetWindowUserPointer(window));

  if (!renderer || !renderer->engine_) {
    return;
  }

  mjv_moveCamera(
    renderer->engine_->getModel(), mjMOUSE_ZOOM, 0, -0.05 * yoffset, &renderer->scn_,
    &renderer->cam_);
}

void MujocoRenderer::mouseButtonCallback(GLFWwindow * window, int button, int act, int mods)
{
  (void)button;
  (void)act;
  (void)mods;
  auto * renderer = static_cast<MujocoRenderer *>(glfwGetWindowUserPointer(window));
  if (!renderer) {
    return;
  }

  // Update the button's state
  renderer->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  renderer->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  renderer->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // Save the mouse coordinates at the moment of the click
  glfwGetCursorPos(window, &renderer->lastx_, &renderer->lasty_);
}

void MujocoRenderer::mouseMoveCallback(GLFWwindow * window, double xpos, double ypos)
{
  auto * renderer = static_cast<MujocoRenderer *>(glfwGetWindowUserPointer(window));
  if (!renderer || !renderer->engine_) {
    return;
  }

  if (!renderer->button_left_ && !renderer->button_middle_ && !renderer->button_right_) {
    return;
  }

  // Mouse movement distance
  double dx = xpos - renderer->lastx_;
  double dy = ypos - renderer->lasty_;
  renderer->lastx_ = xpos;
  renderer->lasty_ = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift =
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // Determine the camera operation (action) based on the button
  mjtMouse action;
  if (renderer->button_right_) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (renderer->button_left_) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(
    renderer->engine_->getModel(), action, dx / height, dy / height, &renderer->scn_,
    &renderer->cam_);
}

}  // namespace mlivr_sim
