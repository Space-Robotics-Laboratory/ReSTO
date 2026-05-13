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

#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "mlivr_sim/mujoco_engine.hpp"

namespace mlivr_sim
{

class MujocoRenderer
{
public:
  explicit MujocoRenderer(MujocoEngine * engine);
  ~MujocoRenderer();

  void render();

  bool isWindowClosed() const;

private:
  // GLFW callback functions
  static void scrollCallback(GLFWwindow * window, double xoffset, double yoffset);
  static void mouseButtonCallback(GLFWwindow * window, int button, int act, int mods);
  static void mouseMoveCallback(GLFWwindow * window, double xpos, double ypos);

  MujocoEngine * engine_;
  GLFWwindow * window_ = nullptr;

  // Data for render
  mjvScene scn_;
  mjrContext con_;
  mjvCamera cam_;
  mjvOption opt_;

  // Variables for mouse contr
  bool button_left_ = false;
  bool button_middle_ = false;
  bool button_right_ = false;
  double lastx_ = 0.0;
  double lasty_ = 0.0;
};

}  // namespace mlivr_sim
