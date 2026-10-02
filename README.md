# ReSTO: Reaction-Suppressing Trajectory Optimization

<img src="./docs/images/resto_sim_case2.gif" alt="resto_sim_case2.gif" width="400">

A pure C++ library that plans the swing motion of a free-floating multi-limbed robot while
suppressing the reaction on its base. The swing is solved as a whole-body optimal control
problem with [Crocoddyl](https://github.com/loco-3d/crocoddyl) (FDDP): the support limbs hold
their grasps, the swing end effector reaches its target, and the system momentum is kept small.

The standalone MuJoCo simulator used in the original study is kept on the `develop` branch.

## Requirements

- [ROS 2](https://docs.ros.org/en/humble/index.html) (Humble)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) (v3 or later)
- [Crocoddyl](https://github.com/loco-3d/crocoddyl) (v3 or later)

## Installation

```bash
mkdir -p ~/resto_ws/src
cd ~/resto_ws/src
git clone https://github.com/MasazumiImai/ReSTO.git

cd ~/resto_ws
colcon build --packages-select resto
source install/setup.bash
```

## Usage

```cpp
#include <resto/swing_optimizer.hpp>

auto model = std::make_shared<pinocchio::Model>(/* free-flyer model, zero gravity */);
resto::SwingOptimizer optimizer(model, resto::SolverParams(), resto::WeightParams());

resto::SwingProblem problem;
problem.x0 = x0;                                  // [q; v]
problem.support_frames = {"limb_1_palm_link"};
problem.support_targets = {support_placement};    // world placements to hold
problem.swing_frame = "limb_2_palm_link";
problem.swing_target = swing_target;              // world placement at the end
problem.duration = 10.0;                          // [s]

const resto::Result result = optimizer.solve(problem);  // result.xs, result.us
```

Joint position, velocity and effort limits are taken from the model; narrow them there to add
margins or avoid self-collision.

## Acknowledgements

For the credits and licenses of the meshes and URDF models used in the original simulator, see
the `develop` branch.
