# ReSTO: Reaction-Suppression Trajectory Optimization

<img src="./docs/images/resto_sim_case2.gif" alt="resto_sim_case2.gif" width="400">

## Requirements

- Ubuntu 22.04
- [ROS 2](https://docs.ros.org/en/humble/index.html) (Humble)
- [MuJoCo](https://mujoco.org/) (v3.4.0)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)
- [Crocoddyl](https://cmastalli.github.io/publications/crocoddyl20icra.html)

## Installation

```bash
# Clone repository
mkdir -p ~/resto_ws/src
cd ~/resto_ws/src
git clone https://github.com/Space-Robotics-Laboratory/ReSTO.git

# Build
cd ~/resto_ws
colcon build --symlink-install
source install/setup.bash
```

## How to Run Simulations

- Terminal #1
  - Build

    ```bash
    colcon build --symlink-install && . install/setup.bash
    ```

  - Launch simulation node
    - ReSTO (Reaction-Suppression Trajectory Optimization)

      ```bash
      ros2 launch mlivr_sim resto_sim.launch.py
      ```

    - Baseline (Standard Manipulation Jacobian)

      ```bash
      ros2 launch mlivr_sim baseline_sim.launch.py
      ```

    - GJM (Generalized Jacobian Matrix)

      ```bash
      ros2 launch mlivr_sim gjm_sim.launch.py
      ```

    - RAMP (Reaction-Aware Motion Planning)

      ```bash
      ros2 launch mlivr_sim ramp_sim.launch.py
      ```

- Terminal #2

  - Trigger simulation start

    ```bash
    . install/setup.bash
    ros2 topic pub --once /start_control std_msgs/msg/Bool "{data: true}"
    ```

## Acknowledgements

This project builds upon several excellent open-source works.
For detailed credits and licenses regarding the 3D meshes and URDF models, please see the [mlivr_description/README](./src/mlivr_description/README.md).
