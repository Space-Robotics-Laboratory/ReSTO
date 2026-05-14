# ReSTO: Reaction-Suppression Trajectory Optimization

<img src="./docs/videos/resto_sim_case2.gif" alt="resto_sim_case2.gif" width="400">

## Requirements

TBD.

## Installation

TBD

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

    - Generalized Jacobian Matrix (GJM)

      ```bash
      ros2 launch mlivr_sim gjm_sim.launch.py
      ```

    - Reaction-Aware Motion Planning (RAMP)

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
For detailed credits and licenses regarding the 3D meshes and URDF models, please see the [mlivr_description README](./src/mlivr_description/README.md).
