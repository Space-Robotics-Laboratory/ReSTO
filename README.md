# multi-limbed_intra-vehicular_robot_trajectory_optimization

## How to Run

### WBC Simulation

- Terminal #1

  ```bash
  colcon build --symlink-install && . install/setup.bash
  ros2 launch mlivr_sim mlivr_wbc_sim.launch.py
  ```

- Terminal #2

  ```bash
  . install/setup.bash
  ros2 topic pub --once /start_control std_msgs/msg/Bool "{data: true}"
  ```

### GJ Simulation

- Terminal #1

  ```bash
  colcon build --symlink-install && . install/setup.bash
  ros2 launch mlivr_sim mlivr_gjc_sim.launch.py
  ```

- Terminal #2

  ```bash
  . install/setup.bash
  ros2 topic pub --once /start_control std_msgs/msg/Bool "{data: true}"
  ```
