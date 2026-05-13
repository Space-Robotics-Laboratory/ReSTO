# multi-limbed_intra-vehicular_robot_trajectory_optimization

## How to Run

### WBC Simulation

- Terminal #1

  ```bash
  colcon build --symlink-install && . install/setup.bash
  ros2 launch mlivr_sim wbc_sim.launch.py
  ```

- Terminal #2

  ```bash
  . install/setup.bash
  ros2 topic pub --once /start_control std_msgs/msg/Bool "{data: true}"
  ```

### GJM Simulation

- Terminal #1

  ```bash
  colcon build --symlink-install && . install/setup.bash
  ros2 launch mlivr_sim gjm_sim.launch.py
  ```

- Terminal #2

  ```bash
  . install/setup.bash
  ros2 topic pub --once /start_control std_msgs/msg/Bool "{data: true}"
  ```

### RAMP Simulation

- Terminal #1

  ```bash
  colcon build --symlink-install && . install/setup.bash
  ros2 launch mlivr_sim ramp_sim.launch.py
  ```

- Terminal #2

  ```bash
  . install/setup.bash
  ros2 topic pub --once /start_control std_msgs/msg/Bool "{data: true}"
  ```
