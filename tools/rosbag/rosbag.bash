#!/bin/bash
ros2 bag record\
  /joint_states\
  /joint_cmds\
  /tf\
  /mlivr_sim/limb_1/ee_ft_sensor\
  /mlivr_sim/limb_2/ee_ft_sensor\
