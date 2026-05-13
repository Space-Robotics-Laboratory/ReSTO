#!/bin/bash
xacro urdf/ur5e/mlivr.xacro > urdf/mlivr.urdf
~/.mujoco/mujoco-3.4.0/bin/compile urdf/mlivr.urdf ../mlivr_sim/models/mlivr_ur5e.xml
