# mlivr_description

This package contains the URDF, meshes, and kinematics descriptions for the MLIVR (Multi-Limbed Intra-Vehicular Robot).

## How to Generate urdf and xml (mjcf) from xacro

Run the following command under `mlivr_description`

```bash
xacro urdf/ur5e/mlivr.xacro > urdf/mlivr.urdf
~/.mujoco/mujoco-3.4.0/bin/compile urdf/mlivr.urdf ../mlivr_sim/models/mlivr_ur5e.xml
```

## Acknowledgements / Third-Party Credits

### UR5e kinematics and meshes

The base UR5e kinematics and meshes used in this package are adapted from the following open-source repository:

- Source: [pybullet_ur5_gripper](https://github.com/culurciello/pybullet_ur5_gripper)
- Modifications: The original URDF was modified for the MLIVR project.

### ISS KIBO meshes

The ISS KIBO (JEM) meshes used in this package are refer from the following open-source repository:

- Source: [int-ball2_isaac_sim](https://github.com/sd-robotics/int-ball2_isaac_sim) by SpaceData.
- License: Apache-2.0
- Modifications: The original USD file was converted to COLLADA format (`.dae`).
