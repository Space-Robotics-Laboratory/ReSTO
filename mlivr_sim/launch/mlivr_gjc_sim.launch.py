# Copyright (c) 2026 Masazumi Imai
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg_dir = get_package_share_directory('mlivr_description')
    sim_pkg_dir = get_package_share_directory('mlivr_sim')

    urdf_file = os.path.join(desc_pkg_dir, 'urdf', 'mlivr.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    robot_description = {'robot_description': robot_desc}

    mlivr_params = os.path.join(desc_pkg_dir, 'config', 'mlivr_params.yaml')
    sim_params = os.path.join(sim_pkg_dir, 'config', 'sim_params.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_config_file = os.path.join(sim_pkg_dir,
                                    'config', 'mlivr_gjc_sim.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    mlivr_sim_node = Node(
        package='mlivr_sim',
        executable='mlivr_sim_node_exec',
        name='mlivr_sim',
        output='screen',
        parameters=[sim_params]
    )

    mlivr_gjc_node = Node(
        package='mlivr_control',
        executable='gj_control_node',
        name='mlivr_gjc',
        output='screen',
        parameters=[mlivr_params]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        mlivr_sim_node,
        mlivr_gjc_node
    ])
