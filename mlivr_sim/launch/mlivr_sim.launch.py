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
    pkg_path = get_package_share_directory('mlivr_sim')
    sim_params = os.path.join(pkg_path, 'config', 'sim_params.yaml')

    mlivr_sim_node = Node(
        package='mlivr_sim',
        executable='mlivr_sim_node_exec',
        name='mlivr_sim',
        output='screen',
        parameters=[sim_params]
    )

    return LaunchDescription([
        mlivr_sim_node,
    ])
