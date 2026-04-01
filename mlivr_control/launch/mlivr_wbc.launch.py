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
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('mlivr_control')
    wbc_params = os.path.join(pkg_path, 'config', 'wbc_params.yaml')

    mlivr_wbc_node = Node(
        package='mlivr_control',
        executable='wb_control_node',
        name='mlivr_wbc',
        output='screen',
        parameters=[wbc_params]
    )

    return LaunchDescription([
        mlivr_wbc_node
    ])
