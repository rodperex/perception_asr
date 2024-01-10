# Copyright 2023 Rodrigo Pérez-Rodríguez
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
import yaml

def generate_launch_description():
    
    ld = LaunchDescription()

    robot_cmd = Node(
        package='perception_asr',
        executable='detection_2d_to_3d_depth',
        output='screen',
        remappings=[
            ('input_detection_2d', 'detection_result'),
            ('input_depth', '/head_front_camera/depth_registered/image_raw'),
            ('camera_info', '/head_front_camera/depth_registered/camera_info')
        ],
        parameters=[{
            'use_sim_time': True,
        }]
    )

    ld.add_action(robot_cmd)

    return ld

