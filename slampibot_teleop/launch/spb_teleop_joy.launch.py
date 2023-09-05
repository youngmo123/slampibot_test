# !/usr/bin/env/ python3
#
# Copyright 2023 EduRobotAILab CO., LTD.
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
#
# Author: Leo Cho

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share_teleop = get_package_share_directory('slampibot_teleop')
    
    params_joy = PathJoinSubstitution([pkg_share_teleop, 'params', 'slampibot_teleop_joy.yaml'])

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',            
    )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[params_joy],            
    )

    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    
    return ld
    