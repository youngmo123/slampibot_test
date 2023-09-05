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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [    
    DeclareLaunchArgument('use_sim_time', 
                          default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation (Gazebo) clock if true'),
]

def generate_launch_description():
    
    pkg_share_bringup = get_package_share_directory('slampibot_bringup')
    pkg_share_teleop = get_package_share_directory('slampibot_teleop')
    
    launch_teleop_joy = PathJoinSubstitution([pkg_share_teleop, 'launch', 'spb_teleop_joy.launch.py'])
    params_rviz = PathJoinSubstitution([pkg_share_bringup, 'rviz', 'spb_bringup.rviz'])    
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', params_rviz]
    )
               
    teleop_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_teleop_joy]),
    )
       
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(teleop_joy_launch)
    ld.add_action(rviz_cmd)

    return ld
