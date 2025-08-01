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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, LifecycleNode

ARGUMENTS = [    
    DeclareLaunchArgument('use_sim_time', 
                          default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation (Gazebo) clock if true'),
]

def generate_launch_description():
    
    pkg_share_bringup = get_package_share_directory('slampibot_bringup')
    pkg_share_description = get_package_share_directory('slampibot_description')
    
    params_ld08 = PathJoinSubstitution([pkg_share_bringup, 'params', 'spb_ld08.yaml'])  # ld08 lidar parameters
    urdf_xacro = PathJoinSubstitution([pkg_share_description, 'urdf', 'spb_urdf.xacro'])
    
    motor_drive_cmd = Node(
        package='slampibot_bringup',
        executable='bringup_node',
        name='bringup_node',
        output='screen'
    )
       
    ld08_cmd = Node(
        package='ld08_driver',
        executable='ld08_node',
        name='ld08_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_ld08],        
    )   

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command(['xacro', ' ', urdf_xacro])},
        ],
    )
        
    camera_pub_cmd = Node(
        package='slampibot_camera',
        executable='camera_pub',
        name='camera_pub',
        output='screen'
    )
           
    ld = LaunchDescription(ARGUMENTS)    
    ld.add_action(robot_state_publisher_cmd)        
    ld.add_action(motor_drive_cmd)        
    ld.add_action(camera_pub_cmd)    
    ld.add_action(ld08_cmd)        

    return ld
