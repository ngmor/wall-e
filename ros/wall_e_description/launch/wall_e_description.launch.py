# Copyright 2024 Nick Morales.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, Shutdown
from launch.substitutions import Command, TextSubstitution, \
    PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    """Launch robot description and optionally RVIZ."""
    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz_enable',
            default_value='true',
            choices=['true', 'false'],
            description='Selects whether or not to launch RVIZ.',
        ),
        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('wall_e_description'),
                'config',
                'wall_e.rviz'
            ]),
            description='Selects config file to use for RVIZ'
        ),
        DeclareLaunchArgument(
            name='jsp_type',
            default_value='gui',
            choices=['gui', 'normal', 'none'],
            description='Options for starting joint state publisher.',
        ),
        SetLaunchConfiguration(
            name='namespace',
            value='wall_e',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'robot_description':
                    ParameterValue(
                        Command([
                            TextSubstitution(text='xacro '),
                            PathJoinSubstitution([
                                FindPackageShare('wall_e_description'),
                                'urdf',
                                'wall_e.urdf.xacro'
                            ]),
                        ]),
                        value_type=str
                    ),
                'frame_prefix': 
                    [
                        LaunchConfiguration('namespace'),
                        '/'
                    ],
            }],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=LaunchConfiguration('namespace'),
            condition=LaunchConfigurationEquals('jsp_type', 'gui'),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            condition=LaunchConfigurationEquals('jsp_type', 'normal'),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration('rviz_enable')),
            arguments=[
                '-d',
                LaunchConfiguration('rviz_config_file'),
            ],
            on_exit=Shutdown()
        ),
    ])
