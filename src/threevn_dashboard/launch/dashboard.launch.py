# Copyright 2026 3VN Systems
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

"""
Launch the dashboard alongside a running robot.

    ros2 launch threevn_dashboard dashboard.launch.py
    ros2 launch threevn_dashboard dashboard.launch.py port:=9000

Starts nothing else: the dashboard observes whatever robot is already up,
whether that is mock, Gazebo or hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='8107',
            description='HTTP port for the dashboard'),
        DeclareLaunchArgument(
            'host', default_value='0.0.0.0',
            description='Bind address. 0.0.0.0 inside a container; bind '
                        '127.0.0.1 on a host reachable from a network.'),
        Node(
            package='threevn_dashboard',
            executable='dashboard',
            name='threevn_dashboard',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'host': LaunchConfiguration('host'),
            }],
        ),
    ])
