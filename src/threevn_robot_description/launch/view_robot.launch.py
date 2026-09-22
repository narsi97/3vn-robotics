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
Show the robot in RViz with interactive joint sliders.

No simulator, no hardware, no controllers -- just the description,
robot_state_publisher and RViz. This is the fastest way to see whether a
change to the YAML or the xacro did what you meant.

  make view                       # default profile
  make view PROFILE=threevn_arm_v1_long_reach
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'threevn_robot_description'


def generate_launch_description():
    profile = LaunchConfiguration('profile')

    params_file = PathJoinSubstitution(
        [FindPackageShare(PKG), 'config', [profile, '.yaml']]
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(PKG), 'urdf', 'threevn_arm.urdf.xacro']
    )

    # robot_state_publisher is the ONLY source of TF in this repo. No
    # hand-written static_transform_publisher, ever -- it publishes
    # /tf_static for fixed joints and /tf from /joint_states.
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' params_file:=', params_file,
            # This launch file never runs controllers, so the hardware
            # target is irrelevant here; mock keeps it simulator-free.
            ' target:=mock',
        ]),
        # A Command substitution yields URDF XML. Without value_type=str,
        # launch tries to parse it as YAML and fails at startup.
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='threevn_arm_v1',
            description='Robot profile in config/ (without the .yaml suffix)',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=[
                '-d',
                os.path.join(get_package_share_directory(PKG), 'rviz', 'view_arm.rviz'),
            ],
        ),
    ])
