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
  make view PROFILE=so101_follower
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from threevn_robot_description.describe import describe, parse_semantics

PKG = 'threevn_robot_description'


def _view(context, *args, **kwargs):
    """Build the actions once `profile` is known."""
    profile = LaunchConfiguration('profile').perform(context)

    # This launch file never runs controllers, so the hardware target is
    # irrelevant here; mock keeps it simulator-free.
    urdf = describe(profile, target='mock')

    # robot_state_publisher is the ONLY source of TF in this repo. No
    # hand-written static_transform_publisher, ever -- it publishes
    # /tf_static for fixed joints and /tf from /joint_states.
    robot_description = ParameterValue(
        urdf,
        # The description is URDF XML. Without value_type=str, launch
        # tries to parse it as YAML and fails at startup.
        value_type=str,
    )

    rviz_args = [
        '-d',
        os.path.join(get_package_share_directory(PKG), 'rviz', 'view_arm.rviz'),
    ]
    # The saved view fixes on base_footprint, a frame our own arm has
    # and somebody else's robot need not. The description says which
    # frame is the robot's base, so a foreign arm is fixed on that.
    semantics = parse_semantics(urdf)
    if semantics and 'base_footprint' not in urdf:
        rviz_args += ['-f', semantics['base_frame']]

    return [
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
            arguments=rviz_args,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='threevn_arm_v1',
            description='Robot profile in config/ (without the .yaml suffix)',
        ),
        OpaqueFunction(function=_view),
    ])
