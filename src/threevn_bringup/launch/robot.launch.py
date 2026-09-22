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
Bring up the 3VN Arm against a real execution target.

  make mock      target:=mock    no simulator, no hardware
  make robot     target:=esp32   physical arm (Phase 5)

For Gazebo use `make sim`, which launches threevn_sim -- the simulator
hosts controller_manager in its own process, so it needs a different
top-level launch file. Everything BELOW that difference (controllers,
topics, actions) is identical, which is the whole point.

This package must never import or depend on anything Gazebo-specific.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DESCRIPTION_PKG = 'threevn_robot_description'
BRINGUP_PKG = 'threevn_bringup'


def generate_launch_description():
    profile = LaunchConfiguration('profile')
    target = LaunchConfiguration('target')

    params_file = PathJoinSubstitution(
        [FindPackageShare(DESCRIPTION_PKG), 'config', [profile, '.yaml']]
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(DESCRIPTION_PKG), 'urdf', 'threevn_arm.urdf.xacro']
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(BRINGUP_PKG), 'config', 'controllers.yaml']
    )

    robot_description = ParameterValue(
        Command(
            ['xacro ', xacro_file, ' params_file:=', params_file, ' target:=', target]
        ),
        # A Command substitution yields URDF XML. Without value_type=str,
        # launch tries to parse it as YAML and fails at startup.
        value_type=str,
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')

    # Controllers are chained off the broadcaster rather than all started
    # at once: starting a trajectory controller before state is flowing
    # produces a confusing "no state interface" failure.
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='threevn_arm_v1',
            description='Robot profile in threevn_robot_description/config/',
        ),
        DeclareLaunchArgument(
            'target', default_value='mock', choices=['mock', 'esp32'],
            description=(
                "Hardware target. 'gz' is intentionally absent here -- use "
                'threevn_sim for Gazebo, which hosts controller_manager itself.'
            ),
        ),
        control_node,
        robot_state_publisher,
        joint_state_broadcaster,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[spawner('arm_controller'), spawner('gripper_controller')],
            )
        ),
    ])
