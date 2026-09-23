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
Launch the 3VN mobile base in Gazebo.

    make base-sim

Same structure as sim.launch.py for the arm: Gazebo hosts
controller_manager through gz_ros2_control, so the spawners chain off
the spawn event rather than starting immediately.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DESCRIPTION_PKG = 'threevn_robot_description'
BRINGUP_PKG = 'threevn_bringup'
SIM_PKG = 'threevn_sim'


def generate_launch_description():
    """Build the launch description."""
    profile = LaunchConfiguration('profile')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    params_file = PathJoinSubstitution(
        [FindPackageShare(DESCRIPTION_PKG), 'config', [profile, '.yaml']])
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(DESCRIPTION_PKG), 'urdf', 'threevn_base.urdf.xacro'])
    world_file = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'worlds', [world, '.sdf']])
    bridge_config = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'config', 'gz_bridge.yaml'])
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(BRINGUP_PKG), 'config', 'base_controllers.yaml'])

    robot_description_content = Command(
        ['xacro ', xacro_file, ' params_file:=', params_file,
         ' target:=gz', ' controllers_file:=', controllers_file])

    gz_args = PythonExpression([
        "'-r -v1 ' if '", gui, "' == 'true' else '-s -r -v1 '"])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': [gz_args, world_file]}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description_content,
                   # z=0: the description's own ride height puts the
                   # wheels on the ground. This was 0.05, which hid a
                   # chassis buried 20 mm into the floor -- the
                   # simulation looked fine and the geometry was wrong.
                   '-name', 'threevn_base', '-z', '0.0'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description_content, value_type=str),
            'use_sim_time': True,
        }],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='threevn_base_v1'),
        DeclareLaunchArgument('world', default_value='empty_bench'),
        DeclareLaunchArgument('gui', default_value='false'),
        gz_sim,
        robot_state_publisher,
        bridge,
        spawn,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn,
                          on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster,
                          on_exit=[spawner('base_controller')])),
    ])
