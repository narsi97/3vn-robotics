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
Launch the 3VN Arm in Gazebo Harmonic.

  make sim              headless server (fast, what CI runs)
  make sim GUI=1        adds the Gazebo GUI via noVNC (slow: llvmpipe)

Gazebo hosts controller_manager inside its own process through the
gz_ros2_control system plugin, which is why this cannot simply reuse
threevn_bringup/robot.launch.py. Everything below that -- the controllers,
the topics, the actions -- is identical to the mock and esp32 targets.
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
    profile = LaunchConfiguration('profile')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    params_file = PathJoinSubstitution(
        [FindPackageShare(DESCRIPTION_PKG), 'config', [profile, '.yaml']]
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(DESCRIPTION_PKG), 'urdf', 'threevn_arm.urdf.xacro']
    )
    world_file = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'worlds', [world, '.sdf']]
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'config', 'gz_bridge.yaml']
    )

    # target:=gz selects gz_ros2_control/GazeboSimSystem and emits the
    # <gazebo> plugin block. Identical description otherwise.
    # The raw substitution, shared by robot_state_publisher AND the
    # spawner below so the two cannot possibly disagree.
    robot_description_content = Command(
        ['xacro ', xacro_file, ' params_file:=', params_file, ' target:=gz']
    )

    # A Command substitution yields URDF XML. Without value_type=str,
    # launch tries to parse it as YAML and fails at startup.
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # On llvmpipe the GUI runs at 5-15 FPS; that is CPU rendering, not a fault.
    # -r starts unpaused; -s is server-only (headless).
    #
    # NOTE: PythonExpression evaluates the joined string as Python, and a
    # LaunchConfiguration substitutes as the literal text 'true'/'false',
    # which are not Python booleans. Comparing as strings is therefore
    # required -- `if ', gui, '` would raise NameError at launch.
    gz_args = PythonExpression([
        "'-r -v1 ' if '", gui, "' == 'true' else '-s -r -v1 '",
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ]),
        launch_arguments={'gz_args': [gz_args, world_file]}.items(),
    )

    # Spawn from the SAME substitution robot_state_publisher uses, not
    # from the /robot_description topic.
    #
    # Spawning from the topic looks tidier and is a real trap: a stale
    # robot_state_publisher left over from `make mock` also publishes
    # /robot_description, and the spawner will happily take that one. The
    # symptom is a pluginlib error 30 seconds later complaining that
    # mock_components/GenericSystem is not a Gazebo plugin - which says
    # nothing about the actual cause.
    #
    # Passing the content directly makes the two byte-identical by
    # construction rather than by coincidence.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description_content,
                   '-name', 'threevn_arm', '-z', '0.0'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
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
        DeclareLaunchArgument('profile', default_value='threevn_arm_v1'),
        DeclareLaunchArgument('world', default_value='empty_bench'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Show the Gazebo GUI (slow on software rendering)',
        ),
        gz_sim,
        robot_state_publisher,
        bridge,
        spawn,
        # Controllers can only be spawned once the model exists in the
        # simulator, because gz_ros2_control creates controller_manager
        # when the model is loaded.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[spawner('arm_controller'), spawner('gripper_controller')],
            )
        ),
    ])
