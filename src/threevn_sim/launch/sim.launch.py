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
Launch a robot in Gazebo Harmonic.

  make sim              headless server (fast, what CI runs)
  make sim GUI=1        adds the Gazebo GUI via noVNC (slow: llvmpipe)

  make sim PROFILE=so101_follower      the same, for the SO-101

Gazebo hosts controller_manager inside its own process through the
gz_ros2_control system plugin, which is why this cannot simply reuse
threevn_bringup/robot.launch.py. Everything below that -- the controllers,
the topics, the actions -- is identical to the mock and hardware targets.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from threevn_robot_description.describe import (
    controllers_file,
    describe,
    model_name,
)

BRINGUP_PKG = 'threevn_bringup'
SIM_PKG = 'threevn_sim'


def _simulate(context, *args, **kwargs):
    """Build the actions once `profile` is known."""
    profile = LaunchConfiguration('profile').perform(context)
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    world_file = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'worlds', [world, '.sdf']]
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'config', 'gz_bridge.yaml']
    )
    # Resolved HERE, not inside the description. threevn_sim depends on
    # both packages; the description depends on neither. The profile
    # names the file, because which controllers a robot runs under is a
    # fact about the robot.
    controllers = PathJoinSubstitution(
        [FindPackageShare(BRINGUP_PKG), 'config', controllers_file(profile)]
    ).perform(context)

    # target gz selects gz_ros2_control/GazeboSimSystem and emits the
    # <gazebo> plugin block. Identical description otherwise.
    #
    # ONE string, shared by robot_state_publisher AND the spawner below
    # so the two cannot possibly disagree.
    robot_description_content = describe(
        profile, target='gz', controllers=controllers)

    # The description is URDF XML. Without value_type=str, launch tries
    # to parse it as YAML and fails at startup.
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

    # Spawn from the SAME string robot_state_publisher uses, not from
    # the /robot_description topic.
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
                   '-name', model_name(profile), '-z', '0.0'],
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
            # --switch-timeout, because the default 5 s is a WALL-clock
            # budget for work done inside the controller_manager's
            # update loop -- and that loop lives in a simulator already
            # short of CPU. Under any other load the switch misses its
            # window, one spawner dies, and the robot comes up missing a
            # controller: no /joint_states, so RViz shows a robot with no
            # joint positions and every client waits forever for state
            # that is never published.
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--switch-timeout', '30'],
            output='screen',
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')

    return [
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
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='threevn_arm_v1'),
        DeclareLaunchArgument('world', default_value='empty_bench'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Show the Gazebo GUI (slow on software rendering)',
        ),
        OpaqueFunction(function=_simulate),
    ])
