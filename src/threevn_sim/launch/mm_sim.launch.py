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
Launch the 3VN mobile manipulator in Gazebo.

    make mm-sim

The controller parameters are MERGED AT LAUNCH, not committed as a third
file. The arm's controllers.yaml and the base's base_controllers.yaml
stay the single source of truth for their own subsystem; this reads both
and applies the composition's prefix to the arm's joint names, because
in the composed robot those joints are called arm_shoulder_pan_joint and
so on.

A committed mm_controllers.yaml would have been the obvious move and the
wrong one: the same joint list would exist in two files, and adding a
joint to the arm would leave the composed robot silently driving the old
set. This is the same reasoning that keeps the URDF expanding at launch
instead of being generated and committed.
"""

import copy
import pathlib
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

DESCRIPTION_PKG = 'threevn_robot_description'
BRINGUP_PKG = 'threevn_bringup'
SIM_PKG = 'threevn_sim'

#: Parameters naming a joint, which therefore need the composition's
#: prefix applied. Listed explicitly: a blanket search-and-replace over
#: the whole document would also rewrite frame ids and controller names,
#: which are not prefixed.
JOINT_KEYS = ('joints', 'joint', 'left_wheel_names', 'right_wheel_names')


def prefix_joint_names(params, prefix):
    """
    Return `params` with every joint name carrying `prefix`.

    Only joint NAMES move. Controller names stay as they are, so the
    topics and actions a client already knows -- /arm_controller/... and
    /base_controller/... -- are the same on the composed robot as on
    either component alone. That is the point of composing rather than
    rewriting: the interface does not change.
    """
    out = copy.deepcopy(params)
    for section in out.values():
        if not isinstance(section, dict):
            continue
        ros_params = section.get('ros__parameters')
        if not isinstance(ros_params, dict):
            continue
        for key in JOINT_KEYS:
            value = ros_params.get(key)
            if isinstance(value, str):
                ros_params[key] = prefix + value
            elif isinstance(value, list):
                ros_params[key] = [prefix + name for name in value]
    return out


def merge_controller_params(arm_params, base_params, prefix):
    """
    Merge the two subsystems' controller parameters into one document.

    `controller_manager` is the only section both declare. Its controller
    entries are unioned so one manager hosts every controller; the scalar
    settings must AGREE rather than one silently winning, because
    update_rate differing between the two would mean the composed robot
    ran its arm at a rate its own config never states.
    """
    merged = prefix_joint_names(arm_params, prefix)

    for name, section in base_params.items():
        if name not in merged:
            merged[name] = copy.deepcopy(section)
            continue
        if name != 'controller_manager':
            raise ValueError(
                f'{name!r} is configured by both subsystems; only '
                f'controller_manager may be')
        target = merged[name]['ros__parameters']
        for key, value in section['ros__parameters'].items():
            if key in target and target[key] != value:
                raise ValueError(
                    f'controller_manager.{key} differs between the arm '
                    f'({target[key]!r}) and the base ({value!r}); the '
                    f'composed robot cannot run both')
            target[key] = value
    return merged


def _write_merged_controllers(prefix):
    """Merge both configs and write them where Gazebo can read them."""
    bringup = pathlib.Path(get_package_share_directory(BRINGUP_PKG)) / 'config'
    arm = yaml.safe_load((bringup / 'controllers.yaml').read_text())
    base = yaml.safe_load((bringup / 'base_controllers.yaml').read_text())

    merged = merge_controller_params(arm, base, prefix)

    handle = tempfile.NamedTemporaryFile(
        mode='w', suffix='_mm_controllers.yaml', delete=False)
    with handle as out:
        yaml.safe_dump(merged, out, default_flow_style=False)
    return handle.name


def _setup(context, *args, **kwargs):
    """
    Build the launch actions once the profile is known.

    An OpaqueFunction rather than plain module code: `profile` is a
    launch argument, so it does not have a value until the launch runs,
    and the composition YAML has to be read to learn the arm's prefix.
    """
    profile = LaunchConfiguration('profile').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    world = LaunchConfiguration('world').perform(context)

    description = pathlib.Path(get_package_share_directory(DESCRIPTION_PKG))
    params_file = description / 'config' / f'{profile}.yaml'
    composition = yaml.safe_load(params_file.read_text())
    prefix = composition['mount']['prefix']

    controllers_file = _write_merged_controllers(prefix)

    xacro_file = description / 'urdf' / 'threevn_mobile_manipulator.urdf.xacro'
    robot_description_content = Command(
        ['xacro ', str(xacro_file), ' params_file:=', str(params_file),
         ' target:=gz', ' controllers_file:=', controllers_file])

    world_file = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'worlds', [world, '.sdf']])
    bridge_config = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'config', 'gz_bridge.yaml'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r -v1 ' if gui == 'true' else '-s -r -v1 ',
                        world_file]
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        # z=0 for the same reason as the base: the description's own ride
        # height puts the wheels on the ground, and a spawn offset would
        # hide it if that were ever wrong again.
        arguments=['-string', robot_description_content,
                   '-name', 'threevn_mm', '-z', '0.0'],
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

    return [
        gz_sim,
        robot_state_publisher,
        bridge,
        spawn,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn,
                          on_exit=[joint_state_broadcaster])),
        # Both subsystems, one after the other. Starting them together
        # races on the same controller_manager service.
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster,
                          on_exit=[spawner('base_controller')])),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster,
                          on_exit=[spawner('arm_controller')])),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster,
                          on_exit=[spawner('gripper_controller')])),
    ]


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='threevn_mm_v1'),
        DeclareLaunchArgument('world', default_value='empty_bench'),
        DeclareLaunchArgument('gui', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
