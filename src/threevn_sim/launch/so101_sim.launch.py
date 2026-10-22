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
Spawn somebody else's robot, and see whether our stack notices.

    make so101-fetch     # once: pulls the Apache-2.0 description
    make so101

This is a SPIKE. It exists to test one claim, the one the whole
architecture rests on: that the application does not know what it is
driving. Every previous test of that claim swapped the execution TARGET
under one robot we designed. This swaps the robot.

The arm is TheRobotStudio's SO-101: 5 DOF plus a jaw, STS3215 bus servos,
Apache-2.0. Nothing of ours describes it. We do not re-author it into a
3VN profile either -- that would be testing our xacro, not our stack.
The upstream URDF is used as it ships, and this file adds only the two
things it lacks for Gazebo:

  * a <ros2_control> block naming its joints, with the same
    GazeboSimSystem plugin our own robot uses
  * the gz_ros2_control plugin tag

Mesh paths are rewritten from relative to absolute because the upstream
URDF names them `assets/x.stl`, which resolves against the caller's
working directory rather than the file's -- fine for MuJoCo, not for a
spawner invoked from somewhere else.

What this does NOT do is give the robot a 3VN profile, named poses or
inertial conventions. Where our stack needs those, it will fail, and the
failure is the finding.
"""

import pathlib
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SIM_PKG = 'threevn_sim'
VENDOR = pathlib.Path('/ws/vendor/so101')

#: The arm's joints, in the order the controller will drive them. Read
#: off the upstream URDF rather than invented: shoulder_pan,
#: shoulder_lift, elbow_flex, wrist_flex, wrist_roll, and the jaw.
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex',
              'wrist_flex', 'wrist_roll')
GRIPPER_JOINT = 'gripper'

#: The upstream URDF has no `world` link, so base_link is a free body:
#: spawned as it ships, a 632 g arm topples the first time a joint moves.
#: Real SO-101s are bolted to a bench, so this bolts it to the world. It
#: is an addition for simulation, not a correction -- the file is right
#: for what it was written for.
ANCHOR_BLOCK = """
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
"""

CONTROL_BLOCK = """
  <ros2_control name="so101" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
{joints}  </ros2_control>
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>{controllers}</parameters>
    </plugin>
  </gazebo>
"""

JOINT_BLOCK = """    <joint name="{name}">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
"""

CONTROLLERS = """controller_manager:
  ros__parameters:
    update_rate: 50
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    gripper_controller:
      type: position_controllers/GripperActionController

arm_controller:
  ros__parameters:
    joints:
{arm_joints}
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: false

gripper_controller:
  ros__parameters:
    joint: {gripper}
    allow_stalling: true
    max_effort: 10.0
"""


def _prepare(context, *args, **kwargs):
    """Build the launch actions, transforming the vendor URDF first."""
    urdf_path = VENDOR / 'so101.urdf'
    if not urdf_path.exists():
        raise RuntimeError(
            f'{urdf_path} is missing. Run `make so101-fetch` first: the '
            f'upstream description is Apache-2.0 and deliberately NOT '
            f'vendored into this repository.')

    urdf = urdf_path.read_text()
    urdf = urdf.replace('filename="assets/',
                        f'filename="file://{VENDOR}/assets/')

    controllers = tempfile.NamedTemporaryFile(
        mode='w', suffix='_so101_controllers.yaml', delete=False)
    controllers.write(CONTROLLERS.format(
        arm_joints='\n'.join(f'      - {j}' for j in ARM_JOINTS),
        gripper=GRIPPER_JOINT))
    controllers.close()

    joints = ''.join(JOINT_BLOCK.format(name=j)
                     for j in (*ARM_JOINTS, GRIPPER_JOINT))
    block = CONTROL_BLOCK.format(joints=joints, controllers=controllers.name)
    urdf = urdf.replace('</robot>', ANCHOR_BLOCK + block + '</robot>')

    world = LaunchConfiguration('world').perform(context)
    world_file = PathJoinSubstitution(
        [FindPackageShare(SIM_PKG), 'worlds', f'{world}.sdf'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                  'launch', 'gz_sim.launch.py'])]),
        launch_arguments={'gz_args': ['-s -r -v1 ', world_file]}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-string', urdf, '-name', 'so101', '-z', '0.0'],
        output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(urdf, value_type=str),
                     'use_sim_time': True}])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution(
            [FindPackageShare(SIM_PKG), 'config', 'gz_bridge.yaml'])}],
        output='screen')

    def spawner(name):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--switch-timeout', '30'],
            output='screen')

    broadcaster = spawner('joint_state_broadcaster')
    return [
        gz_sim, bridge, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster,
                                           on_exit=[spawner('arm_controller')])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster,
                                           on_exit=[spawner('gripper_controller')])),
    ]


def generate_launch_description():
    """Launch the SO-101 in Gazebo under our controllers."""
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty_bench',
                              description='World to spawn into.'),
        OpaqueFunction(function=_prepare),
    ])
