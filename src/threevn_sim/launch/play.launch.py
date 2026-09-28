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
The whole robot, headless, with something to watch.

    make play          # this
    make drive         # teleop, in a second terminal

This is `mm_sim.launch.py` plus RViz. It exists because the two halves
were previously only ever run apart: the simulator had no viewer that
understood a robot which drives, and the only RViz config was fixed to
`base_footprint`, which pins the robot to the centre of the screen and
makes the world slide underneath it. Watching odometry -- the single most
instructive thing a mobile base does -- was impossible.

Gazebo stays HEADLESS on purpose and `gui:=true` is not forwarded: its
GUI buys nothing RViz does not already show, and it is another software
rasteriser competing for the same CPU. If you specifically want Gazebo's
own render, run `make mm-sim GUI=1`.

Two viewers are started, and either can be switched off:

  rviz:=false     Stop drawing on the container's virtual display. Use
                  it when you are watching through Foxglove instead, so
                  nothing is rasterised for a display no one is looking
                  at. It probably frees CPU for the physics as well, but
                  the real-time factor here varies by +/-20% run to run,
                  which is too noisy to demonstrate that -- so this is
                  offered as a reasonable default, not a measured win.
  bridge:=false   Drop the Foxglove WebSocket.

The bridge is the way out of CPU rendering altogether. RViz-over-VNC
ships an encoded framebuffer for every frame; the bridge ships messages
and lets a native viewer on the host draw them on the host's GPU.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#: Foxglove's default. Published to 127.0.0.1:8765 by compose.
BRIDGE_PORT = 8765

DESCRIPTION_PKG = 'threevn_robot_description'
SIM_PKG = 'threevn_sim'


def generate_launch_description():
    """Launch the mobile manipulator headless, with RViz attached."""
    profile = LaunchConfiguration('profile')
    world = LaunchConfiguration('world')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare(SIM_PKG), 'launch', 'mm_sim.launch.py']),
        ]),
        launch_arguments={
            'profile': profile,
            'world': world,
            'gui': 'false',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare(DESCRIPTION_PKG), 'rviz', 'view_mm.rviz']),
        ],
        # Without this RViz timestamps its TF lookups from the wall clock
        # while everything else runs on /clock at 0.43x. Every transform
        # then looks 'too old for the buffer' and the robot never appears
        # -- a blank screen with no error, which is the worst kind.
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': BRIDGE_PORT,
            # 0.0.0.0 INSIDE the container, deliberately. The house rule
            # is that nothing binds a routable address on the HOST, and
            # compose honours it by publishing this to 127.0.0.1 only. A
            # server bound to the container's loopback would not be
            # reachable through that mapping at all.
            'address': '0.0.0.0',
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('bridge')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='threevn_mm_v1',
            description='Robot profile to spawn.'),
        DeclareLaunchArgument(
            'world', default_value='bench_with_target',
            description='World to spawn into. bench_with_target carries '
                        'the green cube the perception phase looks for.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Run RViz on the container display. false when '
                        'you are viewing through Foxglove instead.'),
        DeclareLaunchArgument(
            'bridge', default_value='true',
            description='Run foxglove_bridge on port 8765.'),
        sim,
        rviz,
        bridge,
    ])
