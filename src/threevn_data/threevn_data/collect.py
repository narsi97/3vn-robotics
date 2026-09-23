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
Record a labelled dataset from the running robot.

    make dataset                    # a few short episodes
    make dataset EPISODES=8

Each episode drives the base to a different place, waits for the scene to
settle, and records frames. The label is the target's position in the
camera's optical frame, taken from simulator ground truth: exact, free,
and available for every frame.

THAT IS THE POINT OF RECORDING IN SIMULATION. The same dataset gathered
on a real robot needs either a motion-capture rig or a human drawing
boxes, and a human is slower, more expensive and less accurate than the
number Gazebo already knows. The cost is that everything learned from it
is learned about a simulator, which is why `labels.source` says
`simulator` in every manifest rather than leaving it to be assumed.

Why not rosbag2? Because a bag is a replayable LOG and this is a derived
DATASET, and the two want different things. Measured on this camera, a
bag stores 922 kB per frame - which is exactly 640x480x3, raw and
uncompressed - against 11 kB for the PNG written here. That is 87x, and
the bag still has to be extracted before anything can train on it, with
the labels attached during the extraction anyway.

Recording bags is the right move when the question is "what happened".
This answers "what should the model see".

The 11 kB deserves a caveat: PNG compresses a synthetic scene of flat
colours extremely well, and a real camera's output would be an order of
magnitude larger. The 87x is the honest number for THIS setup, not a
general claim.
"""

import argparse
import collections
import json
import math
import pathlib
import random
import subprocess
import sys
import time

import cv2
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped with tf2)
import tf2_ros

from threevn_data import manifest as manifest_mod

#: Robot poses arrive on dynamic_pose/info; a STATIC model like the
#: target cube does not, because it is not part of the dynamic set. It
#: appears on pose/info instead. Reading the wrong one returns nothing
#: and looks exactly like a simulator that is not running.
DYNAMIC_TOPIC = '/world/{world}/dynamic_pose/info'
STATIC_TOPIC = '/world/{world}/pose/info'
SET_POSE = '/world/{world}/set_pose'
ROBOT_MODEL = 'threevn_mm'
TARGET_MODEL = 'target_cube'


def _read_pose(topic, model, timeout=15):
    """Return (x, y, z, yaw) for a model from a gz pose topic, or None."""
    try:
        out = subprocess.run(['gz', 'topic', '-t', topic, '-e', '-n', '1'],
                             capture_output=True, text=True,
                             timeout=timeout).stdout
    except (subprocess.SubprocessError, OSError):
        return None

    block, seen = [], False
    for line in out.splitlines():
        if 'name:' in line:
            if seen:
                break
            seen = f'"{model}"' in line
            continue
        if seen:
            block.append(line)
    if not seen:
        return None

    def field(section, key):
        inside = False
        for line in block:
            stripped = line.strip()
            if stripped.startswith(section):
                inside = True
                continue
            if inside:
                if stripped.startswith('}'):
                    inside = False
                    continue
                if stripped.startswith(key + ':'):
                    return float(stripped.split(':', 1)[1])
        return 0.0

    qz = field('orientation', 'z')
    qw = field('orientation', 'w') or 1.0
    return (field('position', 'x'), field('position', 'y'),
            field('position', 'z'),
            math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz))


def robot_pose(world, timeout=15):
    """Return the robot's true (x, y, z, yaw), or None."""
    return _read_pose(DYNAMIC_TOPIC.format(world=world), ROBOT_MODEL, timeout)


def target_pose(world, timeout=15):
    """
    Return the target's true (x, y, z, yaw), or None.

    Read rather than assumed. The first version of this carried the
    cube's pose as a module constant copied from the world file, which
    was correct exactly until the recorder started MOVING the cube to
    vary the labels - at which point every label after the first would
    have been silently wrong while the images changed as expected.
    """
    return _read_pose(STATIC_TOPIC.format(world=world), TARGET_MODEL, timeout)


def place_target(world, x, y, z=0.025, timeout=10):
    """
    Move the target. Returns True if the simulator accepted it.

    Gazebo will move a <static> model on request even though physics
    will not; that is what makes it usable as a repositionable prop.
    """
    request = (f'name: "{TARGET_MODEL}", '
               f'position: {{x: {x}, y: {y}, z: {z}}}')
    try:
        out = subprocess.run(
            ['gz', 'service', '-s', SET_POSE.format(world=world),
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', request],
            capture_output=True, text=True, timeout=timeout).stdout
    except (subprocess.SubprocessError, OSError):
        return False
    return 'true' in out.lower()


class Collector(Node):
    """Drive, look, and write down what was seen and where the target was."""

    def __init__(self, out_dir, world):
        super().__init__('collect')
        self.out_dir = pathlib.Path(out_dir)
        (self.out_dir / 'images').mkdir(parents=True, exist_ok=True)
        self.world = world

        self.image = None
        self.info = None
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._on_info,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.cmd = self.create_publisher(
            TwistStamped, '/base_controller/cmd_vel', 10)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.rows = []
        # Why frames were dropped. A recorder that silently keeps
        # 8 of 96 frames looks exactly like one that worked.
        self.rejected = collections.Counter()
        self._reject_reason = 'unknown'

    def _on_image(self, msg):
        self.image = msg

    def _on_info(self, msg):
        self.info = msg

    def spin_for(self, seconds):
        """Spin the executor for a wall-clock duration."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, timeout=60.0):
        """Block until the camera is producing frames and intrinsics."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image is not None and self.info is not None:
                return True
        return False

    def drive(self, linear, angular, seconds, settle=1.0):
        """Hold a velocity command, then stop and let the scene settle."""
        msg = TwistStamped()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        end = time.time() + seconds
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        stop = TwistStamped()
        for _ in range(10):
            stop.header.stamp = self.get_clock().now().to_msg()
            self.cmd.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.spin_for(settle)

    def record_episode(self, episode, linear, angular, frames, settle=0.6):
        """
        Step and shoot: a small move, a pause, a frame. Repeat.

        Two earlier attempts were wrong in opposite ways.

        Driving to one pose and taking every frame there produced
        byte-identical images: with a stationary robot and a noiseless
        camera, twelve frames were one picture, and the dataset claimed
        96 samples while holding 8.

        Capturing continuously while driving fixed that and broke
        something worse. The label comes from the simulator's ground
        truth, read by asking Gazebo AFTER the image arrived, so image
        and label are from different instants. At 0.08 m/s a half-second
        of skew is 40 mm of label error - larger than the 30 mm error
        the geometric baseline already achieves, which would make the
        dataset worse than useless for improving on it.

        Stopping to capture makes the two agree. Each frame is a
        genuinely distinct viewpoint, and consecutive frames are NEARLY
        duplicate, which is the real hazard the split has to survive.
        """
        self.spin_for(settle)
        kept = 0
        for index in range(frames):
            if index:
                self.drive(linear, angular, 0.35, settle=0.45)
            if self.capture(episode, index):
                kept += 1
            else:
                self.rejected[self._reject_reason] += 1
        return kept

    def label_now(self):
        """
        Return the target's position in the camera's optical frame.

        BOTH poses come from ground truth. The robot's world pose carries
        the target into base_footprint, and TF - which is exact, since
        the rest of the chain is rigid and comes from the description -
        carries it to the optical frame.

        Odometry would be the obvious source for the robot pose and is
        the wrong one: it drifts, and a label that drifts is quietly
        wrong in a way no amount of training will fix.
        """
        robot = robot_pose(self.world)
        target = target_pose(self.world)
        if robot is None or target is None:
            return None
        rx, ry, _, ryaw = robot
        tx, ty, tz, _ = target

        dx, dy = tx - rx, ty - ry
        cos, sin = math.cos(-ryaw), math.sin(-ryaw)

        point = PoseStamped()
        point.header.frame_id = 'base_footprint'
        point.pose.position.x = dx * cos - dy * sin
        point.pose.position.y = dx * sin + dy * cos
        point.pose.position.z = tz
        point.pose.orientation.w = 1.0

        try:
            out = self.buffer.transform(
                point, self.image.header.frame_id,
                timeout=rclpy.duration.Duration(seconds=0.5))
        except tf2_ros.TransformException:
            return None
        return (out.pose.position.x, out.pose.position.y, out.pose.position.z)

    def place_target_ahead(self, forward, lateral):
        """
        Put the target a given offset AHEAD OF THE ROBOT, wherever it is.

        Placing it at fixed world coordinates would work for one episode
        and then drift out of frame as the robot drove, which is how an
        earlier run dropped all 96 frames as "target behind the camera".
        Offsetting from the robot's current pose keeps every episode
        usable and, because the robot has moved, still puts the target
        somewhere new in the world.
        """
        robot = robot_pose(self.world)
        if robot is None:
            return False
        rx, ry, _, ryaw = robot
        x = rx + forward * math.cos(ryaw) - lateral * math.sin(ryaw)
        y = ry + forward * math.sin(ryaw) + lateral * math.cos(ryaw)
        if not place_target(self.world, x, y):
            return False
        # The pose takes a moment to reach the render and the pose topic.
        self.spin_for(0.6)
        return True

    def capture(self, episode, index):
        """Write one frame and its label. Returns True if it was kept."""
        if self.image is None:
            self._reject_reason = 'no image'
            return False
        label = self.label_now()
        if label is None:
            self._reject_reason = 'no ground truth or TF'
            return False

        # Behind the camera means not visible. Recording it would teach a
        # model to predict a position for a target that is not in the
        # picture, which is a different and harder problem than the one
        # this dataset is for.
        if label[2] <= 0.0:
            self._reject_reason = 'target behind the camera'
            return False

        msg = self.image
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        name = f'ep{episode:03d}_{index:04d}.png'
        # cv2 writes BGR; the camera publishes RGB.
        cv2.imwrite(str(self.out_dir / 'images' / name), rgb[:, :, ::-1])

        self.rows.append({
            'image': f'images/{name}',
            'episode': episode,
            'index': index,
            'label': list(label),
            'stamp': float(msg.header.stamp.sec
                           + msg.header.stamp.nanosec * 1e-9),
        })
        return True


def target_offsets(count, seed=0):
    """
    Return (forward, lateral) target placements spanning the workspace.

    A deterministic low-discrepancy sweep rather than uniform random
    sampling: with a few dozen episodes, random placement leaves visible
    gaps and clusters, and two runs of the same command are not
    comparable. The jitter keeps the placements off an exact lattice,
    which would otherwise make every label an exact multiple of the grid
    step and let a model memorise the grid instead of the geometry.

    Bounds are what the camera can actually see: nearer than 0.28 m the
    target leaves the bottom of the frame, further than 0.62 m it is a
    handful of pixels wide and the apparent-size range is meaningless.
    """
    rng = random.Random(seed)
    out = []
    for index in range(count):
        # Golden-ratio sequence in both axes, then jittered.
        forward = 0.28 + 0.34 * ((index * 0.6180339887) % 1.0)
        lateral = -0.13 + 0.26 * ((index * 0.3819660113) % 1.0)
        out.append((round(forward + rng.uniform(-0.01, 0.01), 4),
                    round(lateral + rng.uniform(-0.01, 0.01), 4)))
    return out


def main(argv=None):
    """Record a dataset."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/ws/datasets/target',
                        help='where to write the dataset')
    parser.add_argument('--episodes', type=int, default=40)
    parser.add_argument('--frames', type=int, default=12,
                        help='frames per episode')
    parser.add_argument('--world', default='bench_with_target')
    parser.add_argument('--repo', default='/ws')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = Collector(args.out, args.world)

    if not node.wait_ready():
        print('  no camera frames -- is the simulation running with '
              'WORLD=bench_with_target?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Per-STEP velocities, applied for about a third of a second each.
    # Gentle on purpose: a 62 degree field of view at 0.4 m loses the
    # target after roughly 30 degrees of yaw, and an episode that drives
    # past it records nothing. Every entry MOVES - a stationary episode
    # records the same picture repeatedly, which is what the first
    # version of this did.
    moves = [
        (0.05, 0.06), (-0.05, 0.06), (0.04, -0.07), (-0.04, -0.07),
        (0.06, 0.02), (-0.06, 0.02), (0.03, 0.09), (-0.03, -0.09),
    ]
    offsets = target_offsets(args.episodes, seed=args.seed)

    kept = 0
    for episode in range(args.episodes):
        # A NEW TARGET POSITION PER EPISODE. Without this the label
        # barely varies: every episode looks at the same cube from a
        # slightly different angle, and a model has almost nothing to
        # learn a mapping from. It also makes the episode-wise split
        # hold out unseen target PLACEMENTS, not just unseen viewpoints.
        forward, lateral = offsets[episode]
        if not node.place_target_ahead(forward, lateral):
            print(f'  episode {episode}: could not place the target, skipped')
            continue

        before = kept
        kept += node.record_episode(episode, *moves[episode % len(moves)],
                                    args.frames)
        print(f'  episode {episode:3d}  target {forward:.2f} m ahead, '
              f'{lateral:+.2f} m across  ->  {kept - before} frames '
              f'({kept} total)')

    out = pathlib.Path(args.out)
    with (out / 'labels.jsonl').open('w') as handle:
        for row in node.rows:
            handle.write(json.dumps(row) + '\n')

    info = node.info
    doc = manifest_mod.build(
        world=args.world,
        robot_profile='threevn_mm_v1',
        camera={'fx': info.k[0], 'fy': info.k[4], 'cx': info.k[2],
                'cy': info.k[5], 'width': info.width, 'height': info.height,
                'frame_id': node.image.header.frame_id},
        label_source={
            'source': 'simulator',
            'quantity': 'target position in the camera optical frame',
            'units': 'metres',
            # The target MOVES between episodes, so there is no single
            # pose to record. Its position per frame is what the label
            # is, and it is read from ground truth each time rather
            # than assumed.
            'target_repositioned_per_episode': True,
        },
        episodes=len({row['episode'] for row in node.rows}),
        frames=kept,
        repo=args.repo,
        extra={'seed': args.seed},
    )
    manifest_mod.write(out, doc)

    print(f'\n  wrote {kept} frames across '
          f'{len({row["episode"] for row in node.rows})} episodes to {out}')
    if node.rejected:
        print('  dropped:')
        for reason, count in node.rejected.most_common():
            print(f'    {count:4d}  {reason}')

    problems = manifest_mod.validate(doc)
    if problems:
        print('  manifest problems:')
        for problem in problems:
            print(f'    - {problem}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
