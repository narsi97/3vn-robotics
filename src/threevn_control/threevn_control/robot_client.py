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
The 3VN robot abstraction.

A small Python API over the ROS 2 interface, so scenarios read like
intent rather than plumbing. Crucially it is a *thin* wrapper: it adds no
transport of its own, it only calls the ROS primitives that
`ros2_control` already exposes.

That distinction matters. The specification asks for a robot abstraction
AND forbids a proprietary RPC layer. Those are compatible only if the
abstraction is a client-side convenience over standard ROS interfaces,
never a service sitting between the caller and the robot.

Which primitive, and why:

  topic   /joint_states          continuous state, many consumers,
                                 dropping a sample is harmless
  action  follow_joint_trajectory  motion takes seconds, needs feedback
                                 and must be cancellable
  action  gripper_cmd            same
  service controller_manager/*   short, synchronous, needs an answer

Because these come from `ros2_control`, this client works unchanged
against mock, Gazebo and (Phase 5) the ESP32. Nothing here knows or cares
which.
"""

import math
import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
from threevn_control.kinematics import (
    arm_geometry,
    choose_pitch,
    inverse_kinematics,
    Unreachable,
)
from trajectory_msgs.msg import JointTrajectoryPoint

#: The 3VN arm's own joint names. NOT what the client drives -- that is
#: asked of the controller, see arm_joints() -- but the names a robot
#: answers to when its controller cannot be reached, and the reference
#: joint_prefix() compares against to find a composed robot's prefix.
ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_joint',
]

# There is deliberately no POSES here any more.
#
# It was a dictionary of four-element lists: `home`, `ready`, `pick` and
# `place` for one particular arm, in a module that discovered everything
# else about the robot at runtime. The first five-joint arm stopped at
# home() with "expected 5 joint positions, got 4".
#
# Named poses are a fact about a robot, so they live in its profile, and
# the description carries them to whoever is driving it. See
# named_poses() below and urdf/arm/semantics.xacro.


def fetch_controller_param(node, controller, name, timeout=3.0):
    """
    Read one parameter off a running controller, or return None.

    Returns None rather than raising when the controller is absent: a
    base-only robot has no `arm_controller`, and a mock target may not
    have brought one up yet. Callers fall back to a default.
    """
    import rclpy
    client = node.create_client(
        GetParameters, f'/{controller}/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        result = future.result()
        if result is None or not result.values:
            return None
        value = result.values[0]
        # PARAMETER_STRING_ARRAY == 9, PARAMETER_STRING == 4.
        if value.type == 9:
            return list(value.string_array_value) or None
        if value.type == 4:
            return [value.string_value] if value.string_value else None
        return None
    finally:
        node.destroy_client(client)


class RobotNotReady(RuntimeError):
    """Raised when the robot interface does not appear within a timeout."""


class MotionFailed(RuntimeError):
    """Raised when a motion goal is rejected or does not succeed."""


class LimitViolation(ValueError):
    """
    Raised when a commanded position lies outside a joint's limit.

    This exists because of a measured finding, not a hypothetical. The
    URDF declares <limit> on every joint AND min/max on every
    command_interface, and NEITHER is enforced: commanding
    shoulder_pan_joint to 180 degrees against a 90 degree limit drives it
    to 180 under mock_components. Gazebo only looks safe because its
    physics joint has a hard stop, which is a property of the simulator
    rather than of our software.

    Validation therefore has to live above the hardware seam, where it
    applies to mock, Gazebo and the ESP32 alike. It is defence in depth,
    not a safety system - see docs/safety.md.
    """


class RobotClient(Node):
    """
    Client for one 3VN arm.

    Deliberately synchronous: scenarios are scripts, and a script that
    reads top to bottom is worth more here than concurrency nobody needs.
    """

    def __init__(self, node_name='threevn_robot_client', timeout=30.0):
        super().__init__(node_name)
        self._timeout = timeout
        self._state = None

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        self._arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self._gripper = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')

        # TF is needed for anything the joint states cannot answer -- most
        # importantly mimic joints, which robot_state_publisher resolves
        # into transforms but which never appear in /joint_states.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        #: The URDF the robot is running, kept whole. The limits are
        #: parsed out of it, and the IK is solved from it.
        self._description = None
        self._geometry = None

        #: Latest /clock, or None on a robot that publishes no sim time.
        #: Motion timeouts are spent in THIS clock -- see _wait().
        self._sim_time = None
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

        #: Resolved once, from the controller. See arm_joints().
        self._joints = None
        self._prefix = None

        #: joint name -> (lower, upper), parsed from /robot_description.
        self._limits = {}
        #: The <threevn_semantics> block of the running description: the
        #: arm's joints, the gripper's travel and the named poses. None
        #: until the description arrives, {} if it carries no such block.
        self._semantics = None
        #: follower joint -> the joint it mimics.
        self._mimics = {}
        self.create_subscription(
            String, '/robot_description', self._on_description,
            QoSProfile(depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

    # -- state ---------------------------------------------------------

    def _on_joint_state(self, msg):
        self._state = msg

    def _on_clock(self, msg):
        self._sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_description(self, msg):
        """Parse joint limits out of the URDF the robot is actually running."""
        import xml.etree.ElementTree as ET
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError:
            return
        self._description = msg.data
        for joint in root.iter('joint'):
            limit = joint.find('limit')
            name = joint.get('name')
            if limit is None or name is None:
                continue
            lower, upper = limit.get('lower'), limit.get('upper')
            if lower is not None and upper is not None:
                self._limits[name] = (float(lower), float(upper))
        for joint in root.iter('joint'):
            mimic = joint.find('mimic')
            if mimic is not None and joint.get('name'):
                self._mimics[joint.get('name')] = mimic.get('joint')
        self._semantics = self._parse_semantics(root)

    @staticmethod
    def _parse_semantics(root):
        """
        Read the <threevn_semantics> block out of a parsed description.

        Returns {} for a description without one, which is a legitimate
        robot: it can be moved joint by joint, it just has no named poses.
        """
        block = root.find('threevn_semantics')
        if block is None:
            return {}
        semantics = {'profile': block.get('profile'), 'poses': {}}
        arm = block.find('arm')
        if arm is not None:
            semantics['arm_joints'] = (arm.get('joints') or '').split()
            semantics['base_frame'] = arm.get('base_frame')
            semantics['tool_frame'] = arm.get('tool_frame')
        gripper = block.find('gripper')
        if gripper is not None:
            semantics['gripper'] = {
                'joint': gripper.get('joint'),
                'open': float(gripper.get('open')),
                'closed': float(gripper.get('closed')),
            }
        for pose in block.findall('pose'):
            semantics['poses'][pose.get('name')] = [
                float(v) for v in (pose.get('positions') or '').split()]
        return semantics

    def semantics(self):
        """
        Return what the running description says about itself.

        Waits for /robot_description if it has not arrived: the topic is
        latched, so on a robot that is up this returns at once.
        """
        if self._semantics is None:
            import rclpy
            deadline = time.monotonic() + self._timeout
            while self._semantics is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        if self._semantics is None:
            raise RobotNotReady('no /robot_description received yet')
        return self._semantics

    def profile(self):
        """Return the name of the profile this robot was built from, or None."""
        return self.semantics().get('profile')

    def named_poses(self):
        """
        Return this robot's named poses: name -> positions, in arm_joints() order.

        Read from the description the robot is RUNNING, for the same
        reason the limits are: a client attached to a robot gets that
        robot's poses, whichever profile launched it.

        The profile lists positions in the order of ITS `arm.joints`,
        and the controller drives them in the order of its own `joints`
        parameter. Those are two files, so the mapping goes by name
        rather than by position -- and a pose for joints the controller
        does not drive is an error that names both lists, instead of a
        trajectory sent to the wrong joints.
        """
        semantics = self.semantics()
        declared = semantics.get('arm_joints') or []
        driven = self.arm_joints()
        poses = {}
        for name, positions in semantics.get('poses', {}).items():
            if len(positions) != len(declared):
                raise ValueError(
                    f'pose {name!r} has {len(positions)} positions for '
                    f'{len(declared)} arm joints {declared}')
            by_joint = dict(zip(declared, positions))
            if set(by_joint) != set(driven):
                raise ValueError(
                    f'the poses in the description are for joints {declared}, '
                    f'but arm_controller drives {driven}')
            poses[name] = [by_joint[j] for j in driven]
        return poses

    def pose(self, pose_name):
        """Return one named pose, in arm_joints() order."""
        poses = self.named_poses()
        if not poses:
            raise ValueError(
                'the description of this robot declares no named poses; add a '
                '`poses:` block to its profile')
        if pose_name not in poses:
            raise ValueError(
                f'unknown pose {pose_name!r}; one of {sorted(poses)}')
        return poses[pose_name]

    def gripper_joint(self):
        """Return the joint the gripper is commanded and read through."""
        gripper = self.semantics().get('gripper')
        if gripper and gripper.get('joint'):
            return gripper['joint']
        return self.resolve('gripper_left_finger_joint')

    def gripper_travel(self):
        """
        Return the gripper's (closed, open) positions, in its joint's own unit.

        Metres on a prismatic gripper and radians on a revolute one. A
        caller that takes both ends from here never has to know which.
        Falls back to the joint's limits for a description that does not
        say.
        """
        gripper = self.semantics().get('gripper')
        if gripper:
            return gripper['closed'], gripper['open']
        limits = self.get_limits(self.gripper_joint())
        if limits is None:
            raise RobotNotReady('the gripper joint has no limits to open to')
        return limits

    def gripper_followers(self):
        """Return the joints that mimic the gripper joint, if any."""
        commanded = self.gripper_joint()
        return sorted(follower for follower, leader in self._mimics.items()
                      if leader == commanded)

    # -- validation ----------------------------------------------------

    def arm_joints(self):
        """
        Return the arm joints this robot actually has.

        Asked of `arm_controller` rather than assumed, because the name
        depends on the robot. The arm-only profile calls its first joint
        `shoulder_pan_joint`; the mobile manipulator calls the same joint
        `arm_shoulder_pan_joint`, because composing the arm onto a base
        prefixes it.

        Hardcoding either one drives that robot and fails silently on the
        other: the controller REJECTS a trajectory whose joint names it
        does not recognise, which surfaces as `goal rejected` with no clue
        that a name is at fault. That is exactly how this was found.

        The answer is cached, including the fallback, so a robot with no
        arm_controller costs one timeout rather than one per call.
        """
        if self._joints is None:
            discovered = fetch_controller_param(
                self, 'arm_controller', 'joints')
            self._joints = list(discovered) if discovered else list(ARM_JOINTS)
        return self._joints

    def joint_prefix(self):
        """
        Return the prefix this robot puts on the arm's joint names, or ''.

        Derived by comparing what the controller reports against the
        canonical names, so nothing has to declare it twice.
        """
        if self._prefix is None:
            prefix = ''
            joints = self.arm_joints()
            if len(joints) == len(ARM_JOINTS):
                for actual, canonical in zip(joints, ARM_JOINTS):
                    if not actual.endswith(canonical):
                        prefix = ''
                        break
                    candidate = actual[:len(actual) - len(canonical)]
                    if prefix and candidate != prefix:
                        prefix = ''
                        break
                    prefix = candidate
            self._prefix = prefix
        return self._prefix

    def resolve(self, joint):
        """Return `joint` under this robot's naming, accepting either form."""
        prefix = self.joint_prefix()
        if not prefix or joint.startswith(prefix):
            return joint
        return prefix + joint

    def get_limits(self, joint):
        """Return (lower, upper) for a joint, from the running description."""
        return self._limits.get(self.resolve(joint))

    def check_within_limits(self, joint, position):
        """
        Raise LimitViolation if `position` is outside the joint's limit.

        Reads the limits from /robot_description, so it validates against
        the description the robot is ACTUALLY running rather than a copy
        that may have drifted.
        """
        limits = self._limits.get(self.resolve(joint))
        if limits is None:
            return
        lower, upper = limits
        if not (lower <= position <= upper):
            raise LimitViolation(
                f'{joint}: {position:+.4f} rad '
                f'({math.degrees(position):+.1f} deg) is outside its limit '
                f'[{lower:+.4f}, {upper:+.4f}] rad '
                f'([{math.degrees(lower):+.1f}, {math.degrees(upper):+.1f}] deg)')

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            import rclpy
            rclpy.spin_once(self, timeout_sec=0.05)

    def connect(self):
        """
        Wait until state is flowing and both action servers are up.

        Called 'connect' because that is what it means to the caller. There
        is no connection in the TCP sense - ROS 2 discovery has already
        happened - but a scenario still needs to know the robot is ready.
        """
        deadline = time.monotonic() + self._timeout
        import rclpy
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self._state is not None
                    and self._arm.server_is_ready()
                    and self._gripper.server_is_ready()):
                return self
        missing = []
        if self._state is None:
            missing.append('/joint_states')
        if not self._arm.server_is_ready():
            missing.append('arm action server')
        if not self._gripper.server_is_ready():
            missing.append('gripper action server')
        raise RobotNotReady(
            f'robot not ready after {self._timeout:.0f}s: {", ".join(missing)}')

    def get_joint_state(self):
        """Return the latest joint positions as a name -> radians dict."""
        self._spin(0.1)
        if self._state is None:
            raise RobotNotReady('no /joint_states received yet')
        state = dict(zip(self._state.name, self._state.position))
        # Also answer to the unprefixed name, so a scenario written
        # against the arm reads the same joint on the mobile
        # manipulator. setdefault, never overwrite: a real joint that
        # happens to collide with an alias keeps its own value.
        prefix = self.joint_prefix()
        if prefix:
            for name, value in list(state.items()):
                if name.startswith(prefix):
                    state.setdefault(name[len(prefix):], value)
        return state

    def get_arm_pose(self):
        """Return the arm joint positions, in the controller's order."""
        state = self.get_joint_state()
        return [state[j] for j in self.arm_joints()]

    def get_translation(self, parent, child, timeout=2.0):
        """
        Return the (x, y, z) translation of `child` in `parent`.

        Use this for anything /joint_states cannot answer. A URDF <mimic>
        joint is the common case: it has no ros2_control interface by
        design, so it is absent from /joint_states, but
        robot_state_publisher still resolves it and publishes the
        transform. Reading state from the wrong source is why the gripper
        scenario originally failed against a robot that was working.
        """
        import rclpy
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self._tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
                v = tf.transform.translation
                return (v.x, v.y, v.z)
            except Exception as exc:                  # noqa: BLE001
                last = exc
        raise RobotNotReady(f'no transform {parent} -> {child}: {last}')

    def at_pose(self, target, tolerance=0.02):
        """Return True when every arm joint is within tolerance of target."""
        return all(abs(a - b) <= tolerance
                   for a, b in zip(self.get_arm_pose(), target))

    # -- reaching a point ----------------------------------------------

    def geometry(self):
        """
        Return this arm's measured geometry, for the IK.

        Taken from /robot_description, so it describes the robot that is
        actually running rather than a file that may have drifted -- and
        it works unchanged on the composed mobile manipulator, whose
        frames carry a prefix.
        """
        if self._geometry is None:
            # The frames come from the description, so that on an arm
            # this closed form does not fit, arm_geometry() gets as far
            # as the structure and refuses for the true reason -- rather
            # than failing to find a frame name that was only ever the
            # 3VN arm's. semantics() also waits for the description,
            # and raises RobotNotReady if it never arrives.
            semantics = self.semantics()
            self._geometry = arm_geometry(
                self._description,
                target=(semantics.get('tool_frame')
                        or self.resolve('grasp_frame')),
                base=(semantics.get('base_frame')
                      or self.resolve('base_link')))
        return self._geometry

    def approach_pitch(self, position, prefer='middle'):
        """Return an approach pitch that reaches `position`, or None."""
        return choose_pitch(self._description, position,
                            geometry=self.geometry(), prefer=prefer)

    def move_tool_to(self, position, pitch=None, duration=2.5):
        """
        Put the gripper at `position`, choosing an approach if none given.

        Raises Unreachable rather than moving somewhere near it, because
        a grasp that misses by a centimetre and reports success is worse
        than one that refuses.
        """
        if pitch is None:
            pitch = self.approach_pitch(position)
            if pitch is None:
                raise Unreachable(
                    f'no approach angle reaches {tuple(round(v, 3) for v in position)}')
        solution = inverse_kinematics(self._description, position, pitch=pitch,
                                      geometry=self.geometry())
        ordered = [solution[name] for name in self.arm_joints()]
        self.move_joints(ordered, duration=duration)
        return pitch

    # -- grasping ------------------------------------------------------

    def holding(self, threshold=0.004):
        """
        Return True when something is between the fingers.

        Measured, not assumed: a commanded-closed gripper that has
        stopped short is holding something. On this robot the fingers
        reach 1.7 mm on empty air, so anything above a few millimetres
        is an object.

        This works the same way on hardware, which is the point -- it
        reads the robot's own joint state rather than asking the
        simulator what exists. A check that only works in simulation
        cannot tell you a real grasp failed.
        """
        state = self.get_joint_state()
        name = self._gripper_feedback_joint()
        position = state.get(name)
        if position is None:
            raise RobotNotReady('no gripper joint in /joint_states')
        # Strictly BETWEEN closed-on-air and wide open. The upper bound
        # is not fussiness: a gripper that failed to close at all sits
        # at its open limit, which is also "greater than the
        # threshold". Testing only the lower bound reported a confident
        # grasp of an object the robot had merely pushed out of the way.
        limits = self.get_limits(name)
        ceiling = (limits[1] - 0.002) if limits else float('inf')
        return bool(threshold < position < ceiling)

    def pinch_offset(self):
        """
        Return how far the jaws' pinch point sits behind grasp_frame, in metres.

        Measured through TF, because it is not zero and assuming it was
        is a grasp that closes on nothing: the finger origins are 30 mm
        BEHIND grasp_frame, and the fingers are 35 mm long, which leaves
        grasp_frame about 12 mm past the fingertips. Driving grasp_frame
        to an object therefore puts the object beyond the jaws, and the
        gripper pushes it away instead of holding it.
        """
        frame = self.resolve('grasp_frame')
        left = self.get_translation(frame, self.resolve('gripper_left_finger_joint')
                                    .replace('_joint', '_link'))
        right = self.get_translation(frame, self.resolve('gripper_right_finger_joint')
                                     .replace('_joint', '_link'))
        return 0.5 * (left[2] + right[2])

    def grasp_pose(self, position, prefer='middle'):
        """
        Return (where grasp_frame must go, at what pitch) to grip `position`.

        Pitch and target have to be solved TOGETHER. The jaws pinch
        behind grasp_frame, so the commanded point is displaced from the
        object along the approach direction -- which means the
        displacement depends on the pitch, and whether the displaced
        point is reachable depends on the displacement. Choosing a pitch
        from the object's own position and then applying the offset
        gives a point that was never checked, and it is usually outside
        the workspace: the object sits in the reachable band, and the
        commanded point is 30 mm beyond it.

        So every candidate pitch is tested against its OWN displaced
        point. Returns None when none of them work.
        """
        import numpy as np
        from threevn_control.kinematics import PITCH_CANDIDATES, reachable

        offset = self.pinch_offset()
        yaw = math.atan2(position[1], position[0])
        origin = np.array(position, dtype=float)

        viable = []
        for pitch in PITCH_CANDIDATES:
            axis = np.array([math.sin(pitch) * math.cos(yaw),
                             math.sin(pitch) * math.sin(yaw),
                             math.cos(pitch)])
            candidate = tuple(origin - offset * axis)
            if reachable(self._description, candidate, pitch=pitch,
                         geometry=self.geometry()):
                viable.append((candidate, pitch))
        if not viable:
            return None
        if prefer == 'steepest':
            return viable[0]
        # Prefer the pose with the most JOINT margin, not the middle of
        # the pitch range. Measured: a solution that puts
        # shoulder_lift at 118 degrees against a 120 degree limit does
        # not arrive -- the arm settles 6 degrees short and the wrist 11
        # degrees short, missing by 43 mm, because a fully extended arm
        # at the edge of its travel cannot hold itself there. Poses well
        # inside the limits reach to under a millimetre. So the
        # tie-breaker is how far the worst joint sits from its stop.
        return max(viable, key=lambda item: self._limit_margin(item))

    def _limit_margin(self, item):
        """Return how close the tightest joint gets to its limit, in radians."""
        from threevn_control.kinematics import inverse_kinematics
        candidate, pitch = item
        try:
            solution = inverse_kinematics(self._description, candidate,
                                          pitch=pitch, geometry=self.geometry())
        except Unreachable:
            return -1.0
        worst = float('inf')
        for name, value in solution.items():
            bounds = self.get_limits(name)
            if not bounds:
                continue
            worst = min(worst, value - bounds[0], bounds[1] - value)
        return worst if worst != float('inf') else 0.0

    def _gripper_feedback_joint(self):
        return self.gripper_joint()

    def grasp_at(self, position, approach=0.06, pitch=None, duration=2.0):
        """
        Descend onto `position`, close, and report holding().

        The stand-off is directly ABOVE the object, and each waypoint
        solves for its own approach angle. The obvious alternative --
        backing off along the tool axis and coming in along it, holding
        one orientation -- cannot be done on this arm, and that is a
        measured fact rather than a style choice: at a fixed pitch the
        reachable set near the ground is a curve about 4-6 mm wide in
        radius. A stand-off of any useful size leaves it. Allowing the
        wrist to re-aim between waypoints is what buys the room.

        Returns what holding() reports, which reads the gripper's own
        joint state. Whether the object actually came with the robot is
        a separate question, and one the robot cannot answer about
        itself -- see scripts/verify_grasp.py, which lifts and checks.
        """
        chosen = self.grasp_pose(position)
        if chosen is None:
            raise Unreachable(
                f'the jaws cannot be placed on '
                f'{tuple(round(v, 3) for v in position)} at any approach angle')
        target, pitch = chosen

        self.open_gripper()
        for standoff in (approach, approach * 0.6, approach * 0.3):
            above = (target[0], target[1], target[2] + standoff)
            try:
                self.move_tool_to(above, duration=duration)
                break
            except (Unreachable, LimitViolation):
                continue
        else:
            # No room above it. Going straight there is worth trying;
            # it risks nudging the object, which holding() will notice.
            pass

        self.move_tool_to(target, pitch=pitch, duration=duration)
        self.close_gripper()
        self._spin(1.0)
        return self.holding()

    # -- motion --------------------------------------------------------

    def move_joints(self, positions, duration=2.0):
        """
        Move all arm joints to `positions` over `duration` seconds.

        Blocks until the action completes. Raises MotionFailed if the goal
        is rejected or finishes unsuccessfully - a scenario that silently
        continued past a failed motion would produce a misleading pass.
        """
        joints = self.arm_joints()
        if len(positions) != len(joints):
            raise ValueError(
                f'expected {len(joints)} joint positions, got {len(positions)}')

        # Reject before sending. Neither ros2_control nor mock_components
        # enforces the limits declared in the URDF, so a goal that leaves
        # here unchecked reaches the hardware unchecked.
        for joint, position in zip(joints, positions):
            self.check_within_limits(joint, float(position))

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=duration).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = [point]

        return self._send(self._arm, goal, duration + 15.0, 'arm motion')

    def move_joint(self, name, position, duration=2.0):
        """Move a single named joint, holding the others where they are."""
        joints = self.arm_joints()
        name = self.resolve(name)
        if name not in joints:
            raise ValueError(f'{name!r} is not an arm joint; one of {joints}')
        target = self.get_arm_pose()
        target[joints.index(name)] = float(position)
        return self.move_joints(target, duration)

    def move_to_named(self, pose_name, duration=2.0):
        """Move to one of this robot's named poses. See named_poses()."""
        return self.move_joints(self.pose(pose_name), duration)

    def home(self, duration=2.5):
        """Move to the reference configuration."""
        return self.move_to_named('home', duration)

    # -- gripper -------------------------------------------------------

    def set_gripper(self, position, max_effort=5.0):
        """
        Command the gripper joint to `position`.

        In the joint's own unit: an opening in metres on the 3VN arm's
        prismatic gripper, an angle in radians on a revolute jaw. Take
        the ends of the range from gripper_travel().
        """
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self._send(self._gripper, goal, 15.0, 'gripper')

    def open_gripper(self):
        """Open the gripper fully."""
        return self.set_gripper(self.gripper_travel()[1])

    def close_gripper(self, max_effort=5.0):
        """
        Close the gripper.

        `allow_stalling` is set on the controller, so closing onto an
        object succeeds rather than timing out - which is the whole point
        of a gripper.
        """
        return self.set_gripper(self.gripper_travel()[0], max_effort)

    # -- internals -----------------------------------------------------

    def _wait(self, future, timeout):
        """
        Spin until `future` completes, spending `timeout` seconds of ROBOT time.

        Wall time is the wrong clock to measure a motion with. A 2.5 s
        trajectory takes 2.5 seconds of SIMULATION time, and this
        simulation runs at roughly a third of real time on CPU physics --
        so a wall-clock deadline that is comfortable on a fast machine
        expires mid-motion on a slow one, and the failure reads as "no
        result", as though the robot were broken.

        So the deadline is spent in /clock when /clock exists, which makes
        it independent of how fast the simulator happens to be running.
        On real hardware nothing publishes /clock, and wall time is then
        the only clock there is, so it is used instead. The absolute wall
        cap is a backstop for the case that matters most: a simulator that
        has stopped advancing, where a sim-time deadline would never
        arrive at all.
        """
        import rclpy
        started_wall = time.monotonic()
        wall_cap = started_wall + max(timeout * 30.0, 120.0)
        started_sim = None
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            now_sim = self._sim_time
            if now_sim is None:
                if time.monotonic() - started_wall > timeout:
                    return False
            else:
                if started_sim is None:
                    started_sim = now_sim
                elif now_sim - started_sim > timeout:
                    return False
            if time.monotonic() > wall_cap:
                return False
        return True

    def _send(self, client, goal, timeout, what):
        if not client.wait_for_server(timeout_sec=self._timeout):
            raise RobotNotReady(f'{what}: action server unavailable')

        send = client.send_goal_async(goal)
        self._wait(send, timeout)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise MotionFailed(f'{what}: goal rejected')

        result = handle.get_result_async()
        self._wait(result, timeout)
        outcome = result.result()
        if outcome is None:
            raise MotionFailed(
                f'{what}: no result within {timeout:.0f}s of robot time')
        # status 4 == STATUS_SUCCEEDED
        if outcome.status != 4:
            raise MotionFailed(f'{what}: finished with status {outcome.status}')
        return outcome.result


def degrees(*values):
    """Convert degrees to radians, for writing poses readably."""
    return [math.radians(v) for v in values]
