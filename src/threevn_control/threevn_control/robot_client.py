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
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_joint',
]

#: Joint-space poses used by the scenarios. Radians.
POSES = {
    #: All joints at zero: the arm straight up. This is the reference
    #: configuration, not a rest position - an unpowered arm falls.
    'home': [0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, 0.5, -0.4, 0.0],
    'pick': [0.6, 0.9, -0.7, 0.2],
    'place': [-0.6, 0.9, -0.7, 0.2],
}


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

        #: joint name -> (lower, upper), parsed from /robot_description.
        self._limits = {}
        self.create_subscription(
            String, '/robot_description', self._on_description,
            QoSProfile(depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

    # -- state ---------------------------------------------------------

    def _on_joint_state(self, msg):
        self._state = msg

    def _on_description(self, msg):
        """Parse joint limits out of the URDF the robot is actually running."""
        import xml.etree.ElementTree as ET
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError:
            return
        for joint in root.iter('joint'):
            limit = joint.find('limit')
            name = joint.get('name')
            if limit is None or name is None:
                continue
            lower, upper = limit.get('lower'), limit.get('upper')
            if lower is not None and upper is not None:
                self._limits[name] = (float(lower), float(upper))

    # -- validation ----------------------------------------------------

    def get_limits(self, joint):
        """Return (lower, upper) for a joint, from the running description."""
        return self._limits.get(joint)

    def check_within_limits(self, joint, position):
        """
        Raise LimitViolation if `position` is outside the joint's limit.

        Reads the limits from /robot_description, so it validates against
        the description the robot is ACTUALLY running rather than a copy
        that may have drifted.
        """
        limits = self._limits.get(joint)
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
        return dict(zip(self._state.name, self._state.position))

    def get_arm_pose(self):
        """Return the four arm joint positions, in ARM_JOINTS order."""
        state = self.get_joint_state()
        return [state[j] for j in ARM_JOINTS]

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

    # -- motion --------------------------------------------------------

    def move_joints(self, positions, duration=2.0):
        """
        Move all arm joints to `positions` over `duration` seconds.

        Blocks until the action completes. Raises MotionFailed if the goal
        is rejected or finishes unsuccessfully - a scenario that silently
        continued past a failed motion would produce a misleading pass.
        """
        if len(positions) != len(ARM_JOINTS):
            raise ValueError(
                f'expected {len(ARM_JOINTS)} joint positions, got {len(positions)}')

        # Reject before sending. Neither ros2_control nor mock_components
        # enforces the limits declared in the URDF, so a goal that leaves
        # here unchecked reaches the hardware unchecked.
        for joint, position in zip(ARM_JOINTS, positions):
            self.check_within_limits(joint, float(position))

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=duration).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [point]

        return self._send(self._arm, goal, duration + 15.0, 'arm motion')

    def move_joint(self, name, position, duration=2.0):
        """Move a single named joint, holding the others where they are."""
        if name not in ARM_JOINTS:
            raise ValueError(f'{name!r} is not an arm joint; one of {ARM_JOINTS}')
        target = self.get_arm_pose()
        target[ARM_JOINTS.index(name)] = float(position)
        return self.move_joints(target, duration)

    def move_to_named(self, pose_name, duration=2.0):
        """Move to one of the named poses in POSES."""
        if pose_name not in POSES:
            raise ValueError(f'unknown pose {pose_name!r}; one of {sorted(POSES)}')
        return self.move_joints(POSES[pose_name], duration)

    def home(self, duration=2.5):
        """Move to the reference configuration."""
        return self.move_to_named('home', duration)

    # -- gripper -------------------------------------------------------

    def set_gripper(self, position, max_effort=5.0):
        """Command the gripper to an opening in metres (0 = closed)."""
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self._send(self._gripper, goal, 15.0, 'gripper')

    def open_gripper(self):
        """Open the gripper fully."""
        return self.set_gripper(0.018)

    def close_gripper(self, max_effort=5.0):
        """
        Close the gripper.

        `allow_stalling` is set on the controller, so closing onto an
        object succeeds rather than timing out - which is the whole point
        of a gripper.
        """
        return self.set_gripper(0.0, max_effort)

    # -- internals -----------------------------------------------------

    def _send(self, client, goal, timeout, what):
        import rclpy
        if not client.wait_for_server(timeout_sec=self._timeout):
            raise RobotNotReady(f'{what}: action server unavailable')

        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=timeout)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise MotionFailed(f'{what}: goal rejected')

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        outcome = result.result()
        if outcome is None:
            raise MotionFailed(f'{what}: no result within {timeout:.0f}s')
        # status 4 == STATUS_SUCCEEDED
        if outcome.status != 4:
            raise MotionFailed(f'{what}: finished with status {outcome.status}')
        return outcome.result


def degrees(*values):
    """Convert degrees to radians, for writing poses readably."""
    return [math.radians(v) for v in values]
