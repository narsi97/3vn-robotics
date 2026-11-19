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
Keyboard teleoperation: the entry point for driving the robot by hand.

Every other entry point in this repository *verifies* something. The
scenarios assert, the harnesses measure, CI fails. That is the right
default for a project about engineering practice, but it left a real gap:
someone with no hardware had no way to simply move the robot and watch
what happens. This closes it.

It is written against the same three ROS primitives as everything else,
so it drives mock, Gazebo and (Phase 5) the ESP32 without knowing which.

Why this is not `teleop_twist_keyboard`:

  * `diff_drive_controller` in Jazzy subscribes to **TwistStamped**, and
    the stock node publishes bare `Twist`. Nothing errors -- the topic
    name matches, discovery succeeds, the robot simply never moves. That
    is a far worse failure than a crash, and it is why this file exists
    rather than a `ros2 run` line in the docs.
  * Jogging an arm needs the joint limits, and a generic teleop has no
    idea what they are. These come from `/robot_description`, i.e. from
    the URDF the robot is *actually running* rather than a copy.

Joint names are asked for, never assumed. The arm-only profile calls its
first joint `shoulder_pan_joint`; the mobile manipulator calls the same
joint `arm_shoulder_pan_joint`, because composing the arm onto the base
prefixes it. A hardcoded list therefore drives one robot and silently
fails on the other -- the trajectory is accepted, the names match nothing
and no joint moves. So this node reads the `joints` parameter off
`arm_controller` itself, which is by definition the list that controller
will act on, in the order it wants them.

Two behaviours are properties of the robot, not of this script:

  * **Drive keys are hold-to-move.** `DiffDriveController` brakes when it
    has not heard a command within `cmd_vel_timeout`, so a velocity must
    be republished continuously. This node republishes at 20 Hz and
    stops 0.6 s after the last drive key, which turns the controller's
    own watchdog into the safety behaviour you want. Release the key and
    the robot stops because the *controller* decided to, which is
    exactly what should happen on a real machine.
  * **Arm jogs go to the trajectory topic, not the action.** The topic is
    fire-and-forget, which is what jogging wants. The action waits for a
    result, and waiting would stall the drive publisher mid-keystroke --
    making the base stutter every time you moved a joint.
"""

from dataclasses import dataclass
import math
import select
import sys
import termios
import time
import tty
from typing import Any

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import TwistStamped
import rclpy
from threevn_control.robot_client import fetch_controller_param, RobotClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#: Drive keys -> (linear scale, angular scale), each later multiplied by
#: the current speed setting. WASD because it needs no explanation.
DRIVE_KEYS = {
    'w': (+1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, +1.0),
    'd': (0.0, -1.0),
}

#: Jog the selected joint. ',' and '.' carry '<' and '>' on a QWERTY
#: board, so the direction is legible without reading the help.
JOG_KEYS = {',': -1, '.': +1}

#: Gripper: '[' opens, ']' closes. Bracket shape mirrors the fingers.
GRIPPER_KEYS = {'[': +1, ']': -1}

#: Speed scaling, applied to drive and jog alike.
SPEED_KEYS = {'-': -1, '_': -1, '+': +1, '=': +1}

#: Seconds a drive command survives its keystroke. Longer than the
#: terminal's key-repeat interval (~30 ms typical, 500 ms worst case
#: before the first repeat) so held keys drive smoothly, short enough
#: that a released key stops the robot promptly.
HOLD_SECONDS = 0.6

#: Republish rate for the base velocity, in Hz.
PUBLISH_HZ = 20.0

#: Speed steps, in (m/s, rad/s). Index into this with '-' and '+'.
SPEEDS = (
    (0.05, 0.20),
    (0.10, 0.40),
    (0.20, 0.80),
    (0.35, 1.20),
)

#: Radians per jog keystroke, at each speed step.
JOG_STEPS = (0.02, 0.05, 0.10, 0.20)

HELP = """
  3VN teleop -- drive the robot from the keyboard.

  base        w  forward     s  back      a  left      d  right
              space  stop now
  arm         1..9  select joint      ,  jog down      .  jog up
              tab   next joint        h  home the arm
  gripper     [  open        ]  close
  speed       - _  slower    + =  faster
  other       ?  this help   q  quit

  Drive keys are hold-to-move: let go and the controller brakes.
"""


@dataclass(frozen=True)
class Intent:
    """
    One keystroke resolved into what the robot should do.

    Separating this from the node is what makes the key map testable
    without a running robot, a simulator or even an rclpy context.
    """

    kind: str
    value: Any = None


def interpret(key):
    """
    Resolve a single keystroke into an `Intent`.

    Pure: no ROS, no I/O, no clock. Unknown keys resolve to an
    `Intent('unknown', key)` rather than raising, because a teleop that
    exits when you fat-finger a key is useless.
    """
    if not key:
        return Intent('unknown', key)
    if len(key) == 1 and key.isalpha():
        key = key.lower()
    if key in ('q', '\x03', '\x04'):
        return Intent('quit')
    if key == ' ':
        return Intent('stop')
    if key == '\t':
        return Intent('cycle')
    if key == '?':
        return Intent('help')
    if key == 'h':
        return Intent('home')
    if key in DRIVE_KEYS:
        return Intent('drive', DRIVE_KEYS[key])
    if key in JOG_KEYS:
        return Intent('jog', JOG_KEYS[key])
    if key in GRIPPER_KEYS:
        return Intent('gripper', GRIPPER_KEYS[key])
    if key in SPEED_KEYS:
        return Intent('speed', SPEED_KEYS[key])
    if key.isdigit() and key != '0':
        return Intent('select', int(key) - 1)
    return Intent('unknown', key)


def clamp(value, limits):
    """Clamp `value` into `limits`, or return it unchanged if limits is None."""
    if limits is None:
        return value
    lower, upper = limits
    return max(lower, min(upper, value))


class RawTerminal:
    """
    Put the terminal in cbreak mode for the duration of a `with` block.

    Restoring the mode in `__exit__` is not a nicety: leaving cbreak set
    means the shell stops echoing what the user types after this process
    exits, which looks exactly like a hung terminal.
    """

    def __init__(self, stream=None):
        self._stream = stream or sys.stdin
        self._saved = None

    def __enter__(self):
        """Switch to cbreak mode and return self."""
        self._saved = termios.tcgetattr(self._stream.fileno())
        tty.setcbreak(self._stream.fileno())
        return self

    def __exit__(self, *exc):
        """Restore the terminal mode, whatever happened inside the block."""
        if self._saved is not None:
            termios.tcsetattr(
                self._stream.fileno(), termios.TCSADRAIN, self._saved)
        return False

    def read_key(self, timeout):
        """Return one keystroke, or None if `timeout` seconds pass first."""
        ready, _, _ = select.select([self._stream], [], [], timeout)
        if not ready:
            return None
        return self._stream.read(1)


class Teleop(RobotClient):
    """
    Keyboard teleoperation node.

    Subclasses `RobotClient` to inherit joint state, the joint limits
    parsed from `/robot_description`, the gripper action and `home()`.
    Re-deriving any of those here would be a second copy of a decision
    that already exists -- the failure mode this project keeps hitting.
    """

    def __init__(self):
        super().__init__(node_name='threevn_teleop')
        self._cmd_vel = self.create_publisher(
            TwistStamped, '/base_controller/cmd_vel', 10)
        self._arm_traj = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        #: The joints this robot actually has. RobotClient.arm_joints()
        #: asks the controller; see the module docstring for why.
        self._joints = self.arm_joints()
        gripper = fetch_controller_param(self, 'gripper_controller', 'joint')
        self._gripper_joint = (
            gripper[0] if gripper else 'gripper_left_finger_joint')

        self._speed = 1
        self._joint = 0
        self._drive = (0.0, 0.0)
        self._drive_at = 0.0
        self._gripper_target = 0.0
        self._message = ''
        #: Arm target held here rather than read back each keystroke: a
        #: jog must accumulate from the last commanded position, not from
        #: the measured one, or every step fights the tracking error and
        #: the arm creeps instead of moving.
        self._target = None

    def ready(self):
        """Return True once limits and joint states have both arrived."""
        return bool(self.get_limits(self._joints[0])) and self._state is not None

    def report(self):
        """Print what was discovered, so a surprise is visible not silent."""
        print(f'  joints: {", ".join(self._joints)}')
        if not self.get_limits(self._joints[0]):
            print('  warning: no limits from /robot_description; '
                  'jogs will not be clamped')

    # -- base ----------------------------------------------------------

    def _publish_drive(self):
        """Republish the current base velocity, expiring stale commands."""
        if self._drive != (0.0, 0.0) and \
                time.monotonic() - self._drive_at > HOLD_SECONDS:
            self._drive = (0.0, 0.0)
        linear, angular = self._drive
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self._cmd_vel.publish(msg)

    def _on_drive(self, scales):
        linear_scale, angular_scale = scales
        linear_max, angular_max = SPEEDS[self._speed]
        self._drive = (linear_scale * linear_max, angular_scale * angular_max)
        self._drive_at = time.monotonic()

    # -- arm -----------------------------------------------------------

    def _measured(self):
        """Return measured arm positions from /joint_states, or None."""
        state = self._state
        if state is None:
            return None
        index = {name: i for i, name in enumerate(state.name)}
        try:
            return [state.position[index[j]] for j in self._joints]
        except (KeyError, IndexError):
            return None

    def _current_target(self):
        """Return the arm target, seeded from the measured pose on first use."""
        if self._target is None:
            pose = self._measured()
            self._target = list(pose) if pose else [0.0] * len(self._joints)
        return self._target

    def _send_arm(self, positions, seconds=0.35):
        msg = JointTrajectory()
        msg.joint_names = list(self._joints)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
        msg.points = [point]
        self._arm_traj.publish(msg)

    def _on_jog(self, direction):
        target = self._current_target()
        if not 0 <= self._joint < len(target):
            return
        name = self._joints[self._joint]
        step = JOG_STEPS[self._speed] * direction
        limits = self.get_limits(name)
        wanted = target[self._joint] + step
        clamped = clamp(wanted, limits)
        if clamped != wanted:
            self._message = f'{name} at its limit'
        target[self._joint] = clamped
        self._send_arm(target)

    def _on_home(self):
        self._target = [0.0] * len(self._joints)
        self._send_arm(self._target, seconds=2.0)
        self._message = 'homing'

    # -- gripper -------------------------------------------------------

    def _on_gripper(self, direction):
        """Open or close the gripper without waiting for the result."""
        # Between closed and open as the robot's own description gives
        # them, NOT between the joint's limits. Both robots' limits now
        # include slack past closed and past open (the 3VN arm's by 1 mm
        # since its gripper stopped moving in Gazebo), and the difference
        # is not cosmetic in the simulator: a joint parked exactly on a
        # hard stop can stay there, because the physics engine stops
        # driving a joint that is on a limit. Four presses span the travel.
        travel = self.gripper_travel()
        limits = (min(travel), max(travel))
        span = limits[1] - limits[0]
        self._gripper_target = clamp(
            self._gripper_target + direction * span / 4.0, limits)
        goal = GripperCommand.Goal()
        goal.command.position = float(self._gripper_target)
        goal.command.max_effort = 5.0
        # Fire and forget: waiting here would stall the drive publisher.
        self._gripper.send_goal_async(goal)

    # -- loop ----------------------------------------------------------

    def _status(self):
        linear, angular = SPEEDS[self._speed]
        name = (self._joints[self._joint]
                if self._joint < len(self._joints) else '-')
        target = self._current_target()
        angle = target[self._joint] if self._joint < len(target) else 0.0
        note = f'  {self._message}' if self._message else ''
        return (f'\r  speed {self._speed + 1}/{len(SPEEDS)} '
                f'({linear:.2f} m/s {angular:.2f} rad/s)  |  '
                f'{name} {math.degrees(angle):+6.1f} deg'
                f'{note}\033[K')

    def run(self):
        """Read keys until the user quits. Blocks; restores the terminal."""
        if not sys.stdin.isatty():
            print('teleop needs a terminal: run it with `make play` or '
                  '`docker compose exec ros ...` (no -T).', file=sys.stderr)
            return 1
        print(HELP)
        period = 1.0 / PUBLISH_HZ
        with RawTerminal() as term:
            while True:
                key = term.read_key(period)
                if key is not None:
                    intent = interpret(key)
                    if intent.kind == 'quit':
                        break
                    self._dispatch(intent)
                self._publish_drive()
                rclpy.spin_once(self, timeout_sec=0.0)
                sys.stdout.write(self._status())
                sys.stdout.flush()
        # Leave the robot stopped, not coasting on the last command.
        self._drive = (0.0, 0.0)
        for _ in range(5):
            self._publish_drive()
            time.sleep(0.02)
        print('\n  stopped')
        return 0

    def _dispatch(self, intent):
        self._message = ''
        if intent.kind == 'drive':
            self._on_drive(intent.value)
        elif intent.kind == 'stop':
            self._drive = (0.0, 0.0)
        elif intent.kind == 'jog':
            self._on_jog(intent.value)
        elif intent.kind == 'select':
            if intent.value < len(self._joints):
                self._joint = intent.value
            else:
                self._message = f'no joint {intent.value + 1}'
        elif intent.kind == 'cycle':
            self._joint = (self._joint + 1) % len(self._joints)
        elif intent.kind == 'gripper':
            self._on_gripper(intent.value)
        elif intent.kind == 'home':
            self._on_home()
        elif intent.kind == 'speed':
            self._speed = max(
                0, min(len(SPEEDS) - 1, self._speed + intent.value))
        elif intent.kind == 'help':
            print(HELP)


def main(argv=None):
    """Run the teleop node."""
    rclpy.init(args=argv)
    node = Teleop()
    try:
        # Deliberately not RobotClient.connect(): that waits for the arm
        # action server, and teleop is useful against a base-only robot
        # with no arm at all. Spin briefly so /robot_description and the
        # first /joint_states arrive before the status line is drawn.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.report()
        return node.run()
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
