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
Gazebo: the gripper is driven by its servo, from the moment it spawns.

The 3VN arm's gripper used to sit on its hard stops. Its travel and its
<limit> were the same numbers, so it spawned on the lower stop and was
commanded onto both. Gazebo's physics engine (DART) drives each joint
with ONE constraint per step, in the order position limit, velocity
limit, servo. A joint at or past a stop gets the stop and not its servo.
Rounding decided which side of the stop the finger landed on: at
-5.4e-21 m every command stalled, for good, and the gripper "did not
move". See docs/manipulation.md.

These tests fail on that description. The first two measure the state
it left the finger in. The last one makes the failure deterministic: a
press far too light to beat the servo, which on a stop locked the finger
anyway.

Marked `slow`; `make test-sim` runs it. The simulator fixture is in
conftest.py. This module gets its own, so the finger is measured as it
spawned, before anything has commanded it.
"""

import time
import xml.etree.ElementTree as ET

from conftest import PROFILE
from conftest import run
import pytest
from threevn_control.kinematics import tool_position
from threevn_robot_description.describe import robot_facts

pytestmark = pytest.mark.slow

ROBOT = robot_facts(PROFILE, target='gz')
JOINT = ROBOT['gripper_joint']
LOWER, UPPER = ROBOT['limits'][JOINT]
CLOSED, OPEN = ROBOT['gripper']['closed'], ROBOT['gripper']['open']

#: sim.launch.py's default world.
WORLD = 'empty_bench'

#: How far from either hard stop the finger must be held, in the joint's
#: own unit. Half the clearance the 3VN profiles ship, so the margin can
#: be tuned but not removed. A finger asymptotically approaching a stop
#: from a servo command rests within a micron of it, which is one nudge
#: from the lockout these tests exist for.
STOP_CLEARANCE = 0.0005

#: How close a commanded position has to be reached. The gripper
#: controller's own goal_tolerance is 10 mm by default, more than half
#: this gripper's travel, so its `reached_goal` says almost nothing.
#: These tests read /joint_states instead.
ARRIVED = 0.001

#: The press, in newtons. A twentieth of the finger's 10 N effort limit,
#: and half of the ~0.1 N that does beat it in DART: mass x velocity
#: limit / time step, 0.002 kg x 0.05 m/s / 1 ms. Any harder and the
#: velocity-limit constraint replaces the servo and the press wins on
#: ANY description, which would make this test about something else.
PRESS = 0.05


def finger_position(attempts=3):
    """Return the commanded finger's position from /joint_states."""
    out = ''
    for _ in range(attempts):
        code, out = run(['timeout', '10', 'ros2', 'topic', 'echo',
                         '/joint_states', '--once'], timeout=20)
        lines = out.splitlines()
        if code == 0 and any(ln.startswith('position:') for ln in lines):
            break
    else:
        raise AssertionError(f'no /joint_states in {attempts} attempts:\n{out}')
    names_at = lines.index('name:')
    position_at = next(i for i, ln in enumerate(lines) if ln.startswith('position:'))
    names = [ln.strip('- ').strip() for ln in lines[names_at + 1:position_at]]
    values = []
    for ln in lines[position_at + 1:]:
        if not ln.startswith('- '):
            break
        values.append(float(ln[2:]))
    return dict(zip(names, values))[JOINT]


def command(position):
    """Send one gripper goal straight to the action; return (code, output)."""
    goal = f'{{command: {{position: {position}, max_effort: 5.0}}}}'
    return run(['ros2', 'action', 'send_goal', '/gripper_controller/gripper_cmd',
                'control_msgs/action/GripperCommand', goal], timeout=60)


def settle_at(target, timeout=10.0):
    """Return where the finger is once it reaches `target` or stops trying."""
    deadline = time.monotonic() + timeout
    position = finger_position()
    while abs(position - target) > ARRIVED and time.monotonic() < deadline:
        time.sleep(0.5)
        position = finger_position()
    return position


def clearance(position):
    """Return how far `position` is from the nearer hard stop."""
    return min(position - LOWER, UPPER - position)


def closing_direction():
    """
    Return the unit vector, in world, along which the finger closes.

    The model spawns at the world origin, unrotated, with every arm joint
    at zero, so the description's root frame is the world. Measured
    through the description's own kinematics rather than assumed, so a
    profile that mounts the gripper differently is still pressed the
    right way.
    """
    import numpy as np
    # findall, not iter: <ros2_control> holds <joint> elements of its own.
    root = ET.fromstring(ROBOT['urdf'])
    joints = root.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    base = next(link.get('name') for link in root.findall('link')
                if link.get('name') not in children)
    finger = next(j.find('child').get('link') for j in joints
                  if j.get('name') == JOINT)
    closed = np.array(tool_position(ROBOT['urdf'], {JOINT: CLOSED},
                                    target=finger, base=base))
    opened = np.array(tool_position(ROBOT['urdf'], {JOINT: OPEN},
                                    target=finger, base=base))
    along = closed - opened
    return along / np.linalg.norm(along), f"{ROBOT['model']}::{finger}"


def test_the_finger_spawns_clear_of_its_stops(simulator):
    """
    Before anything commands it, the finger is not on a hard stop.

    The old description spawned it AT its lower stop. Measured over six
    spawns it rested between -5.4e-21 m and +2.0e-11 m: on the stop, and
    on either side of it according to rounding.
    """
    position = finger_position()
    assert clearance(position) >= STOP_CLEARANCE, (
        f'{JOINT} spawned at {position!r}, {clearance(position):.3g} from a '
        f'hard stop of [{LOWER}, {UPPER}]. A joint on its stop is held by the '
        'stop, not driven by its servo: the gripper may not move at all.')


def test_the_gripper_opens_and_closes(simulator):
    """Open, closed, open, closed: each one reached, none of them stalled."""
    for target in (OPEN, CLOSED, OPEN, CLOSED):
        code, out = command(target)
        assert code == 0, out
        position = settle_at(target)
        assert abs(position - target) <= ARRIVED, (
            f'commanded {target} m, {JOINT} stopped at {position!r}:\n{out}')
        assert clearance(position) >= STOP_CLEARANCE, (
            f'{JOINT} came to rest {clearance(position):.3g} from a hard stop')


def test_a_light_press_does_not_lock_the_finger(simulator):
    """
    A press the servo can easily resist does not stop the gripper opening.

    The finger is closed and pressed toward its closed end with PRESS
    newtons, then told to open. A servo held clear of its stop resists
    the press and opens. A finger at its stop is locked by the same press,
    and on the old description it stayed locked after the press ended.
    """
    code, out = run(
        ['gz', 'service', '-s', f'/world/{WORLD}/entity/system/add',
         '--reqtype', 'gz.msgs.EntityPlugin_V', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req',
         # Entity 1 is the world: gz-sim creates it first.
         'entity: {id: 1} plugins: {name: "gz::sim::systems::ApplyLinkWrench" '
         'filename: "gz-sim-apply-link-wrench-system"}'])
    assert code == 0 and 'true' in out, f'could not add ApplyLinkWrench:\n{out}'

    direction, link = closing_direction()
    force = ', '.join(f'{axis}: {PRESS * value:.6f}'
                      for axis, value in zip('xyz', direction))
    code, out = command(CLOSED)
    assert code == 0, out
    settle_at(CLOSED)
    try:
        code, out = run(
            ['gz', 'topic', '-t', f'/world/{WORLD}/wrench/persistent',
             '-m', 'gz.msgs.EntityWrench', '-p',
             f'entity: {{name: "{link}", type: LINK}}, wrench: {{force: {{{force}}}}}'])
        assert code == 0, out
        time.sleep(1.5)
        pressed = finger_position()

        code, out = command(OPEN)
        assert code == 0, out
        position = settle_at(OPEN)
        assert abs(position - OPEN) <= ARRIVED, (
            f'pressed toward closed with {PRESS} N, {JOINT} sat at {pressed!r} '
            f'and, told to open, got to {position!r}. A servo loses to a press '
            'that light only when its joint is on a hard stop.')
    finally:
        run(['gz', 'topic', '-t', f'/world/{WORLD}/wrench/clear',
             '-m', 'gz.msgs.Entity', '-p', f'name: "{link}", type: LINK'])
