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
Tests for the teleop key map.

`interpret` and `clamp` are pure, so all of this runs with no robot, no
simulator and no rclpy context -- which is the whole reason the key map
was separated from the node.
"""

from threevn_control.teleop import (
    clamp,
    DRIVE_KEYS,
    GRIPPER_KEYS,
    HELP,
    interpret,
    JOG_KEYS,
    SPEED_KEYS,
)

#: Keys that do not appear literally in the help text, and the word the
#: help uses for them instead.
SPELLED_OUT = {' ': 'space', '\t': 'tab'}


def test_every_drive_key_resolves_to_a_distinct_direction():
    """WASD must produce four different velocity pairs, not aliases."""
    directions = {interpret(key).value for key in DRIVE_KEYS}
    assert len(directions) == len(DRIVE_KEYS)
    for key in DRIVE_KEYS:
        assert interpret(key).kind == 'drive'


def test_forward_and_back_are_opposite():
    """A sign error here would be invisible until the robot hit a wall."""
    forward = interpret('w').value
    back = interpret('s').value
    assert forward[0] == -back[0]
    assert forward[0] > 0


def test_left_and_right_are_opposite_and_purely_angular():
    """Turning must not also translate: skid-steer scrub is bad enough."""
    left, right = interpret('a').value, interpret('d').value
    assert left[1] == -right[1]
    assert left[0] == 0.0 and right[0] == 0.0
    assert left[1] > 0, 'positive yaw is left under REP-103'


def test_alpha_keys_are_case_insensitive():
    """Caps lock should not silently disable the robot."""
    for key in ('w', 'a', 's', 'd', 'h', 'q'):
        assert interpret(key.upper()) == interpret(key)


def test_quit_covers_ctrl_c_and_ctrl_d():
    """Both terminal interrupts must leave the loop, not just 'q'."""
    for key in ('q', '\x03', '\x04'):
        assert interpret(key).kind == 'quit'


def test_joint_selection_is_zero_based():
    """The user types 1 for the first joint; the list index is 0."""
    assert interpret('1') == interpret('1')
    assert interpret('1').kind == 'select'
    assert interpret('1').value == 0
    assert interpret('4').value == 3


def test_zero_is_not_a_joint():
    """There is no joint 0 on the keypad, so '0' must not select index -1."""
    assert interpret('0').kind == 'unknown'


def test_unknown_keys_do_not_raise():
    """A fat-fingered key must never end the session."""
    for key in ('z', '%', '\x1b', '', None):
        assert interpret(key).kind == 'unknown'


def test_jog_directions_are_opposite():
    """',' and '.' must jog opposite ways or the help text lies."""
    assert interpret(',').value == -interpret('.').value


def test_gripper_open_and_close_are_opposite():
    """'[' opens and ']' closes; equal magnitudes, opposite signs."""
    assert interpret('[').value == -interpret(']').value
    assert interpret('[').value > 0


def test_speed_aliases_agree():
    """'+' and '=' share a physical key, so they must mean the same thing."""
    assert interpret('+') == interpret('=')
    assert interpret('-') == interpret('_')
    assert interpret('+').value == -interpret('-').value


def test_clamp_passes_through_when_limits_are_unknown():
    """
    A joint with no <limit> must still be joggable.

    Continuous joints have no limits in the URDF. Clamping to a default
    range would quietly restrict a joint the description says is free.
    """
    assert clamp(99.0, None) == 99.0


def test_clamp_holds_the_bounds():
    """Clamping is inclusive, and does not move a value already inside."""
    assert clamp(2.0, (-1.0, 1.0)) == 1.0
    assert clamp(-2.0, (-1.0, 1.0)) == -1.0
    assert clamp(0.5, (-1.0, 1.0)) == 0.5
    assert clamp(1.0, (-1.0, 1.0)) == 1.0


def test_every_bound_key_is_documented_in_the_help():
    """
    The help text is the only key reference a student has.

    A key bound in a table but missing from HELP is a feature nobody can
    find, which is the same as not shipping it. This test is the reason
    the two cannot drift apart.
    """
    bound = (set(DRIVE_KEYS) | set(JOG_KEYS) | set(GRIPPER_KEYS)
             | set(SPEED_KEYS) | {' ', '\t', 'h', 'q', '?'})
    missing = [key for key in bound
               if SPELLED_OUT.get(key, key) not in HELP]
    assert not missing, f'bound but undocumented: {missing}'


def test_every_documented_key_is_actually_bound():
    """The reverse drift: help promising a key that does nothing."""
    bound = (set(DRIVE_KEYS) | set(JOG_KEYS) | set(GRIPPER_KEYS)
             | set(SPEED_KEYS) | {' ', '\t', 'h', 'q', '?'})
    for key in bound:
        assert interpret(key).kind != 'unknown', key
