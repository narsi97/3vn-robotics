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
The parts, placed where the URDF says the joints are.

WHY THIS IS WORTH EXPORTING. Every check so far tests one number at a
time: a bore against a bushing, a beam against a joint spacing, a shaft
against an origin. All of them passed while the turret held its servo
the wrong way up, because no single measurement was wrong - the part was
simply the wrong shape, and that is something a person sees in a second
and a test does not see at all.

So this places every part at its joint position and writes one STEP
file. Opening it is the cheapest review the design will get before
filament is spent.

IT IS NOT A KINEMATIC MODEL. The URDF is, and it stays the source of
truth for where the joints are; this reads those origins and puts
plastic at them. If the two ever disagree, the URDF is right and the CAD
is wrong, because the URDF is what the controllers, the planner and
every test actually use.

Fasteners and servos are not drawn. They would make the file look more
finished and tell you nothing you cannot already read in PRINTING.md.
"""

from build123d import Compound, Location, Rotation

from threevn_cad import parts as parts_mod
from threevn_cad import profile as prof

#: Which printed part sits at which link, and how it has to be turned to
#: get there. The rotations are the difference between a part modelled
#: flat for printing and the same part in its working pose - a beam is
#: drawn standing up along Z, and on the robot it lies along the arm.
PLACEMENT = (
    # (link, part, extra rotation as (rx, ry, rz) degrees)
    ('base_link', 'base_plate', (0, 0, 0)),
    ('shoulder_link', 'shoulder_turret', (0, 0, 0)),
    ('upper_arm_link', 'upper_arm_link', (0, 0, 0)),
    ('forearm_link', 'forearm_link', (0, 0, 0)),
    ('wrist_link', 'wrist_bracket', (0, 0, 0)),
    ('gripper_base_link', 'gripper_base', (0, 0, 0)),
)

#: Joint that carries each link, for walking the chain. Taken from the
#: profile rather than hardcoded, so a re-parented arm still assembles.
def _chain(cfg):
    """Return [(link, cumulative_z_mm)] down the arm, from the profile."""
    joints = cfg['joints']
    order = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_joint')
    height = 0.0
    out = [('base_link', 0.0)]
    for name in order:
        joint = joints[name]
        height += prof.mm(joint['origin']['xyz'][2])
        out.append((joint['child'], height))
    # The gripper hangs off the wrist through a fixed frame.
    frames = cfg.get('frames', {})
    mount = frames.get('gripper_base_mount')
    if mount:
        out.append(('gripper_base_link',
                    height + prof.mm(mount['xyz'][2])))
    return out


def assemble(cfg, mechanism='direct'):
    """
    Return a Compound of every printed part at its joint position.

    The arm is built at its HOME pose - every joint at zero - because
    that is the configuration the URDF's origins describe directly and
    the one a reviewer can check against the numbers without doing any
    trigonometry.
    """
    heights = dict(_chain(cfg))
    available = parts_mod.parts_for(mechanism)

    placed = []
    for link, part_name, rotation in PLACEMENT:
        if part_name not in available or link not in heights:
            continue
        solid = available[part_name](cfg, mechanism=mechanism)
        located = Rotation(*rotation) * solid
        placed.append(Location((0, 0, heights[link])) * located)

    if not placed:
        raise ValueError('nothing was placed; the profile has no chain')
    return Compound(children=placed)


def stack_height(cfg):
    """Return the arm's total height at the home pose, in mm."""
    return max(height for _, height in _chain(cfg))
