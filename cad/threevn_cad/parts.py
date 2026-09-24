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
The printed parts, as functions of the profile.

MECHANISM IS A PARAMETER. The direct-drive and parallel-linkage arms are
not two designs in two files; they are one generator called with
`mechanism='direct'` or `mechanism='linkage'`, so the comparison is
between two things built from the same numbers rather than between two
things someone drew on different days.

That choice is deliberately unmade. Direct drive keeps the URDF honest,
because an open chain is what URDF models exactly. A parallel linkage
distributes mass far better and gets more payload from the same servos,
but it is a CLOSED chain that URDF cannot express, so the simulation
would become an approximation of the robot - and every test, the FK
checks, the tipping analysis and the ML labels rest on the model being
faithful. Neither answer is obviously right, so both get printed.

WHAT IS AND IS NOT HERE. This generates the shoulder bracket and the
upper-arm link for both mechanisms: enough to print, hold a servo, and
compare. It does not generate a complete arm. The remaining links, the
wrist and the gripper are not designed, and the roadmap says so rather
than a comment here implying otherwise.
"""

from build123d import (
    Align,
    BuildPart,
    BuildSketch,
    Cylinder,
    Box,
    Locations,
    Mode,
    Plane,
    RectangleRounded,
    extrude,
    insert,
)

from threevn_cad import profile as prof


def _servo_pocket(servo, fab, depth=None):
    """
    Return the solid to SUBTRACT for a servo body.

    Oversized by the pocket clearance on every side. FDM prints
    dimensionally proud on internal features, so a pocket cut to the
    nominal body size does not accept the servo - and finding that out
    costs a print.
    """
    slack = fab['servo_pocket_clearance_mm']
    return Box(
        servo['length_mm'] + 2 * slack,
        servo['width_mm'] + 2 * slack,
        (depth if depth is not None else servo['height_mm']) + slack,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )


def shoulder_bracket(cfg, mechanism='direct'):
    """
    The part that carries the shoulder servo and the upper arm.

    This is where the two mechanisms genuinely differ, which is why it
    is the part chosen for the comparison:

    direct   the servo sits IN the bracket at the joint, so the bracket
             is a pocket with two bearing faces and the servo's mass is
             out on the arm.
    linkage  the servo sits at the BASE and drives the joint through a
             rod, so the bracket carries a plain pivot and a rod anchor
             instead - lighter at the joint, and one more thing to print
             and align.
    """
    if mechanism not in ('direct', 'linkage'):
        raise ValueError(
            f"mechanism must be 'direct' or 'linkage', not {mechanism!r}")

    fab = prof.fabrication(cfg)
    servo = prof.servo(cfg, 'mg996r')
    shoulder = prof.link_mm(cfg, 'shoulder_link')

    wall = fab['min_wall_mm']
    # Wide enough to wrap the servo with a wall either side.
    width = servo['width_mm'] + 2 * wall + 2 * fab['servo_pocket_clearance_mm']

    # LONG ENOUGH FOR THE MOUNTING FLANGE, which is the constraint that
    # actually sets this dimension. The screws sit 49.5 mm apart on an
    # MG996R - wider than the 40.7 mm body - so a bracket sized to the
    # BODY puts both holes outside the material. They then cut nothing,
    # the part still exports, and the mistake is invisible until someone
    # tries to screw the servo in.
    #
    # test_m3_holes_carry_the_printing_clearance caught exactly that:
    # the only cylindrical faces in the first version were the corner
    # rounds.
    flange_span = servo['mount_hole_spacing_mm'] + 2 * fab['m3_head_mm']
    depth = max(2.0 * shoulder.get('radius', 0.0), flange_span)
    height = servo['flange_height_mm'] + wall

    with BuildPart() as part:
        with BuildSketch(Plane.XY):
            RectangleRounded(depth, width, radius=min(wall * 2, 3.0))
        extrude(amount=height)

        if mechanism == 'direct':
            # The servo drops in from the top and its flange lands on
            # the rim, which is what actually carries the load - the
            # screws only stop it lifting out.
            with Locations((0, 0, wall)):
                insert(_servo_pocket(servo, fab), mode=Mode.SUBTRACT)
            # Mounting screws, on the datasheet spacing.
            for offset in (-servo['mount_hole_spacing_mm'] / 2.0,
                           servo['mount_hole_spacing_mm'] / 2.0):
                with Locations(Plane.XY.offset(height)):
                    with Locations((offset, 0)):
                        Cylinder(
                            radius=(fab['m3_hole_mm']
                                    + fab['hole_clearance_mm']) / 2.0,
                            height=height * 3,
                            align=(Align.CENTER, Align.CENTER, Align.CENTER),
                            mode=Mode.SUBTRACT)
        else:
            # A plain pivot: a through-bore for an M3 shoulder screw
            # running in the printed plastic, plus an anchor for the
            # push rod coming from the base servo.
            with Locations(Plane.YZ.offset(depth)):
                with Locations((0, height / 2.0)):
                    Cylinder(
                        radius=(fab['m3_hole_mm']
                                + fab['hole_clearance_mm']) / 2.0,
                        height=depth * 3,
                        align=(Align.CENTER, Align.CENTER, Align.CENTER),
                        mode=Mode.SUBTRACT)
            # Rod anchor, offset from the pivot: the offset IS the lever
            # arm, and it sets how much of the servo's travel reaches
            # the joint.
            with Locations(Plane.YZ.offset(depth)):
                with Locations((depth * 0.30, height / 2.0)):
                    Cylinder(
                        radius=(fab['m3_hole_mm']
                                + fab['hole_clearance_mm']) / 2.0,
                        height=depth * 3,
                        align=(Align.CENTER, Align.CENTER, Align.CENTER),
                        mode=Mode.SUBTRACT)

        # NO fillet() here. The vertical arrises are already rounded,
        # by the RectangleRounded the body is extruded from - and
        # filleting an edge that is itself a fillet raises
        # "There are no suitable edges for chamfer or fillet", which
        # reads like the selector found nothing rather than like it
        # found something already round.

    return part.part


def upper_arm_link(cfg, mechanism='direct'):
    """
    The upper arm, as a printed beam.

    Its LENGTH is not a CAD decision: it comes from the same
    `upper_arm_link` geometry the URDF reads, so changing the arm's
    reach in the profile changes both the simulation and the part.

    The cross-section is a CAD decision, and it is hollow. A solid beam
    at this length is most of the arm's printed mass for stiffness that
    the servo cannot exploit anyway.
    """
    fab = prof.fabrication(cfg)
    link = prof.link_mm(cfg, 'upper_arm_link')
    wall = fab['min_wall_mm']

    if link['type'] != 'box':
        raise ValueError(
            'upper_arm_link is not a box in the profile; this generator '
            'makes a rectangular beam and would silently ignore a radius')

    length, width, thickness = link['z'], link['y'], link['x']

    with BuildPart() as part:
        Box(thickness, width, length,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        # Hollow, leaving a wall all round and a closed end at each
        # joint where the screws land.
        with Locations((0, 0, wall)):
            Box(thickness - 2 * wall, width - 2 * wall, length - 2 * wall,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT)

        # Joint holes at both ends, on the long axis.
        for z in (wall / 2.0, length - wall / 2.0):
            with Locations(Plane.XZ.offset(-width / 2.0)):
                with Locations((0, z)):
                    Cylinder(
                        radius=(fab['m3_hole_mm']
                                + fab['hole_clearance_mm']) / 2.0,
                        height=width * 2,
                        align=(Align.CENTER, Align.CENTER, Align.CENTER),
                        mode=Mode.SUBTRACT)

    return part.part


#: Every part the generator can make, by name. Used by the exporter and
#: by the tests, so a part that is added and not exported is visible.
PARTS = {
    'shoulder_bracket': shoulder_bracket,
    'upper_arm_link': upper_arm_link,
}

MECHANISMS = ('direct', 'linkage')
