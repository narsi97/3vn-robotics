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

WHAT IS AND IS NOT HERE. Every structural part of the arm is generated,
for both mechanisms: base, turret, shoulder bracket, both beams, wrist
and gripper. What is NOT here is the assembly - the rods for the linkage
variant, the bearing itself, the horn adapters and the fasteners. Those
are bought parts and joint details, and claiming them would be claiming
the arm has been built.
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


def _beam(cfg, link_name, mechanism, distal_servo=None):
    """
    A hollow rectangular beam between two joints.

    ONE function for both arm segments. They are the same part at
    different sizes, and writing them twice is how the two quietly stop
    agreeing about wall thickness or hole placement - the failure mode
    this session hit three times in other files.

    Its LENGTH is not a CAD decision: it comes from the profile the URDF
    reads. The cross-section is, and it is hollow, because a solid beam
    at this length is most of the arm's printed mass for stiffness the
    servo cannot exploit.

    `distal_servo` names the servo that drives the NEXT joint, which in
    a direct-drive arm mounts on this link's far end. Under a linkage
    that servo lives on the turret, so the far end gets a plain pivot
    and a rod anchor instead.
    """
    fab = prof.fabrication(cfg)
    link = prof.link_mm(cfg, link_name)
    wall = fab['min_wall_mm']

    if link['type'] != 'box':
        raise ValueError(
            f'{link_name} is not a box in the profile; this generator makes '
            f'a rectangular beam and would silently ignore a radius')

    thickness, width, length = link['x'], link['y'], link['z']
    bore = (fab['m3_hole_mm'] + fab['hole_clearance_mm']) / 2.0

    with BuildPart() as part:
        Box(thickness, width, length,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        # Hollow, leaving a wall all round and a closed end at each joint
        # where the screws land.
        with Locations((0, 0, wall)):
            Box(thickness - 2 * wall, width - 2 * wall, length - 2 * wall,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT)

        # Pivot bores at both ends, across the beam.
        for z in (wall / 2.0, length - wall / 2.0):
            with Locations(Plane.XZ.offset(-width / 2.0)):
                with Locations((0, z)):
                    Cylinder(radius=bore, height=width * 2,
                             align=(Align.CENTER, Align.CENTER, Align.CENTER),
                             mode=Mode.SUBTRACT)

        if mechanism == 'linkage':
            # Rod anchor near the proximal end. The offset from the
            # pivot IS the lever arm, and it sets how much of the
            # servo's travel reaches the joint.
            with Locations(Plane.XZ.offset(-width / 2.0)):
                with Locations((0, length * 0.18)):
                    Cylinder(radius=bore, height=width * 2,
                             align=(Align.CENTER, Align.CENTER, Align.CENTER),
                             mode=Mode.SUBTRACT)
        elif distal_servo is not None:
            # Direct drive: screw bosses at the far end for the servo
            # that drives the next joint.
            servo = prof.servo(cfg, distal_servo)
            for offset in (-servo['mount_hole_spacing_mm'] / 2.0,
                           servo['mount_hole_spacing_mm'] / 2.0):
                z = length - wall / 2.0 + offset * 0.0  # holes on the face
                with Locations(Plane.XY.offset(length)):
                    with Locations((0, offset * 0.25)):
                        Cylinder(radius=bore, height=wall * 4,
                                 align=(Align.CENTER, Align.CENTER,
                                        Align.CENTER),
                                 mode=Mode.SUBTRACT)

    return part.part


def upper_arm_link(cfg, mechanism='direct'):
    """The upper arm. Carries the elbow servo when directly driven."""
    return _beam(cfg, 'upper_arm_link', mechanism, distal_servo='mg996r')


def forearm_link(cfg, mechanism='direct'):
    """The forearm. Carries the wrist servo when directly driven."""
    return _beam(cfg, 'forearm_link', mechanism, distal_servo='sg90')


def base_plate(cfg, mechanism='direct'):
    """
    The base, and the only part that is the same for both mechanisms.

    The pan joint rotates the WHOLE arm about Z, and there is nothing to
    gain by driving that through a linkage: the servo is already at the
    bottom, which is the entire point of a linkage elsewhere. So the pan
    servo sits here either way.

    The raised boss around the shaft is the bearing seat. Running the
    turret directly on a servo horn puts the arm's whole overturning
    moment through the servo's output gear, which is how a hobby servo
    develops slop after an afternoon.
    """
    fab = prof.fabrication(cfg)
    base = prof.link_mm(cfg, 'base_link')
    servo = prof.servo(cfg, 'mg996r')
    wall = fab['min_wall_mm']

    if base['type'] != 'box':
        raise ValueError('base_link is expected to be a box')

    with BuildPart() as part:
        with BuildSketch(Plane.XY):
            RectangleRounded(base['x'], base['y'], radius=min(wall * 2, 4.0))
        extrude(amount=base['z'])

        # HOLLOW. A solid block this size prints at 343 g of PLA - more
        # than four times the entire moving arm - for stiffness a
        # bench-mounted base does not need. A shell with a floor is
        # stiff enough and prints in a fraction of the time.
        with Locations((0, 0, wall)):
            Box(base['x'] - 2 * wall, base['y'] - 2 * wall,
                base['z'] - wall,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT)

        # The pan servo lies on its side, shaft up, in a pocket that
        # opens downward so the horn reaches the turret above.
        with Locations((0, 0, wall)):
            insert(_servo_pocket(servo, fab), mode=Mode.SUBTRACT)

        # Bearing seat: an annular boss the turret sits on, concentric
        # with the shaft.
        seat = prof.link_mm(cfg, 'shoulder_link')
        outer = seat['radius'] + 2 * wall
        with Locations((0, 0, base['z'])):
            Cylinder(radius=outer, height=wall * 2,
                     align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((0, 0, base['z'])):
            Cylinder(radius=seat['radius'] + fab['hole_clearance_mm'],
                     height=wall * 4,
                     align=(Align.CENTER, Align.CENTER, Align.MIN),
                     mode=Mode.SUBTRACT)

        # Bolt-down holes at the corners, so the arm can be fixed to a
        # bench. An arm that can slide is an arm that will.
        inset = min(base['x'], base['y']) * 0.5 - fab['m3_head_mm']
        for sx in (-1, 1):
            for sy in (-1, 1):
                with Locations((sx * inset, sy * inset, 0)):
                    Cylinder(radius=(fab['m3_hole_mm']
                                     + fab['hole_clearance_mm']) / 2.0,
                             height=base['z'] * 3,
                             align=(Align.CENTER, Align.CENTER, Align.CENTER),
                             mode=Mode.SUBTRACT)

    return part.part


def shoulder_turret(cfg, mechanism='direct'):
    """
    The rotating turret between the pan joint and the upper arm.

    THIS IS WHERE THE TWO MECHANISMS DIVERGE MOST. Under direct drive it
    carries one servo, for the shoulder lift. Under a linkage it carries
    TWO - lift and elbow - because moving the elbow servo down here is
    the whole reason to accept a closed chain, and the elbow's rod runs
    from this part up past the shoulder.

    So the linkage turret is the heavier part and the linkage ARM is the
    lighter one, which is exactly the trade the decision turns on.
    """
    fab = prof.fabrication(cfg)
    turret = prof.link_mm(cfg, 'shoulder_link')
    servo = prof.servo(cfg, 'mg996r')
    wall = fab['min_wall_mm']

    mounted = 2 if mechanism == 'linkage' else 1
    # Long enough for however many servo flanges have to land on it.
    span = servo['mount_hole_spacing_mm'] + 2 * fab['m3_head_mm']
    width = mounted * (servo['width_mm'] + 2 * wall)
    height = turret['length'] + wall

    with BuildPart() as part:
        with BuildSketch(Plane.XY):
            RectangleRounded(span, width, radius=min(wall * 2, 3.0))
        extrude(amount=height)

        for index in range(mounted):
            offset = (index - (mounted - 1) / 2.0) * (servo['width_mm']
                                                      + 2 * wall)
            with Locations((0, offset, wall)):
                insert(_servo_pocket(servo, fab), mode=Mode.SUBTRACT)

        # The bore that drops over the base's bearing seat.
        Cylinder(radius=turret['radius'] + fab['hole_clearance_mm'],
                 height=wall * 3,
                 align=(Align.CENTER, Align.CENTER, Align.MIN),
                 mode=Mode.SUBTRACT)

    return part.part


def wrist_bracket(cfg, mechanism='direct'):
    """
    The wrist, between the forearm and the gripper.

    A short cylinder rather than a beam, because it carries no bending
    load worth speaking of - the gripper hangs off its end and the whole
    assembly is under fifty grams. Its job is to hold the SG90 and
    present a flat face for the gripper to bolt to.
    """
    fab = prof.fabrication(cfg)
    wrist = prof.link_mm(cfg, 'wrist_link')
    servo = prof.servo(cfg, 'sg90')
    wall = fab['min_wall_mm']

    if wrist['type'] != 'cylinder':
        raise ValueError('wrist_link is expected to be a cylinder')

    # Wide enough for the micro servo, whatever the nominal radius says:
    # the profile describes the COLLISION shape, and a pocket has to hold
    # a real part.
    radius = max(wrist['radius'], servo['width_mm'] / 2.0 + wall)

    with BuildPart() as part:
        Cylinder(radius=radius, height=wrist['length'],
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((0, 0, wall)):
            insert(_servo_pocket(servo, fab, depth=wrist['length'] - wall),
                   mode=Mode.SUBTRACT)
        # Pivot bore to the forearm.
        with Locations(Plane.XZ.offset(-radius)):
            with Locations((0, wall / 2.0)):
                Cylinder(radius=(fab['m3_hole_mm']
                                 + fab['hole_clearance_mm']) / 2.0,
                         height=radius * 3,
                         align=(Align.CENTER, Align.CENTER, Align.CENTER),
                         mode=Mode.SUBTRACT)

    return part.part


def gripper_base(cfg, mechanism='direct'):
    """
    The gripper body: one servo, two fingers, one of them a mimic.

    The fingers run in slots rather than on pivots because the URDF
    models them as PRISMATIC, and a printed part that pivots while the
    model slides is a part that disagrees with every grasp the planner
    computes.
    """
    fab = prof.fabrication(cfg)
    body = prof.link_mm(cfg, 'gripper_base_link')
    finger = prof.link_mm(cfg, 'gripper_left_finger_link')
    servo = prof.servo(cfg, 'sg90')
    wall = fab['min_wall_mm']

    with BuildPart() as part:
        with BuildSketch(Plane.XY):
            RectangleRounded(body['x'], body['y'], radius=min(wall, 2.0))
        extrude(amount=body['z'])

        with Locations((0, 0, wall)):
            insert(_servo_pocket(servo, fab, depth=body['z'] - wall),
                   mode=Mode.SUBTRACT)

        # Two slots for the fingers, one either side, sized to slide.
        slack = fab['hole_clearance_mm']
        for sign in (-1, 1):
            with Locations((0, sign * body['y'] / 4.0, body['z'])):
                Box(finger['x'] + 2 * slack, finger['y'] + 2 * slack,
                    wall * 3,
                    align=(Align.CENTER, Align.CENTER, Align.CENTER),
                    mode=Mode.SUBTRACT)

    return part.part


def gripper_finger(cfg, mechanism='direct'):
    """
    One finger. Printed twice.

    SYMMETRIC ON PURPOSE, so the same part serves both sides. The URDF
    mirrors them through a mimic joint with a -1.0 multiplier; a
    handed pair would mean two STLs, two chances to print the wrong one,
    and a left finger that fits only if flipped - which changes which
    face the layer lines run along.
    """
    fab = prof.fabrication(cfg)
    finger = prof.link_mm(cfg, 'gripper_left_finger_link')
    wall = fab['min_wall_mm']

    with BuildPart() as part:
        Box(finger['x'], finger['y'], finger['z'],
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        # A gripping face with a shallow step, so a cube does not simply
        # squeeze out of a pair of flat plates.
        with Locations((finger['x'] / 2.0, 0, finger['z'] * 0.75)):
            Box(wall, finger['y'] * 2, finger['z'] * 0.5,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
                mode=Mode.SUBTRACT)

    return part.part


#: Every part the generator can make, by name. Used by the exporter and
#: by the tests, so a part that is added and not exported is visible.
PARTS = {
    'base_plate': base_plate,
    'shoulder_turret': shoulder_turret,
    'shoulder_bracket': shoulder_bracket,
    'upper_arm_link': upper_arm_link,
    'forearm_link': forearm_link,
    'wrist_bracket': wrist_bracket,
    'gripper_base': gripper_base,
    'gripper_finger': gripper_finger,
}

MECHANISMS = ('direct', 'linkage')
