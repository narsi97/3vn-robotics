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

import functools
import math

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
    SlotCenterToCenter,
    extrude,
    insert,
)

from threevn_cad import fits
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
                            radius=fits.screw_bore_radius(cfg),
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
                        radius=fits.screw_bore_radius(cfg),
                        height=depth * 3,
                        align=(Align.CENTER, Align.CENTER, Align.CENTER),
                        mode=Mode.SUBTRACT)
            # Rod anchor, offset from the pivot: the offset IS the lever
            # arm, and it sets how much of the servo's travel reaches
            # the joint.
            with Locations(Plane.YZ.offset(depth)):
                with Locations((depth * 0.30, height / 2.0)):
                    Cylinder(
                        radius=fits.screw_bore_radius(cfg),
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
    # BUSHED. Sized to the bushing, not the screw - see fits.py for the
    # 2.4 mm interference that taught this.
    bore = fits.pivot_bore_radius(cfg, bushed=True)

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

    ORDER OF OPERATIONS IS LOAD BEARING HERE. The servo pocket is cut
    LAST, after every additive feature, so nothing can fill it back in.
    The bearing column was originally added afterwards and did exactly
    that - the part gained 66 g and quietly lost the cavity the servo
    goes in, while still passing every geometric check.

    The column itself is a TUBE around the servo, not a post through it:
    the arm's overturning moment is carried by the ring, and the servo
    occupies the middle. A solid column would need the space the servo
    needs.
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
        # bench-mounted base does not need.
        with Locations((0, 0, wall)):
            Box(base['x'] - 2 * wall, base['y'] - 2 * wall,
                base['z'] - wall,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT)

        # A tube up to the top, carrying the bearing seat. Without it
        # the seat is a disc spanning the hollow on a 1.2 mm rim, with
        # the whole arm's moment through it - which the printability
        # check found as 301 mm2 of unsupported ceiling. A face needing
        # support is usually a face with nothing under it.
        Cylinder(radius=fits.bearing_seat_outer_radius(cfg),
                 height=base['z'],
                 align=(Align.CENTER, Align.CENTER, Align.MIN))

        # The seat proper, standing above the top face.
        with Locations((0, 0, base['z'])):
            Cylinder(radius=fits.bearing_seat_outer_radius(cfg),
                     height=wall * 2,
                     align=(Align.CENTER, Align.CENTER, Align.MIN))

        # Bore the tube out so it is a ring rather than a post, and the
        # servo has somewhere to be.
        #
        # FROM THE FLOOR UP, not through everything. Cutting the full
        # height removed the base's own floor and left an 836 mm2
        # downward ledge at the floor line - which the printability
        # check flagged, and which was really the floor having a hole
        # punched in it.
        #
        # Wide enough for the servo's DIAGONAL: the body is 41.3 x 20.3,
        # so its corners sweep a 23.0 mm radius and a bore sized to the
        # seat's 22.35 mm would foul them.
        servo_diagonal = math.hypot(
            servo['length_mm'] + 2 * fab['servo_pocket_clearance_mm'],
            servo['width_mm'] + 2 * fab['servo_pocket_clearance_mm']) / 2.0
        with Locations((0, 0, wall)):
            Cylinder(radius=max(fits.bearing_seat_inner_radius(cfg),
                                servo_diagonal),
                     height=base['z'] * 2,
                     align=(Align.CENTER, Align.CENTER, Align.MIN),
                     mode=Mode.SUBTRACT)

        # Bolt-down holes at the corners: an arm that can slide is an
        # arm that will.
        inset = min(base['x'], base['y']) * 0.5 - fab['m3_head_mm']
        for sx in (-1, 1):
            for sy in (-1, 1):
                with Locations((sx * inset, sy * inset, 0)):
                    Cylinder(radius=fits.screw_bore_radius(cfg),
                             height=base['z'] * 3,
                             align=(Align.CENTER, Align.CENTER, Align.CENTER),
                             mode=Mode.SUBTRACT)

        # THE SERVO POCKET, LAST. Whatever else was added, the servo
        # still fits.
        with Locations((0, 0, wall)):
            insert(_servo_pocket(servo, fab), mode=Mode.SUBTRACT)

    return part.part


def shoulder_turret(cfg, mechanism='direct'):
    """
    The rotating turret between the pan joint and the upper arm.

    THE LIFT SERVO LIES ON ITS SIDE, and that is not cosmetic. The
    shoulder-lift axis is horizontal - the URDF says `axis: [0, 1, 0]` -
    so the servo's output shaft has to point along Y. The first version
    of this part stood the servo upright like the pan servo below it,
    which would have put the lift axis about 44 mm up instead of the
    20 mm the URDF places it at: an arm that assembles into a different
    robot than the one every test, trajectory and tipping margin was
    computed for.

    Nothing in a bracket's own geometry reveals that. It took comparing
    the printed stack against the joint origins, which is what
    test_the_printed_stack_matches_the_joint_origins does.

    Under direct drive the turret carries one servo. Under a linkage it
    carries TWO - lift and elbow - because moving the elbow servo down
    here is the whole reason to accept a closed chain.
    """
    fab = prof.fabrication(cfg)
    turret = prof.link_mm(cfg, 'shoulder_link')
    servo = prof.servo(cfg, 'mg996r')
    wall = fab['min_wall_mm']
    slack = fab['servo_pocket_clearance_mm']

    # Where the URDF puts the lift axis, relative to this part's base.
    lift_axis_z = prof.mm(
        cfg['joints']['shoulder_lift_joint']['origin']['xyz'][2])

    mounted = 2 if mechanism == 'linkage' else 1
    span = servo['mount_hole_spacing_mm'] + 2 * fab['m3_head_mm']
    # Lying down, the servo's WIDTH is its vertical dimension.
    lying_height = servo['width_mm'] + 2 * slack
    # The shaft sits on the servo's centreline in that direction, so the
    # body's floor has to be half a width below the axis.
    floor = lift_axis_z - lying_height / 2.0
    if floor < wall:
        raise ValueError(
            f'the lift axis at {lift_axis_z:.1f} mm is too low for a '
            f'{servo["width_mm"]:.1f} mm servo lying on its side; the body '
            f'would start {floor:.1f} mm above this part\'s base')

    width = max(mounted * (servo['height_mm'] + 2 * wall),
                2 * fits.turret_bore_radius(cfg) + 2 * wall)
    height = lift_axis_z + lying_height / 2.0 + wall

    with BuildPart() as part:
        with BuildSketch(Plane.XY):
            RectangleRounded(span, width, radius=min(wall * 2, 3.0))
        extrude(amount=height)

        # The bore that drops over the base's bearing seat. Derived from
        # the SEAT, not from the turret's nominal radius.
        #
        # It runs UP TO THE SERVO CAVITY rather than stopping short. A
        # blind bore leaves a 53 mm flat roof - far past what the
        # printer will bridge - and the only way to print it is upside
        # down, which then puts the servo cavity's floor in the air.
        # Meeting the cavity makes the turret open top to bottom and the
        # problem disappears. The servo hangs on its flange screws,
        # which is how it mounts in any case.
        #
        # FULL HEIGHT. Stopping at the cavity floor works for one servo,
        # whose cavity sits over the bore and consumes its roof. With
        # TWO cavities side by side the roof survives as a strip between
        # them - 226 mm2 spanning 27 mm, past what the printer bridges.
        # Taking the bore all the way up removes it.
        Cylinder(radius=fits.turret_bore_radius(cfg),
                 height=height + wall,
                 align=(Align.CENTER, Align.CENTER, Align.MIN),
                 mode=Mode.SUBTRACT)

        # Servo cavities, lying down: length along X, shaft along Y,
        # width vertical.
        for index in range(mounted):
            offset = (index - (mounted - 1) / 2.0) * (servo['height_mm']
                                                      + 2 * wall)
            # OPEN AT THE TOP. The servo drops in from above, which is
            # how it is fitted anyway - and a closed cavity leaves a
            # 0.7 mm ceiling spanning 20.3 mm, just past what the
            # printer will bridge, for no benefit.
            with Locations((0, offset, floor)):
                Box(servo['length_mm'] + 2 * slack,
                    servo['height_mm'] + 2 * slack,
                    height - floor + wall,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                    mode=Mode.SUBTRACT)

        # The shaft's own clearance, out through the side at axis height.
        with Locations(Plane.XZ.offset(-width / 2.0)):
            with Locations((0, lift_axis_z)):
                Cylinder(radius=(servo['horn_diameter_mm'] / 2.0
                                 + fab['hole_clearance_mm']),
                         height=width * 2,
                         align=(Align.CENTER, Align.CENTER, Align.CENTER),
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
                Cylinder(radius=fits.screw_bore_radius(cfg),
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


# ----------------------------------------------------------------------
# Assembly parts: what joins the structural parts to each other and to
# the servos. These are the pieces that decide whether a pile of
# correct-looking brackets becomes an arm.
# ----------------------------------------------------------------------

def horn_adapter(cfg, mechanism='direct', servo_name='mg996r'):
    """
    Bolts to the servo's supplied METAL horn and presents a flat face.

    THE SPLINE IS NOT PRINTED, and the arithmetic says why. A 25T spline
    on a 5.9 mm shaft has a 0.74 mm tooth pitch - 1.9 extrusions wide at
    a 0.4 mm nozzle. The tooth form is unresolvable at that scale, and a
    printed spline stripped by a servo that delivers 0.9 N.m is not a
    part, it is a consumable.

    Every hobby servo ships a metal horn with its own screw holes. This
    sits on that horn, takes its screws, and gives the driven part a
    flat bolted interface. The metal handles the torque; the plastic
    only has to locate.

    A recess receives the horn so the adapter sits flat on the servo's
    boss rather than perching on the horn's rim - the difference between
    a joint with a defined axis and one that rocks.
    """
    fab = prof.fabrication(cfg)
    servo = prof.servo(cfg, servo_name)
    wall = fab['min_wall_mm']

    for key in ('horn_screw_circle_mm', 'horn_screw_count',
                'horn_screw_hole_mm', 'horn_thickness_mm'):
        if key not in servo:
            raise KeyError(
                f'{servo_name} has no {key}. The adapter bolts to the horn, '
                f'so the horn interface has to be described.')

    circle = servo['horn_screw_circle_mm']
    outer = circle / 2.0 + fab['m3_head_mm']
    thickness = servo['horn_thickness_mm'] + 2 * wall

    with BuildPart() as part:
        Cylinder(radius=outer, height=thickness,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))

        # Recess for the metal horn, so the adapter seats on the servo
        # boss and not on the horn's edge.
        Cylinder(radius=circle / 2.0 + wall,
                 height=servo['horn_thickness_mm'] + fab['hole_clearance_mm'],
                 align=(Align.CENTER, Align.CENTER, Align.MIN),
                 mode=Mode.SUBTRACT)

        # Clearance for the shaft and its retaining screw.
        Cylinder(radius=(servo['horn_diameter_mm']
                         + 2 * fab['hole_clearance_mm']) / 2.0,
                 height=thickness * 3,
                 align=(Align.CENTER, Align.CENTER, Align.CENTER),
                 mode=Mode.SUBTRACT)

        # The horn's own screws, on its own circle.
        count = int(servo['horn_screw_count'])
        for index in range(count):
            angle = 2.0 * math.pi * index / count
            with Locations((circle / 2.0 * math.cos(angle),
                            circle / 2.0 * math.sin(angle), 0)):
                Cylinder(radius=(servo['horn_screw_hole_mm']
                                 + fab['hole_clearance_mm']) / 2.0,
                         height=thickness * 3,
                         align=(Align.CENTER, Align.CENTER, Align.CENTER),
                         mode=Mode.SUBTRACT)

        # M3 holes for the driven part, deliberately off the horn's
        # circle so the two sets of screws do not collide.
        driven = outer - fab['m3_head_mm'] / 2.0
        for index in range(2):
            angle = math.pi * index + math.pi / 4.0
            with Locations((driven * math.cos(angle),
                            driven * math.sin(angle), 0)):
                Cylinder(radius=fits.screw_bore_radius(cfg),
                         height=thickness * 3,
                         align=(Align.CENTER, Align.CENTER, Align.CENTER),
                         mode=Mode.SUBTRACT)

    return part.part


def push_rod(cfg, mechanism='direct'):
    """
    The rod that carries the elbow drive up to the forearm. LINKAGE ONLY.

    Its length is not free. A parallel linkage works because the rod and
    the link it parallels form a PARALLELOGRAM: equal and parallel
    sides, so the forearm holds its angle as the shoulder moves. That
    makes the rod the same length as the upper arm, taken from the same
    profile entry the URDF reads.

    Get the length wrong and the mechanism still moves - it just stops
    being a parallelogram, the forearm angle drifts with shoulder angle,
    and the arm's kinematics quietly stop matching any model of it.

    Printed flat and loaded in tension and compression along its length,
    which is the direction FDM layers are weakest in. It is deliberately
    thick for its job.
    """
    if mechanism != 'linkage':
        raise ValueError(
            'push_rod exists only for the linkage mechanism; direct drive '
            'puts a servo at the joint instead')

    fab = prof.fabrication(cfg)
    upper = prof.link_mm(cfg, 'upper_arm_link')
    wall = fab['min_wall_mm']

    length = upper['z']            # the parallelogram side
    # The rod ends are BUSHED like every other pivot, so they take
    # the bushing rather than the screw.
    bore = fits.pivot_bore_radius(cfg, bushed=True)
    width = bore * 2 + 2 * wall
    thickness = max(wall * 2, 4.0)

    with BuildPart() as part:
        with BuildSketch(Plane.XY):
            SlotCenterToCenter(length, width)
        extrude(amount=thickness)
        for x in (-length / 2.0, length / 2.0):
            with Locations((x, 0)):
                Cylinder(radius=bore, height=thickness * 3,
                         align=(Align.CENTER, Align.CENTER, Align.CENTER),
                         mode=Mode.SUBTRACT)

    return part.part


def pivot_bushing(cfg, mechanism='direct'):
    """
    A sleeve between an M3 screw and a printed hole.

    Printed plastic running directly on a steel screw wears, and it wears
    into an oval rather than staying round - so the joint develops slop
    in one direction only, which reads as a wobbly arm rather than as a
    worn bearing. A sacrificial sleeve is a part anyone can reprint.

    Sized so the screw slides and the outside is an interference fit in
    the pivot bore: the bushing is meant to stay put and let the screw
    turn.
    """
    fab = prof.fabrication(cfg)
    wall = fab['min_wall_mm']

    with BuildPart() as part:
        Cylinder(radius=fits.bushing_outer_radius(cfg), height=wall * 4,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
        Cylinder(radius=fits.bushing_inner_radius(cfg), height=wall * 12,
                 align=(Align.CENTER, Align.CENTER, Align.CENTER),
                 mode=Mode.SUBTRACT)
        # A flange, so it cannot push all the way through the bore.
        Cylinder(radius=fits.bushing_flange_radius(cfg), height=wall,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
        Cylinder(radius=fits.bushing_inner_radius(cfg), height=wall * 4,
                 align=(Align.CENTER, Align.CENTER, Align.MIN),
                 mode=Mode.SUBTRACT)

    return part.part


def thrust_washer(cfg, mechanism='direct'):
    """
    The flat washer the turret spins on, at the pan joint.

    NOT A BEARING. A real thrust bearing is a bought part and the BOM
    should carry one; this is what makes the joint work without it -
    a sacrificial disc between two printed faces, so the wear happens
    somewhere replaceable rather than on the base.

    Printed flat, which puts the layer lines perpendicular to the load
    and is the one orientation FDM is good at.
    """
    fab = prof.fabrication(cfg)
    turret = prof.link_mm(cfg, 'shoulder_link')
    wall = fab['min_wall_mm']

    if turret['type'] != 'cylinder':
        raise ValueError('shoulder_link is expected to be a cylinder')

    with BuildPart() as part:
        Cylinder(radius=fits.washer_outer_radius(cfg), height=wall,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
        Cylinder(radius=fits.washer_bore_radius(cfg), height=wall * 3,
                 align=(Align.CENTER, Align.CENTER, Align.CENTER),
                 mode=Mode.SUBTRACT)

    return part.part


#: Parts every arm needs, whatever the mechanism.
PARTS = {
    'base_plate': base_plate,
    'shoulder_turret': shoulder_turret,
    'shoulder_bracket': shoulder_bracket,
    'upper_arm_link': upper_arm_link,
    'forearm_link': forearm_link,
    'wrist_bracket': wrist_bracket,
    'gripper_base': gripper_base,
    'gripper_finger': gripper_finger,
    'horn_adapter_mg996r': functools.partial(horn_adapter,
                                             servo_name='mg996r'),
    'horn_adapter_sg90': functools.partial(horn_adapter,
                                           servo_name='sg90'),
    'pivot_bushing': pivot_bushing,
    'thrust_washer': thrust_washer,
}

#: Parts that exist only for one mechanism.
#:
#: The push rod is the linkage, in the same way the servo pocket is
#: direct drive. Listing it as a normal part and letting it raise for
#: the other mechanism would make "generate everything" fail for a
#: reason that is not an error.
MECHANISM_ONLY = {
    'linkage': {'push_rod': push_rod},
    'direct': {},
}

MECHANISMS = ('direct', 'linkage')


def parts_for(mechanism):
    """Return every part a given mechanism needs, by name."""
    if mechanism not in MECHANISMS:
        raise ValueError(
            f"mechanism must be one of {MECHANISMS}, not {mechanism!r}")
    return {**PARTS, **MECHANISM_ONLY[mechanism]}
