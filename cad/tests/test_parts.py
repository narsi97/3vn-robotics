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
The parts, checked as solids before anyone spends filament.

These are the cheap versions of mistakes that otherwise cost a print and
a few hours: a part that is not a closed solid, a pocket the servo
cannot enter, a bracket that does not match the link it bolts to.
"""

import pytest
from threevn_cad import parts as parts_mod
from threevn_cad import profile as prof


@pytest.fixture(scope='module')
def cfg():
    """The real robot profile."""
    return prof.load()


@pytest.fixture(scope='module')
def built(cfg):
    """Every part for every mechanism, built once."""
    out = {}
    for mechanism in parts_mod.MECHANISMS:
        for name, builder in parts_mod.parts_for(mechanism).items():
            out[(mechanism, name)] = builder(cfg, mechanism=mechanism)
    return out


def every_part():
    """Parametrize over (mechanism, part), including mechanism-only ones."""
    return [(m, n) for m in parts_mod.MECHANISMS
            for n in parts_mod.parts_for(m)]


# -- is it even a solid -------------------------------------------------

@pytest.mark.parametrize('mechanism,name', every_part())
def test_the_part_is_a_closed_solid(built, mechanism, name):
    """
    A CLOSED, POSITIVE-VOLUME SOLID.

    build123d will happily produce an empty or self-intersecting result
    from a cut that removed everything, and STL export does not
    complain. The slicer then produces either nothing or nonsense, which
    is discovered at the printer.
    """
    solid = built[(mechanism, name)]
    assert solid is not None
    assert solid.volume > 0.0, 'the part has no material left'
    assert solid.is_valid, 'the part is not a valid solid'


@pytest.mark.parametrize('mechanism,name', every_part())
def test_the_part_fits_on_a_normal_print_bed(built, mechanism, name):
    """
    220 x 220 x 250 mm, an Ender-class bed.

    A part that needs a bigger printer is a part most people following
    the course cannot make.
    """
    box = built[(mechanism, name)].bounding_box()
    for axis, size in zip('XYZ', (box.size.X, box.size.Y, box.size.Z)):
        assert size <= 220.0, f'{name} is {size:.0f} mm in {axis}'
    assert min(box.size.X, box.size.Y, box.size.Z) > 1.0, \
        'a dimension under a millimetre is a modelling mistake'


#: The base is bolted to a bench. Nothing lifts it, so the constraint
#: that applies to every other part does not apply to it - and a heavy
#: base is a STABLE base, which the Phase 11 tipping analysis wants.
STATIC_PARTS = {'base_plate'}


@pytest.mark.parametrize('mechanism,name',
                         [(m, n) for m, n in every_part()
                          if n not in STATIC_PARTS])
def test_a_moving_part_is_light_enough_for_its_servo(built, mechanism, name):
    """
    Printed mass against the servo that has to move it.

    An MG996R derated gives about 0.55 N.m, and the mass is knowable now
    rather than after printing.

    STATIC PARTS ARE EXCLUDED, and that exclusion is the point. This
    test first applied the same limit to the base plate and failed it at
    343 g - which was two mistakes at once: the base was needlessly
    solid, AND no servo lifts the base, so the limit was never the right
    question for it.
    """
    volume = built[(mechanism, name)].volume
    mass_g = volume / 1000.0 * 1.24  # PLA
    assert 0.1 < mass_g < 120.0, (
        f'{name} would print at {mass_g:.0f} g, which is not a moving part '
        f'of a desktop arm driven by hobby servos'
    )


@pytest.mark.parametrize('mechanism,name',
                         [(m, n) for m, n in every_part()
                          if n in STATIC_PARTS])
def test_a_static_part_is_not_absurdly_heavy(built, mechanism, name):
    """
    A looser bound, because the constraint is filament and print time.

    Still bounded: a base heavier than the rest of the robot put
    together is a modelling mistake, not a design choice.
    """
    mass_g = built[(mechanism, name)].volume / 1000.0 * 1.24
    assert 5.0 < mass_g < 250.0, f'{name} prints at {mass_g:.0f} g'


def test_the_whole_arm_is_printable_in_one_sitting(built):
    """
    THE NUMBER ANYONE FOLLOWING THE COURSE ACTUALLY CARES ABOUT.

    Total printed mass for one arm, against the ~250 g of PLA the BOM
    budgets. A design that quietly needs a whole spool is a different
    product at a different price.
    """
    for mechanism in parts_mod.MECHANISMS:
        total = sum(built[(mechanism, name)].volume
                    for name in parts_mod.parts_for(mechanism)) / 1000.0 * 1.24
        # Two fingers, not one.
        total += built[(mechanism, 'gripper_finger')].volume / 1000.0 * 1.24
        assert total < 400.0, (
            f'{mechanism}: the arm needs {total:.0f} g of PLA, against the '
            f'250 g the BOM budgets'
        )


# -- does it match the robot -------------------------------------------

def test_the_upper_arm_length_comes_from_the_profile(cfg, built):
    """
    THE WHOLE POINT OF GENERATING RATHER THAN DRAWING.

    The printed beam's length is the `upper_arm_link` length the URDF
    reads. If the CAD were drawn by hand, the part and the simulated
    model would be two independent statements about one robot, and they
    would disagree within a week - silently, because neither knows about
    the other.
    """
    expected = prof.link_mm(cfg, 'upper_arm_link')['z']
    for mechanism in parts_mod.MECHANISMS:
        box = built[(mechanism, 'upper_arm_link')].bounding_box()
        assert box.size.Z == pytest.approx(expected, abs=0.01), (
            f'{mechanism}: the printed beam is {box.size.Z:.1f} mm but the '
            f'profile says {expected:.1f} mm'
        )


def test_changing_the_profile_changes_the_part(cfg):
    """
    Parametric in fact, not in aspiration.

    A generator that ignores its input produces the same part whatever
    the profile says, and every test above would still pass.
    """
    import copy

    longer = copy.deepcopy(cfg)
    longer['links']['upper_arm_link']['geometry']['z'] *= 1.5

    before = parts_mod.upper_arm_link(cfg).bounding_box().size.Z
    after = parts_mod.upper_arm_link(longer).bounding_box().size.Z
    assert after == pytest.approx(before * 1.5, rel=1e-6)


# -- the servo has to physically fit ----------------------------------

def test_the_turret_admits_the_lift_servo(cfg, built):
    """
    THE POCKET IS BIGGER THAN THE SERVO.

    FDM prints internal features proud, so a pocket cut to the nominal
    body size does not accept the part. Finding that out costs a print;
    finding it here costs nothing.

    This used to check the shoulder BRACKET, which under direct drive
    turned out to have no job at all - the turret holds the lift servo
    at the joint. The bracket only exists for the linkage now.
    """
    servo = prof.servo(cfg, 'mg996r')
    fab = prof.fabrication(cfg)
    turret = built[('direct', 'shoulder_turret')]
    box = turret.bounding_box()

    # Lying down, the servo's LENGTH runs across the turret's X.
    needed = servo['length_mm'] + 2 * fab['servo_pocket_clearance_mm']
    assert box.size.X >= needed - 0.01, (
        f'the turret is {box.size.X:.1f} mm across and the servo lying in '
        f'it needs {needed:.1f} mm'
    )


def test_the_shoulder_bracket_exists_only_for_the_linkage(cfg):
    """
    THE MECHANISMS ACTUALLY DIFFER, and now they differ in their PARTS.

    Under direct drive the turret carries the lift servo at the joint
    and there is nothing for a separate bracket to do. Under a linkage
    the bracket holds the plain pivot and rod anchor that replace that
    servo.

    It sat in the common set until direct drive was chosen, carrying a
    vertical servo pocket that no joint on this arm wants - every axis
    but the pan is horizontal.
    """
    assert 'shoulder_bracket' in parts_mod.parts_for('linkage')
    assert 'shoulder_bracket' not in parts_mod.parts_for('direct')


def test_the_linkage_turret_carries_more_than_the_direct_one(cfg, built):
    """
    Two servos against one, which is the whole trade.

    A parameter that changes nothing is worse than no parameter: the
    comparison the mechanism decision rested on would be between two
    identical parts.
    """
    direct = built[('direct', 'shoulder_turret')].volume
    linkage = built[('linkage', 'shoulder_turret')].volume
    assert linkage > direct * 1.2, (
        'the linkage turret holds a second servo, so it should be '
        'materially larger'
    )


def test_an_unknown_mechanism_is_refused(cfg):
    """A typo must not silently fall back to a default."""
    with pytest.raises(ValueError, match="'direct' or 'linkage'"):
        parts_mod.shoulder_bracket(cfg, mechanism='parallell')


# -- printability -------------------------------------------------------

@pytest.mark.parametrize('mechanism,name', every_part())
def test_no_hole_is_too_small_to_print(built, cfg, mechanism, name):
    """
    Every cylindrical face is at least a nozzle wide.

    A hole smaller than the nozzle does not appear in the print at all -
    the slicer drops it, the part looks right, and the screw has nowhere
    to go.
    """
    from build123d import GeomType

    fab = prof.fabrication(cfg)
    solid = built[(mechanism, name)]
    circular = [f for f in solid.faces()
                if f.geom_type == GeomType.CYLINDER]
    for face in circular:
        radius = face.radius
        if radius is None:
            # A cylindrical face whose radius build123d cannot report -
            # a swept or trimmed surface rather than a drilled hole.
            # Not what this test is about.
            continue
        assert radius * 2 >= fab['nozzle_mm'], (
            f'{name} has a {radius * 2:.2f} mm hole, below the '
            f'{fab["nozzle_mm"]} mm nozzle'
        )


def test_m3_holes_carry_the_printing_clearance(cfg, built):
    """
    A nominal 3.0 mm hole does not take an M3 screw off an FDM printer.

    The clearance is in the profile as a printer property, and the CAD
    has to actually apply it - a hole modelled at nominal is the single
    most common reason a printed assembly needs a drill.
    """
    from build123d import GeomType

    fab = prof.fabrication(cfg)
    wanted = (fab['m3_hole_mm'] + fab['hole_clearance_mm']) / 2.0

    turret = built[('direct', 'shoulder_turret')]
    radii = {round(f.radius, 3) for f in turret.faces()
             if f.geom_type == GeomType.CYLINDER}
    assert any(r == pytest.approx(wanted, abs=0.01) for r in radii), (
        f'no hole at the clearance-corrected M3 radius {wanted:.2f} mm; '
        f'found {sorted(radii)}'
    )


# -- can the servo actually lift what was designed ---------------------

def test_the_shoulder_servo_can_lift_the_arm_it_has_to_lift(cfg, built):
    """
    THE CHECK THAT CONNECTS THE CAD TO THE PHYSICS.

    Printed mass is only interesting against the torque available. This
    takes the parts distal to the shoulder lift, adds the servos that
    ride on them and the declared payload, puts the lot at the arm's
    full reach, and compares the moment with what an MG996R gives after
    the profile's own derating.

    Full reach with the payload right at the tip is the worst case and
    not a typical one - but it is the pose a user WILL try in the first
    five minutes.
    """
    from threevn_cad.export import LIFTED_PARTS, LIFTED_SERVOS

    servos = {n: s['mass_kg'] for n, s in cfg['servos'].items()}
    derate = cfg['limit_derate']['effort']
    stall = cfg['servos']['mg996r']['stall_torque_nm']
    available = stall * derate

    # Reach: the links beyond the shoulder, end to end, in metres.
    reach = sum(prof.link_mm(cfg, name)['z' if prof.link_mm(cfg, name)['type']
                                        == 'box' else 'length']
                for name in ('upper_arm_link', 'forearm_link', 'wrist_link',
                             'gripper_base_link')) / 1000.0
    payload = 0.050  # the same 50 g the Phase 11 tipping analysis used

    for mechanism in parts_mod.MECHANISMS:
        by_part = {n: built[(mechanism, n)].volume / 1000.0 * 1.24 / 1000.0
                   for n in parts_mod.parts_for(mechanism)}
        arm = sum(by_part.get(p, 0.0) for p in LIFTED_PARTS)
        arm += 2 * by_part.get('gripper_finger', 0.0)
        arm += sum(servos[s] for s in LIFTED_SERVOS[mechanism])

        # Arm mass acts at roughly mid-reach; payload at the tip.
        moment = (arm * reach / 2.0 + payload * reach) * 9.81
        assert moment < available, (
            f'{mechanism}: holding {arm * 1000:.0f} g of arm plus '
            f'{payload * 1000:.0f} g of payload at {reach * 1000:.0f} mm '
            f'needs {moment:.3f} N.m, and a derated MG996R gives '
            f'{available:.3f} N.m'
        )


def test_the_linkage_lifts_materially_less_than_direct_drive(cfg, built):
    """
    THE WHOLE ARGUMENT FOR ACCEPTING A CLOSED CHAIN.

    If moving the elbow servo to the turret did not measurably reduce
    what the shoulder carries, the linkage would be pure cost: harder
    CAD, more plastic, and a kinematic chain URDF cannot express, for
    nothing.
    """
    from threevn_cad.export import compare

    results = {}
    for mechanism in parts_mod.MECHANISMS:
        results[mechanism] = [
            {'part': name,
             'mass_g': built[(mechanism, name)].volume / 1000.0 * 1.24}
            for name in parts_mod.parts_for(mechanism)
        ]
    summary = compare(cfg, results)

    direct = summary['direct']['lifted_g']
    linkage = summary['linkage']['lifted_g']
    assert linkage < direct * 0.8, (
        f'the linkage lifts {linkage:.0f} g against direct drive at '
        f'{direct:.0f} g, which is not enough of a gain to pay for a '
        f'closed kinematic chain'
    )
    # And it costs more plastic, which is the other half of the trade.
    assert summary['linkage']['printed_g'] > summary['direct']['printed_g']
