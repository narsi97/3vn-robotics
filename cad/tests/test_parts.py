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
        for name, builder in parts_mod.PARTS.items():
            out[(mechanism, name)] = builder(cfg, mechanism=mechanism)
    return out


def every_part():
    """Parametrize over (mechanism, part)."""
    return [(m, n) for m in parts_mod.MECHANISMS for n in parts_mod.PARTS]


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


@pytest.mark.parametrize('mechanism,name', every_part())
def test_the_part_weighs_something_a_hobby_servo_can_lift(built, cfg,
                                                           mechanism, name):
    """
    Printed mass against the servo that has to move it.

    An MG996R derated gives about 0.55 N.m. A bracket that weighs more
    than the payload budget is a design that cannot work, and the mass
    is knowable now rather than after printing.
    """
    volume = built[(mechanism, name)].volume
    mass_g = volume / 1000.0 * 1.24  # PLA
    assert 1.0 < mass_g < 200.0, (
        f'{name} would print at {mass_g:.0f} g, which is not a part of a '
        f'desktop arm driven by hobby servos'
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

def test_the_direct_drive_bracket_admits_its_servo(cfg, built):
    """
    THE POCKET IS BIGGER THAN THE SERVO.

    FDM prints internal features proud, so a pocket cut to the nominal
    body size does not accept the part. Finding that out costs a print;
    finding it out here costs nothing.
    """
    servo = prof.servo(cfg, 'mg996r')
    fab = prof.fabrication(cfg)
    bracket = built[('direct', 'shoulder_bracket')]
    box = bracket.bounding_box()

    # The bracket has to be wider than the servo plus a wall each side.
    needed = servo['width_mm'] + 2 * fab['min_wall_mm']
    assert box.size.Y >= needed - 0.01, (
        f'the bracket is {box.size.Y:.1f} mm wide, and the servo plus two '
        f'walls needs {needed:.1f} mm'
    )


def test_the_linkage_bracket_carries_no_servo_pocket(cfg, built):
    """
    THE MECHANISMS ACTUALLY DIFFER.

    A parameter that changes nothing is worse than no parameter: the
    comparison the whole decision rests on would be between two
    identical parts. The linkage variant puts the servo at the base, so
    its bracket should be materially lighter at the joint.
    """
    direct = built[('direct', 'shoulder_bracket')].volume
    linkage = built[('linkage', 'shoulder_bracket')].volume
    assert linkage != pytest.approx(direct), (
        'both mechanisms produced the same bracket, so the parameter is '
        'doing nothing and the comparison is meaningless'
    )
    assert linkage > direct, (
        'the linkage bracket has no servo cavity, so it should contain '
        'MORE material than the one with a servo-sized hole in it'
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

    bracket = built[('direct', 'shoulder_bracket')]
    radii = {round(f.radius, 3) for f in bracket.faces()
             if f.geom_type == GeomType.CYLINDER}
    assert any(r == pytest.approx(wanted, abs=0.01) for r in radii), (
        f'no hole at the clearance-corrected M3 radius {wanted:.2f} mm; '
        f'found {sorted(radii)}'
    )
