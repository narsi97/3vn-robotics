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
The parts that join the other parts.

These decide whether a pile of correct-looking brackets becomes an arm,
and most of them fail in ways a bracket cannot: a rod of the wrong
length still moves, a bushing that is a slip fit still assembles.
"""

import math

import pytest
from threevn_cad import parts as parts_mod
from threevn_cad import profile as prof


@pytest.fixture(scope='module')
def cfg():
    """The real robot profile."""
    return prof.load()


# -- the spline decision, kept checkable -------------------------------

@pytest.mark.parametrize('servo_name,teeth', [('mg996r', 25), ('sg90', 20)])
def test_the_spline_is_too_fine_to_print(cfg, servo_name, teeth):
    """
    THE ARITHMETIC BEHIND NOT PRINTING THE SPLINE.

    This is the reasoning that shaped the whole horn interface, so it
    lives in a test rather than a comment: if someone later fits a
    coarser spline or a finer nozzle, the number changes and the
    decision deserves revisiting.

    A tooth needs several extrusions across it to have a form at all.
    Under about three, the printer is drawing a wavy circle.
    """
    fab = prof.fabrication(cfg)
    servo = prof.servo(cfg, servo_name)

    pitch = math.pi * servo['horn_diameter_mm'] / teeth
    extrusions = pitch / fab['nozzle_mm']

    assert extrusions < 3.0, (
        f'{servo_name}: a {teeth}T spline on a '
        f'{servo["horn_diameter_mm"]} mm shaft is {extrusions:.1f} '
        f'extrusions per tooth, which is now printable - the adapter '
        f'could bolt to a printed spline instead of the metal horn'
    )


def test_the_adapter_clears_the_shaft_rather_than_gripping_it(cfg):
    """
    The centre bore is CLEARANCE, not a spline.

    If the adapter gripped the shaft it would be carrying torque through
    printed plastic, which is exactly what the horn exists to avoid.
    """
    from build123d import GeomType

    servo = prof.servo(cfg, 'mg996r')
    fab = prof.fabrication(cfg)
    part = parts_mod.horn_adapter(cfg, servo_name='mg996r')

    wanted = (servo['horn_diameter_mm'] + 2 * fab['hole_clearance_mm']) / 2.0
    radii = {round(f.radius, 3) for f in part.faces()
             if f.geom_type == GeomType.CYLINDER and f.radius is not None}
    assert any(r == pytest.approx(wanted, abs=0.02) for r in radii), (
        f'no shaft clearance bore at {wanted:.2f} mm; found {sorted(radii)}'
    )


def test_the_adapter_takes_the_horn_s_own_screws(cfg):
    """
    The load path is metal horn to screws to plastic, in that order.

    An adapter with no horn screws is a disc sitting on a shaft.
    """
    from build123d import GeomType

    servo = prof.servo(cfg, 'mg996r')
    fab = prof.fabrication(cfg)
    part = parts_mod.horn_adapter(cfg, servo_name='mg996r')

    wanted = (servo['horn_screw_hole_mm'] + fab['hole_clearance_mm']) / 2.0
    radii = [round(f.radius, 3) for f in part.faces()
             if f.geom_type == GeomType.CYLINDER and f.radius is not None]
    matching = [r for r in radii if r == pytest.approx(wanted, abs=0.02)]
    assert len(matching) >= servo['horn_screw_count'], (
        f'expected {servo["horn_screw_count"]} horn screw holes at '
        f'{wanted:.2f} mm, found {len(matching)}'
    )


@pytest.mark.parametrize('servo_name', ['mg996r', 'sg90'])
def test_an_adapter_needs_its_horn_described(cfg, servo_name):
    """A servo with no horn interface cannot have an adapter generated."""
    import copy

    stripped = copy.deepcopy(cfg)
    del stripped['servos'][servo_name]['body']['horn_screw_circle_mm']
    with pytest.raises(KeyError, match='horn_screw_circle_mm'):
        parts_mod.horn_adapter(stripped, servo_name=servo_name)


def test_the_horn_numbers_are_marked_estimated(cfg):
    """
    THE MOST LIKELY NUMBERS IN THIS FILE TO BE WRONG.

    Horns vary between manufacturers even for a servo sold as an
    MG996R. Marking them estimated is what tells someone to measure
    before printing rather than after.
    """
    for name in ('mg996r', 'sg90'):
        body = prof.servo(cfg, name)
        assert body.get('horn_provenance') == 'estimated', (
            f'{name}: horn dimensions claim better provenance than they '
            f'have; nobody has measured one'
        )


# -- the push rod -------------------------------------------------------

def test_the_push_rod_is_a_parallelogram_side(cfg):
    """
    ITS LENGTH IS NOT FREE.

    A parallel linkage works because the rod and the link it parallels
    form a parallelogram - equal and parallel sides - so the forearm
    holds its angle as the shoulder moves. Wrong length and the
    mechanism still moves; it just stops being a parallelogram, and the
    arm's kinematics quietly stop matching any model of it.
    """
    expected = prof.link_mm(cfg, 'upper_arm_link')['z']
    rod = parts_mod.push_rod(cfg, mechanism='linkage')
    box = rod.bounding_box()
    # Length plus the rounded ends of the slot.
    assert box.size.X > expected, 'the rod is shorter than the link it parallels'
    assert box.size.X < expected * 1.3, (
        f'the rod is {box.size.X:.0f} mm against a {expected:.0f} mm link; '
        f'that is not a parallelogram'
    )


def test_the_push_rod_pivots_are_a_link_length_apart(cfg):
    """The HOLES define the linkage, not the outline."""
    from build123d import GeomType

    expected = prof.link_mm(cfg, 'upper_arm_link')['z']
    from threevn_cad import fits

    # The rod ends are BUSHED, like every other pivot. Computing the
    # expected radius here from the screw size instead of asking fits.py
    # is the same mistake the parts made.
    bore_radius = fits.pivot_bore_radius(cfg, bushed=True)

    rod = parts_mod.push_rod(cfg, mechanism='linkage')
    # ONLY the bores. The slot's outer end caps are cylindrical too, and
    # a cylindrical face's centre lies on its SURFACE - so including
    # them measures the outline, which is a link length plus a rod
    # width, and looks like a rod 5 mm too long.
    bores = [f for f in rod.faces()
             if f.geom_type == GeomType.CYLINDER and f.radius is not None
             and abs(f.radius - bore_radius) < 0.02]
    centres = sorted({round(f.center().X, 2) for f in bores})
    assert len(centres) >= 2, 'the rod needs a pivot at each end'
    assert max(centres) - min(centres) == pytest.approx(expected, abs=0.5)


def test_asking_for_a_push_rod_on_a_direct_drive_arm_is_an_error(cfg):
    """
    It is the linkage, the way a servo pocket is direct drive.

    Returning something would put a rod in a box of parts that has
    nothing to connect it to.
    """
    with pytest.raises(ValueError, match='only for the linkage'):
        parts_mod.push_rod(cfg, mechanism='direct')


def test_the_rod_ships_only_with_the_linkage(cfg):
    """The registry agrees with the part."""
    assert 'push_rod' in parts_mod.parts_for('linkage')
    assert 'push_rod' not in parts_mod.parts_for('direct')


# -- bushings and washers ----------------------------------------------

def test_the_bushing_lets_the_screw_turn_and_stays_put_itself(cfg):
    """
    A SLIDING FIT INSIDE, AN INTERFERENCE FIT OUTSIDE.

    Backwards and the bushing spins in its bore while gripping the
    screw, which wears the bracket instead of the sacrificial part - the
    exact outcome it exists to prevent.
    """
    from build123d import GeomType

    fab = prof.fabrication(cfg)
    bushing = parts_mod.pivot_bushing(cfg)
    radii = sorted({round(f.radius, 3) for f in bushing.faces()
                    if f.geom_type == GeomType.CYLINDER
                    and f.radius is not None})

    bore = (fab['m3_hole_mm'] + fab['hole_clearance_mm']) / 2.0
    assert any(r == pytest.approx(bore, abs=0.02) for r in radii), (
        f'no M3 sliding bore at {bore:.2f} mm; found {radii}'
    )
    # The outside carries no clearance: it is pressed in.
    outer = max(radii)
    assert outer > bore + fab['min_wall_mm'] * 0.5


def test_the_thrust_washer_fits_over_the_bearing_seat(cfg):
    """
    It has to drop over the seat the base plate raises.

    A washer with a bore smaller than the seat does not go on, and one
    much larger does not stay centred.
    """
    turret = prof.link_mm(cfg, 'shoulder_link')
    fab = prof.fabrication(cfg)
    washer = parts_mod.thrust_washer(cfg)
    box = washer.bounding_box()

    assert box.size.X > 2 * turret['radius'], 'the washer is smaller than the seat'
    assert box.size.Z <= fab['min_wall_mm'] * 1.5, (
        'a thrust washer this thick is a spacer'
    )


def test_the_washer_is_called_a_washer_and_not_a_bearing(cfg):
    """
    IT IS NOT A BEARING, AND THE DOCSTRING SAYS SO.

    A real thrust bearing is a bought part. This is what makes the joint
    work without one, by putting the wear somewhere replaceable. A test
    on a docstring is unusual; the distinction is worth it, because
    "printed bearing" is how a design acquires a reputation for slop.
    """
    assert 'NOT A BEARING' in parts_mod.thrust_washer.__doc__


# -- every assembly part is still a printable solid --------------------

@pytest.mark.parametrize('name', ['horn_adapter_mg996r', 'horn_adapter_sg90',
                                  'pivot_bushing', 'thrust_washer'])
def test_the_assembly_parts_are_solids(cfg, name):
    """The same floor every other part has to clear."""
    part = parts_mod.parts_for('direct')[name](cfg, mechanism='direct')
    assert part.volume > 0.0
    assert part.is_valid
