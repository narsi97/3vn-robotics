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
Does the thing fit in the hole.

THE CHECKS THAT WOULD HAVE CAUGHT THE WORST BUGS IN THIS DESIGN. Two
joints were sized independently, both looked reasonable, and neither
fitted:

    beam pivot bore   3.35 mm   bushing outside   5.75 mm
    turret bore      44.70 mm   bearing seat OD  48.80 mm

Neither is a hard error. Both parts exported, passed every geometric
check, looked correct, and would have been discovered with the parts in
hand. The cause was sizing each hole against a nominal feature - a
screw, a link radius - instead of against the part that goes into it.

Every check below compares a hole against the actual mating part, using
the same function both sides of the joint use.
"""

import pytest
from build123d import GeomType
from threevn_cad import fits
from threevn_cad import parts as parts_mod
from threevn_cad import profile as prof


@pytest.fixture(scope='module')
def cfg():
    """The real robot profile."""
    return prof.load()


def radii_of(solid):
    """Every reportable cylindrical radius in a solid."""
    return sorted({round(f.radius, 3) for f in solid.faces()
                   if f.geom_type == GeomType.CYLINDER
                   and f.radius is not None})


def has_radius(solid, wanted, tol=0.02):
    """Whether a solid has a cylindrical face at a given radius."""
    return any(abs(r - wanted) < tol for r in radii_of(solid))


# -- the pairs, from the shared table ----------------------------------

@pytest.mark.parametrize('hole,plug,label',
                         [(h, p, l) for h, p, l in fits.MATING_PAIRS
                          if p is not None])
def test_the_plug_enters_the_hole(cfg, hole, plug, label):
    """
    EVERY FIT IN THE ASSEMBLY, WALKED FROM ONE TABLE.

    Parametrizing over the table rather than writing each pair by hand
    is deliberate: the failure here was never a wrong formula, it was
    two formulas nobody compared. A new joint added to MATING_PAIRS is
    checked automatically; one added without going in the table is the
    bug this file exists to prevent, which is why `fits.py` keeps the
    table beside the functions.
    """
    hole_r = hole(cfg)
    plug_r = plug(cfg)
    assert hole_r >= plug_r, (
        f'{label}: the hole is {hole_r * 2:.2f} mm and the part is '
        f'{plug_r * 2:.2f} mm - oversize by {(plug_r - hole_r) * 2:.2f} mm'
    )


@pytest.mark.parametrize('hole,plug,label',
                         [(h, p, l) for h, p, l in fits.MATING_PAIRS
                          if p is not None])
def test_the_fit_is_not_so_loose_it_rattles(cfg, hole, plug, label):
    """
    Clearance, not a gap.

    A pivot with half a millimetre of play is a joint with backlash, and
    backlash at the shoulder is amplified by the whole arm. Anything
    beyond the printer's own clearance allowance is a mistake in the
    other direction.
    """
    fab = prof.fabrication(cfg)
    slack = (hole(cfg) - plug(cfg)) * 2
    assert slack <= fab['hole_clearance_mm'] * 2 + 0.01, (
        f'{label}: {slack:.2f} mm of play, which is more than the '
        f'{fab["hole_clearance_mm"]:.2f} mm the printer needs'
    )


# -- the specific bugs, pinned -----------------------------------------

def test_a_bushing_fits_the_pivot_bore_of_every_bushed_part(cfg):
    """
    THE 2.4 MM BUG.

    The bushing's outside is two wall thicknesses larger than the screw
    it carries. A pivot sized to the screw cannot admit it - and the
    beams, brackets and rod all have pivots.
    """
    outer = fits.bushing_outer_radius(cfg)
    bore = fits.pivot_bore_radius(cfg, bushed=True)
    assert bore >= outer

    for mechanism in parts_mod.MECHANISMS:
        for name in ('upper_arm_link', 'forearm_link'):
            part = parts_mod.parts_for(mechanism)[name](
                cfg, mechanism=mechanism)
            assert has_radius(part, bore), (
                f'{mechanism}/{name}: no bushed pivot bore at '
                f'{bore * 2:.2f} mm; found {radii_of(part)}'
            )


def test_the_turret_drops_over_the_bearing_seat(cfg):
    """
    THE 4.1 MM BUG.

    The seat is a BOSS around the turret's nominal radius, not the
    radius itself, so a bore sized to the nominal radius interferes with
    it by two wall thicknesses.
    """
    seat = fits.bearing_seat_outer_radius(cfg)
    bore = fits.turret_bore_radius(cfg)
    assert bore >= seat

    base = parts_mod.base_plate(cfg)
    turret = parts_mod.shoulder_turret(cfg)
    assert has_radius(base, seat), 'the base has no bearing seat'
    assert has_radius(turret, bore), 'the turret has no bore for it'


def test_the_thrust_washer_goes_on_the_same_seat(cfg):
    """
    It sits between the base and the turret, so it fits the same boss.

    A washer sized to a different reference is a washer that only goes
    on one of the two parts it separates.
    """
    washer = parts_mod.thrust_washer(cfg)
    assert has_radius(washer, fits.washer_bore_radius(cfg))
    assert fits.washer_bore_radius(cfg) >= fits.bearing_seat_outer_radius(cfg)


def test_the_washer_is_thinner_than_the_seat_is_tall(cfg):
    """
    Otherwise it holds the turret off its own bore and the joint rocks.

    The seat has to still engage the turret with the washer in place.
    """
    fab = prof.fabrication(cfg)
    seat_height = fab['min_wall_mm'] * 2
    washer = parts_mod.thrust_washer(cfg)
    assert washer.bounding_box().size.Z < seat_height


def test_an_m3_screw_slides_through_a_bushing(cfg):
    """The one fit that carries motion rather than locating a part."""
    fab = prof.fabrication(cfg)
    bore = fits.bushing_inner_radius(cfg) * 2
    assert bore > fab['m3_hole_mm'], 'an M3 will not turn in this'
    assert bore < fab['m3_hole_mm'] + 1.0, 'that is not a bearing, it is a hole'


# -- the structural invariant -----------------------------------------

def test_no_part_computes_a_mating_dimension_for_itself(cfg):
    """
    THE RULE THAT PREVENTS THE WHOLE CLASS OF BUG.

    Mating dimensions come from fits.py, so a hole and the thing that
    enters it cannot disagree - only one of them is a decision. A part
    that derives an M3 bore from the fabrication constants itself has
    stepped outside that, and the next joint added will be sized
    independently again.
    """
    import pathlib

    source = (pathlib.Path(parts_mod.__file__)).read_text()
    assert "fab['m3_hole_mm']" not in source, (
        'parts.py computes an M3 dimension directly instead of calling '
        'fits.py, which is how the 2.4 mm and 4.1 mm interferences '
        'happened'
    )


# -- does the printed stack build the robot the URDF describes ---------

def test_the_lift_axis_sits_where_the_urdf_puts_it(cfg):
    """
    THE CHECK THAT CAUGHT A TURRET HOLDING ITS SERVO THE WRONG WAY UP.

    The shoulder-lift axis is horizontal - the URDF says
    `axis: [0, 1, 0]` - so the servo's shaft must point along Y, which
    means the servo lies on its side. The first turret stood it upright
    like the pan servo below it, putting the lift axis about 44 mm up
    instead of the 20 mm the URDF places it at.

    Nothing in the bracket's own geometry reveals that. It looks like a
    perfectly good bracket. It just assembles into a different robot
    than the one every trajectory, FK test and tipping margin was
    computed for.
    """
    from build123d import GeomType

    servo = prof.servo(cfg, 'mg996r')
    fab = prof.fabrication(cfg)
    lift_z = cfg['joints']['shoulder_lift_joint']['origin']['xyz'][2] * 1000.0

    for mechanism in parts_mod.MECHANISMS:
        turret = parts_mod.shoulder_turret(cfg, mechanism=mechanism)
        wanted = servo['horn_diameter_mm'] / 2.0 + fab['hole_clearance_mm']
        shafts = [f for f in turret.faces()
                  if f.geom_type == GeomType.CYLINDER
                  and f.radius is not None
                  and abs(f.radius - wanted) < 0.05]
        assert shafts, (
            f'{mechanism}: the turret has no shaft clearance at '
            f'{wanted * 2:.1f} mm, so the lift servo has nowhere to point'
        )
        heights = [f.center().Z for f in shafts]
        assert min(heights) == pytest.approx(lift_z, abs=0.5), (
            f'{mechanism}: the lift shaft is at {min(heights):.1f} mm and '
            f'the URDF puts the joint at {lift_z:.1f} mm'
        )


def test_the_turret_is_tall_enough_for_a_servo_lying_down(cfg):
    """
    The axis height has to clear half a servo width plus a floor.

    Asked for an axis lower than that, the generator raises rather than
    producing a part with the servo poking out of the bottom.
    """
    import copy

    low = copy.deepcopy(cfg)
    low['joints']['shoulder_lift_joint']['origin']['xyz'][2] = 0.002
    with pytest.raises(ValueError, match='too low'):
        parts_mod.shoulder_turret(low)


def test_the_arm_beams_match_their_joint_spacing(cfg):
    """
    A beam's length IS the distance to the next joint.

    The elbow sits 110 mm along the upper arm and the wrist 95 mm along
    the forearm, so a beam of any other length moves a joint.
    """
    for link, joint in (('upper_arm_link', 'elbow_joint'),
                        ('forearm_link', 'wrist_joint')):
        spacing = cfg['joints'][joint]['origin']['xyz'][2] * 1000.0
        for mechanism in parts_mod.MECHANISMS:
            beam = parts_mod.parts_for(mechanism)[link](
                cfg, mechanism=mechanism)
            assert beam.bounding_box().size.Z == pytest.approx(
                spacing, abs=0.01), f'{mechanism}/{link}'


def test_the_turret_clears_the_base_it_sits_on(cfg):
    """
    The turret's footprint has to cover the bearing seat it drops over.

    A turret narrower than its own bore is a part with a hole in its
    edge rather than a bore through its middle.
    """
    turret = parts_mod.shoulder_turret(cfg)
    box = turret.bounding_box()
    needed = fits.turret_bore_radius(cfg) * 2
    assert min(box.size.X, box.size.Y) >= needed, (
        f'the turret is {min(box.size.X, box.size.Y):.1f} mm across and its '
        f'bore is {needed:.1f} mm'
    )
