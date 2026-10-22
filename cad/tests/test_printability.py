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
Whether the parts come off a printer in one piece.

`max_overhang_deg` sat in the profile unused while the CAD was written,
which made it a number describing an intention rather than a constraint.

The check itself was wrong twice before it was right, and both mistakes
produced reassuring output: the first reported zero unsupported area for
every part because its comparison was inverted, and the second flagged
both arm beams because it measured the bridge across a cavity's LENGTH
instead of its width. A printability check that says "all clear" while
doing nothing is worse than no check.
"""

import math

import numpy as np
import pytest
from threevn_cad import parts as parts_mod
from threevn_cad import printability as pr
from threevn_cad import profile as prof


@pytest.fixture(scope='module')
def cfg():
    """The real robot profile."""
    return prof.load()


@pytest.fixture(scope='module')
def limits(cfg):
    """The printer's overhang and bridging limits."""
    fab = prof.fabrication(cfg)
    return fab['max_overhang_deg'], fab['max_bridge_mm']


# -- the check is not vacuous ------------------------------------------

def test_a_horizontal_ceiling_is_detected():
    """
    THE CASE THE FIRST VERSION MISSED ENTIRELY.

    A downward-facing horizontal face is a 0-degree overhang and the
    worst case there is. The original condition skipped faces steeper
    than the limit, which are exactly these, and reported a clean bill
    of health for the whole design.
    """
    up = np.array([0.0, 0.0, 1.0])
    assert pr.overhang_angle(np.array([0.0, 0.0, -1.0]), up) == \
        pytest.approx(0.0)


def test_a_vertical_wall_is_not_an_overhang():
    """
    None, not 90 - it does not face downward at all.

    The boundary is worth pinning. A vertical wall is the limiting case
    between "supports itself" and "is not an overhang", and either
    answer is defensible; the function returns None, so anything
    summing areas skips it rather than counting a 90-degree face as
    just inside the limit.
    """
    up = np.array([0.0, 0.0, 1.0])
    assert pr.overhang_angle(np.array([1.0, 0.0, 0.0]), up) is None


def test_an_upward_face_is_not_an_overhang():
    """None, not zero: it does not face downward at all."""
    up = np.array([0.0, 0.0, 1.0])
    assert pr.overhang_angle(np.array([0.0, 0.0, 1.0]), up) is None


@pytest.mark.parametrize('angle_deg', [10.0, 30.0, 44.0, 46.0, 70.0])
def test_the_angle_is_measured_from_the_plate(angle_deg):
    """
    A 45-degree face reads as 45, whichever way the arithmetic runs.

    Getting this backwards is the difference between flagging every
    vertical wall and flagging nothing.
    """
    up = np.array([0.0, 0.0, 1.0])
    # A normal tilted so the FACE sits `angle_deg` from the plate.
    radians = math.radians(angle_deg)
    normal = np.array([math.sin(radians), 0.0, -math.cos(radians)])
    assert pr.overhang_angle(normal, up) == pytest.approx(angle_deg, abs=0.01)


def test_the_bridge_span_is_the_narrow_direction(cfg):
    """
    A slicer bridges across the SHORT side of a ceiling.

    Measuring the long side flagged both arm beams - 107 mm cavities
    that bridge across 15 mm - as needing support they do not need, and
    a check that cries wolf gets switched off.
    """
    from build123d import Box, GeomType

    slab = Box(100.0, 10.0, 2.0)
    face = next(f for f in slab.faces()
                if f.geom_type == GeomType.PLANE
                and abs(f.normal_at().Z + 1.0) < 1e-6)
    assert pr._span(face) == pytest.approx(10.0, abs=0.01)


# -- the design itself --------------------------------------------------

def test_every_part_prints_without_support(cfg, limits):
    """
    THE STANDARD THE DESIGN IS HELD TO.

    Support on small brackets means a surface nobody can clean up, and
    inside a sealed cavity it cannot be removed at all. Getting here
    took two real design changes: a column under the bearing seat, and a
    bore that stops at the base's floor instead of punching through it.
    """
    limit, bridge = limits
    offenders = []
    for mechanism in parts_mod.MECHANISMS:
        for name, builder in parts_mod.parts_for(mechanism).items():
            report = pr.report(builder(cfg, mechanism=mechanism),
                               limit, bridge)
            if report['unsupported_mm2_best'] >= 1.0:
                offenders.append(
                    f'{mechanism}/{name}: '
                    f'{report["unsupported_mm2_best"]:.0f} mm2 '
                    f'(best orientation {report["best_orientation"]})')

    assert not offenders, (
        'these parts need support:\n  ' + '\n  '.join(offenders))


def test_the_recommended_orientation_is_never_worse_than_as_modelled(
        cfg, limits):
    """
    A recommendation has to buy something.

    Support is the first criterion and nothing outranks it, so whatever
    the strength tie-break below picks, it can never be a placement that
    needs more support than the part as drawn.
    """
    limit, bridge = limits
    for mechanism in parts_mod.MECHANISMS:
        for name, builder in parts_mod.parts_for(mechanism).items():
            solid = builder(cfg, mechanism=mechanism)
            as_modelled = pr.overhanging_area(
                solid, (0.0, 0.0, 1.0), limit, bridge)
            _, best = pr.best_orientation(solid, limit, bridge)
            assert best <= as_modelled + 1e-9, f'{mechanism}/{name}'


def test_the_bridge_limit_is_declared_not_assumed(cfg):
    """
    It describes the PRINTER, so it belongs in the profile.

    A bridging distance hardcoded in the checker is a machine assumption
    nobody can see.
    """
    fab = prof.fabrication(cfg)
    assert 'max_bridge_mm' in fab
    assert 5.0 < fab['max_bridge_mm'] < 50.0


# -- the strength tie-break ---------------------------------------------

def test_build_height_follows_the_build_direction():
    """The extent along the build axis, not the Z extent of the model."""
    from build123d import Box

    slab = Box(10.0, 20.0, 30.0)
    assert pr.build_height(slab, (0.0, 0.0, 1.0)) == pytest.approx(30.0)
    assert pr.build_height(slab, (1.0, 0.0, 0.0)) == pytest.approx(10.0)
    assert pr.build_height(slab, (0.0, -1.0, 0.0)) == pytest.approx(20.0)


def test_a_tie_at_zero_support_breaks_toward_the_shorter_build():
    """
    THE REASON THIS TIE-BREAK EXISTS.

    A plain beam needs no support whichever way up it goes, so support
    alone cannot choose, and the old rule kept it standing on end -
    which is the orientation that puts a bending load across every
    layer interface. Lying down is both stronger and less likely to be
    knocked over while it prints.
    """
    from build123d import Box

    beam = Box(10.0, 10.0, 60.0)
    label, area = pr.best_orientation(beam)
    assert area == pytest.approx(0.0)
    assert label != pr.AS_MODELLED
    assert pr.build_height(beam, pr.ORIENTATIONS[label]) == \
        pytest.approx(10.0)


def test_a_part_already_lying_down_is_left_alone():
    """Nothing to buy, so no recommendation to rotate."""
    from build123d import Box

    assert pr.best_orientation(Box(60.0, 60.0, 10.0))[0] == pr.AS_MODELLED


def test_a_near_cube_is_not_rotated_for_a_fraction_of_a_millimetre():
    """
    A 0.3 mm saving is not worth an instruction to rotate a part.

    Without a tolerance the wrist bracket flips orientation on the
    0.01 mm between its 32.00 and 31.99 mm faces, which is noise in the
    model dressed up as advice.
    """
    from build123d import Box

    assert pr.best_orientation(Box(10.0, 10.0, 10.3))[0] == pr.AS_MODELLED


def test_the_tie_break_never_stands_a_part_taller(cfg, limits):
    """
    Where height is the deciding criterion, it only ever lowers a part.

    Held across the real design, because the strength argument is only
    sound while that stays true. The condition matters: this asserts
    nothing about a part that had to be rotated to escape support, which
    is a different trade and the next test covers it.
    """
    limit, bridge = limits
    for mechanism in parts_mod.MECHANISMS:
        for name, builder in parts_mod.parts_for(mechanism).items():
            solid = builder(cfg, mechanism=mechanism)
            label, best = pr.best_orientation(solid, limit, bridge)
            as_modelled = pr.overhanging_area(
                solid, (0.0, 0.0, 1.0), limit, bridge)
            if as_modelled > best + 1e-9:
                continue          # support decided this one, not height
            assert pr.build_height(solid, pr.ORIENTATIONS[label]) <= \
                pr.build_height(solid) + pr.HEIGHT_TOLERANCE_MM, \
                f'{mechanism}/{name} would print taller than as modelled'


def test_support_outranks_height(cfg, limits):
    """
    THE ORDER THE TWO CRITERIA COME IN, ON THE PART THAT PROVES IT.

    The wrist bracket has 670 mm2 of flat ceiling too wide to bridge as
    drawn, and the orientation that clears it stands 32 mm tall against
    30. It gets recommended anyway: a part 2 mm taller is a cost worth
    paying, and support inside it is not removable at any price.
    """
    limit, bridge = limits
    solid = parts_mod.parts_for('direct')['wrist_bracket'](
        cfg, mechanism='direct')
    label, best = pr.best_orientation(solid, limit, bridge)

    assert best < 1.0
    assert pr.overhanging_area(solid, (0.0, 0.0, 1.0), limit, bridge) > 1.0
    assert label != pr.AS_MODELLED
    assert pr.build_height(solid, pr.ORIENTATIONS[label]) > \
        pr.build_height(solid)


@pytest.mark.parametrize('name', ['upper_arm_link', 'forearm_link'])
def test_the_arm_beams_print_lying_down(cfg, limits, name):
    """
    THE PART OF THE DESIGN THIS CHANGE IS FOR.

    Both beams are hollow, 1.2 mm walled and loaded in bending, and both
    print without support either way up. Standing on end they are around
    100 mm of stacked layer interfaces carrying that bending; lying down
    the load runs along the extrusions instead.
    """
    limit, bridge = limits
    solid = parts_mod.parts_for('direct')[name](cfg, mechanism='direct')
    label, area = pr.best_orientation(solid, limit, bridge)
    assert area < 1.0, 'still has to need no support'
    assert pr.build_height(solid, pr.ORIENTATIONS[label]) < \
        pr.build_height(solid) / 2.0
