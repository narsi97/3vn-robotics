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

    Ties break toward the part as modelled, because telling someone to
    rotate a part costs attention and should only happen when it helps.
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
