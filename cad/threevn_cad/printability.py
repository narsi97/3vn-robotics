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
Whether a part can come off a printer in one piece.

`max_overhang_deg` has been in the profile since the CAD started and
nothing used it, which made it a number that described an intention
rather than a constraint. This checks it.

WHAT THIS IS NOT. It is not a slicer. A slicer decides support from the
actual toolpaths and knows about bridging, and this only looks at face
normals - so it flags surfaces a slicer would bridge happily, and misses
nothing that matters. It is a screening check, and the honest reading of
a flagged face is "look at this in the slicer", not "this cannot print".

ORIENTATION IS THE ANSWER TO MOST OVERHANGS, and it is not a property of
the solid - it is a decision made when the part is placed on the bed.
`worst_overhang` therefore takes the build direction, and
`best_orientation` tries the six axis-aligned ones so the exported part
can carry a recommendation rather than leaving every user to guess.

It ranks them on support first and then on how tall the part would
stand, which is a strength criterion rather than a printability one:
printed PLA is weakest across its layer lines, so a part is stronger
lying down than standing up. Support still outranks it.
"""

import itertools
import math

import numpy as np
from build123d import GeomType

#: The orientation the part was drawn in, which wins ties.
AS_MODELLED = '+Z up (as modelled)'

#: Build heights closer than this count as the same height. The wrist
#: bracket is 32.00 by 31.99 mm, and 0.01 mm is not a reason to tell
#: anyone to rotate a part.
HEIGHT_TOLERANCE_MM = 0.5

#: The six axis-aligned build directions, by the face that goes down.
ORIENTATIONS = {
    AS_MODELLED: (0.0, 0.0, 1.0),
    '-Z up (flipped)': (0.0, 0.0, -1.0),
    '+X up': (1.0, 0.0, 0.0),
    '-X up': (-1.0, 0.0, 0.0),
    '+Y up': (0.0, 1.0, 0.0),
    '-Y up': (0.0, -1.0, 0.0),
}


def _face_normals_and_areas(solid):
    """Return (normal, area) for every planar face."""
    out = []
    for face in solid.faces():
        if face.geom_type != GeomType.PLANE:
            continue
        try:
            normal = face.normal_at()
            out.append((np.array([normal.X, normal.Y, normal.Z]), face.area))
        except (TypeError, ValueError):
            continue
    return out


def _face_points(solid, build_dir):
    """Return (normal, area, height) per planar face, height along build_dir."""
    up = np.array(build_dir, dtype=float)
    up /= np.linalg.norm(up)
    rows = []
    for face in solid.faces():
        if face.geom_type != GeomType.PLANE:
            continue
        try:
            normal = face.normal_at()
            centre = face.center()
        except (TypeError, ValueError):
            continue
        n = np.array([normal.X, normal.Y, normal.Z], dtype=float)
        c = np.array([centre.X, centre.Y, centre.Z], dtype=float)
        rows.append((n, face.area, float(np.dot(c, up))))
    return rows, up


def overhang_angle(normal, up):
    """
    Return a downward face's angle from the build plate, in degrees.

    A horizontal ceiling is 0 degrees and is the worst case; a vertical
    wall is 90 and needs nothing. Returns None for a face that does not
    point downward at all.

    THIS WAS BACKWARDS AT FIRST, and the check reported zero unsupported
    area for every part in the design - which reads as "nothing to worry
    about" rather than as "this test does nothing". The original
    condition skipped faces steeper than the limit, which are exactly
    the overhangs.
    """
    component = float(np.dot(normal, up))
    if component >= -1e-9:
        return None
    return math.degrees(math.acos(min(1.0, -component)))


def _span(face):
    """
    Return the distance a slicer would have to bridge across a face, mm.

    THE SHORTER in-plane dimension, not the longer. A slicer bridges
    across the narrow direction of a ceiling, so a long narrow cavity
    roof is easy and a square one of the same area is not. Taking the
    longest dimension flagged both arm beams - 107 mm cavities that
    bridge across 15 mm - as needing support they do not need.
    """
    box = face.bounding_box()
    sizes = sorted((box.size.X, box.size.Y, box.size.Z))
    # sizes[0] is the face's own thickness, ~0 for a plane.
    return sizes[1]


def overhanging_area(solid, build_dir=(0.0, 0.0, 1.0), limit_deg=45.0,
                     bridge_mm=0.0):
    """
    Return the planar area that would need support, in mm2.

    Two exclusions, both of which are the difference between a useful
    check and one that gets switched off:

    ON THE PLATE. A downward face resting on the build plate needs
    nothing, which is why the face's height matters and not just its
    normal - the largest downward face on most of these parts is the
    floor.

    BRIDGEABLE. A flat ceiling narrower than `bridge_mm` is bridged, not
    supported. Without this, every hollow part here is flagged: the
    beams have a sealed cavity whose ceiling a slicer spans without
    difficulty, and support inside a sealed cavity would be
    unremovable anyway.
    """
    rows, up = _face_points(solid, build_dir)
    if not rows:
        return 0.0
    plate = min(height for _, _, height in rows)

    faces = [f for f in solid.faces() if f.geom_type == GeomType.PLANE]
    total = 0.0
    for face in faces:
        try:
            normal = face.normal_at()
            centre = face.center()
        except (TypeError, ValueError):
            continue
        n = np.array([normal.X, normal.Y, normal.Z], dtype=float)
        height = float(np.dot(
            np.array([centre.X, centre.Y, centre.Z], dtype=float), up))

        angle = overhang_angle(n, up)
        if angle is None or abs(height - plate) < 1e-6:
            continue
        if angle >= limit_deg:
            continue
        if bridge_mm and _span(face) <= bridge_mm:
            continue                      # the slicer will bridge it
        total += face.area
    return total


def worst_overhang(solid, build_dir=(0.0, 0.0, 1.0)):
    """
    Return the steepest unsupported downward face angle, from the plate.

    90 means nothing overhangs. Lower is worse.
    """
    rows, up = _face_points(solid, build_dir)
    if not rows:
        return 90.0
    plate = min(height for _, _, height in rows)

    worst = 90.0
    for normal, _, height in rows:
        angle = overhang_angle(normal, up)
        if angle is None or abs(height - plate) < 1e-6:
            continue
        worst = min(worst, angle)
    return worst


def build_height(solid, build_dir=(0.0, 0.0, 1.0)):
    """Return how tall the part stands along the build direction, mm."""
    up = np.array(build_dir, dtype=float)
    up /= np.linalg.norm(up)
    box = solid.bounding_box()
    heights = [
        float(np.dot(np.array(corner, dtype=float), up))
        for corner in itertools.product(
            (box.min.X, box.max.X), (box.min.Y, box.max.Y),
            (box.min.Z, box.max.Z))
    ]
    return max(heights) - min(heights)


def best_orientation(solid, limit_deg=45.0, bridge_mm=0.0):
    """
    Return (label, unsupported_area) for the best axis-aligned placement.

    Three criteria, applied in order.

    SUPPORT FIRST. Support on these parts is either a surface nobody can
    clean up or, inside a sealed cavity, unremovable, so no amount of
    anything else buys it.

    THEN THE SHORTEST BUILD HEIGHT, which is a strength argument.
    Printed PLA is weakest across its layer interfaces - roughly half to
    two thirds of its strength along them - so a part loaded in bending
    wants its long axis lying in the build plane instead of stacked up
    the Z axis. The arm links are the case that matters: hollow beams
    with 1.2 mm walls, carrying bending, and standing them upright puts
    every one of those loads across a layer boundary. Laying a tall thin
    part down also stops it wobbling on a bed-slinger, and gives it more
    first-layer area to hold on with.

    THIS ONLY CHOOSES BETWEEN PLACEMENTS THAT ALREADY NEED NO SUPPORT,
    which is what keeps it honest - it never trades an unremovable
    support for a stronger part. What it does cost is bridging: laid
    down, a beam's sealed cavity becomes a long ceiling the slicer has
    to span. That is a surface nobody sees, carrying little load, and a
    slicer bridges it on purpose.

    THEN THE PART AS MODELLED, because a recommendation to rotate
    carries a cost - someone has to notice it, and the orientation has
    to survive the trip to the slicer - and it should only be made when
    it buys something.
    """
    scored = [
        (label,
         overhanging_area(solid, direction, limit_deg, bridge_mm),
         build_height(solid, direction))
        for label, direction in ORIENTATIONS.items()
    ]

    least = min(area for _, area, _ in scored)
    standing = [row for row in scored if row[1] <= least + 1e-9]

    lowest = min(height for _, _, height in standing)
    standing = [row for row in standing
                if row[2] <= lowest + HEIGHT_TOLERANCE_MM]

    # min() keeps the first of equals, so this falls back to the order
    # ORIENTATIONS is declared in once as-modelled is out of the running.
    label, area, _ = min(standing, key=lambda row: row[0] != AS_MODELLED)
    return label, area


def report(solid, limit_deg=45.0, bridge_mm=0.0):
    """Summarise printability for one part."""
    label, area = best_orientation(solid, limit_deg, bridge_mm)
    return {
        'unsupported_mm2_as_modelled': overhanging_area(
            solid, (0.0, 0.0, 1.0), limit_deg, bridge_mm),
        'best_orientation': label,
        'unsupported_mm2_best': area,
        'build_height_mm_best': build_height(solid, ORIENTATIONS[label]),
        'worst_face_deg': worst_overhang(solid),
    }
