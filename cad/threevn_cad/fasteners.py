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
Counting the screws, from the holes that are actually in the parts.

The BOM said "M3 hardware, ~$5" and a note that no schedule had been
derived. This derives one - by measuring the geometry rather than by
someone counting features in a drawing and getting it slightly wrong.

WHAT IT CANNOT KNOW, and says so rather than guessing:

    LENGTH.    A hole's depth is not a screw's length. How far the
               screw has to reach depends on what is behind it, which
               is an assembly question this has no model of.
    PAIRING.   Two coaxial holes in two parts are one screw, not two.
               Without an assembly, a hole is a hole.
    NUTS.      Whether a hole takes a nut, a heat-set insert or a
               self-tapping screw into plastic is a decision nobody
               has made yet.

So the output is a count of HOLES by diameter, which is a real
measurement, and an explicit statement that turning it into a shopping
list needs the assembly. A confident schedule derived from half the
information would be worse than the honest note it replaces.
"""

import collections

from build123d import GeomType

from threevn_cad import fits
from threevn_cad import parts as parts_mod
from threevn_cad import profile as prof


def holes_in(solid, tolerance=0.05):
    """
    Return {diameter_mm: count} for the cylindrical faces in a solid.

    Counts FACES, which is not the same as counting holes: a hole cut
    through a hollow part appears twice, once per wall it passes
    through. That is reported as-is rather than halved, because halving
    would be right for a through-hole and wrong for a blind one, and
    nothing here can tell them apart.
    """
    counts = collections.Counter()
    for face in solid.faces():
        if face.geom_type != GeomType.CYLINDER or face.radius is None:
            continue
        counts[round(face.radius * 2, 2)] += 1
    return dict(counts)


def schedule(cfg, mechanism):
    """
    Return the fastener-relevant holes across every part, by size.

    Only diameters that correspond to a known fastener are reported.
    Every other cylindrical face in the design is a bore, a pocket or a
    corner round, and listing them as fastener holes would inflate the
    count with geometry nobody screws into.
    """
    # ONE LABEL PER DIAMETER, because a diameter is all that can be
    # measured. A plain M3 clearance hole and a bushing's own bore are
    # the same size by construction - the bushing slides on the same
    # screw - so they cannot be told apart from geometry, and giving
    # them separate labels only meant one silently overwrote the other
    # in this dict.
    known = {
        round(fits.screw_bore_radius(cfg) * 2, 2):
            'M3 clearance (plain holes and bushing bores alike)',
        round(fits.pivot_bore_radius(cfg, bushed=True) * 2, 2):
            'bushed pivot (takes a bushing, not a screw)',
    }
    for name in ('mg996r', 'sg90'):
        servo = prof.servo(cfg, name)
        size = round((servo['horn_screw_hole_mm']
                      + prof.fabrication(cfg)['hole_clearance_mm']), 2)
        known.setdefault(size, f'{name} horn screw')

    rows = []
    for part_name, builder in sorted(parts_mod.parts_for(mechanism).items()):
        solid = builder(cfg, mechanism=mechanism)
        for diameter, count in sorted(holes_in(solid).items()):
            if diameter not in known:
                continue
            rows.append({'part': part_name, 'diameter_mm': diameter,
                         'faces': count, 'kind': known[diameter]})
    return rows


def totals(rows):
    """Sum faces by fastener kind."""
    out = collections.Counter()
    for row in rows:
        out[row['kind']] += row['faces']
    return dict(out)
