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
Generate the printable files.

    make cad

Writes STL and STEP for every part, for both mechanisms, into
`cad/out/<mechanism>/`. That directory is gitignored: generated
geometry is a build product, and committing it would create a second
copy of the truth that goes stale the moment the profile changes - the
same reason the URDF is expanded at launch instead of being generated
and committed.

STL is what a slicer eats. STEP is what anyone wanting to modify the
part in real CAD needs, and exporting only STL is how an "open" design
becomes un-editable.
"""

import argparse
import pathlib
import sys

from build123d import export_step, export_stl

from threevn_cad import parts as parts_mod
from threevn_cad import profile as prof

OUT = pathlib.Path(__file__).resolve().parents[1] / 'out'


def generate(cfg, mechanism, out_dir):
    """Write every part for one mechanism. Returns a list of summaries."""
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    written = []
    for name, builder in sorted(parts_mod.PARTS.items()):
        solid = builder(cfg, mechanism=mechanism)
        stl = out_dir / f'{name}.stl'
        step = out_dir / f'{name}.step'
        export_stl(solid, str(stl))
        export_step(solid, str(step))

        box = solid.bounding_box()
        written.append({
            'part': name,
            'volume_mm3': solid.volume,
            # PLA at 1.24 g/cm3. The printed mass is what the URDF's
            # link masses ought to agree with, and today they are
            # `provenance: estimated` guesses that predate any geometry.
            'mass_g': solid.volume / 1000.0 * 1.24,
            'bbox_mm': (box.size.X, box.size.Y, box.size.Z),
            'stl': stl,
            'step': step,
        })
    return written


def main(argv=None):
    """Generate parts for both mechanisms and report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default=None)
    parser.add_argument('--out', default=str(OUT))
    parser.add_argument('--mechanism', choices=parts_mod.MECHANISMS + ('both',),
                        default='both')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    cfg = prof.load(args.profile)
    wanted = (parts_mod.MECHANISMS if args.mechanism == 'both'
              else (args.mechanism,))

    print()
    for mechanism in wanted:
        print(f'  {mechanism}')
        results = generate(cfg, mechanism,
                           pathlib.Path(args.out) / mechanism)
        for row in results:
            print('    %-18s %7.0f mm3  %5.1f g  %4.0f x %4.0f x %4.0f mm' % (
                row['part'], row['volume_mm3'], row['mass_g'],
                *row['bbox_mm']))
        print()

    print(f'  written to {args.out}/<mechanism>/')
    print()
    print('  The masses above are what the PARTS weigh. The URDF still')
    print('  carries provenance: estimated guesses that predate any')
    print('  geometry, and reconciling the two is the next honest step -')
    print('  the simulation is currently modelling a robot nobody has')
    print('  designed.')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
