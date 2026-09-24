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


#: Parts distal to the shoulder-lift joint: everything that servo has to
#: raise. The turret and base are proximal to it and are not lifted.
LIFTED_PARTS = ('upper_arm_link', 'forearm_link', 'wrist_bracket',
                'gripper_base')

#: Servos distal to the shoulder lift, per mechanism. This is the whole
#: argument for a linkage: the elbow servo moves from the arm to the
#: turret, and 55 g at the end of a lever is worth more than 55 g.
LIFTED_SERVOS = {
    'direct': ('mg996r', 'sg90', 'sg90'),   # elbow, wrist, gripper
    'linkage': ('sg90', 'sg90'),            # wrist, gripper only
}


def compare(cfg, results):
    """
    Report the number the mechanism decision actually turns on.

    Not total printed mass - that favours direct drive and is nearly
    irrelevant. What matters is what the shoulder servo has to lift,
    because that is the joint with the longest lever and the least
    margin.
    """
    servos = {name: spec['mass_kg'] * 1000.0
              for name, spec in cfg['servos'].items()}
    out = {}
    for mechanism, rows in results.items():
        by_part = {row['part']: row['mass_g'] for row in rows}
        printed = sum(by_part.values()) + by_part.get('gripper_finger', 0.0)
        lifted = sum(by_part.get(p, 0.0) for p in LIFTED_PARTS)
        lifted += 2 * by_part.get('gripper_finger', 0.0)
        lifted += sum(servos[s] for s in LIFTED_SERVOS[mechanism])
        out[mechanism] = {'printed_g': printed, 'lifted_g': lifted}
    return out


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
    collected = {}
    for mechanism in wanted:
        print(f'  {mechanism}')
        results = generate(cfg, mechanism,
                           pathlib.Path(args.out) / mechanism)
        collected[mechanism] = results
        for row in results:
            print('    %-18s %7.0f mm3  %5.1f g  %4.0f x %4.0f x %4.0f mm' % (
                row['part'], row['volume_mm3'], row['mass_g'],
                *row['bbox_mm']))
        print()

    print(f'  written to {args.out}/<mechanism>/')

    if len(wanted) > 1:
        summary = compare(cfg, collected)
        print()
        print('  mechanism      PLA total   lifted by the shoulder servo')
        for mechanism, row in summary.items():
            print('    %-12s %6.0f g    %6.0f g' % (
                mechanism, row['printed_g'], row['lifted_g']))
        direct = summary['direct']
        linkage = summary['linkage']
        print()
        print('  The linkage needs %.0f g MORE plastic and lifts %.0f g LESS'
              % (linkage['printed_g'] - direct['printed_g'],
                 direct['lifted_g'] - linkage['lifted_g']))
        print('  (%.0f%% less at the joint with the longest lever). Total'
              % (100 * (direct['lifted_g'] - linkage['lifted_g'])
                 / direct['lifted_g']))
        print('  printed mass is the number that favours direct drive and')
        print('  it is nearly irrelevant; this one is not.')
        budget = 250.0
        for mechanism, row in summary.items():
            if row['printed_g'] > budget:
                print()
                print('  NOTE: %s needs %.0f g of PLA against the %.0f g the'
                      % (mechanism, row['printed_g'], budget))
                print('  BOM budgets. docs/bom-hardware.md needs updating or')
                print('  the design needs thinning.')
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
