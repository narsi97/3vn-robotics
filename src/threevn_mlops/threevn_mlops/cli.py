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
The registry, from a shell.

    make registry-register            # store the freshly trained model
    make registry-promote             # run the gates, then promote
    make registry-status              # what is running, and what else exists
    make registry-rollback            # put the previous one back

Promotion runs the gates and reports EVERY failure rather than the first,
because a model that fails three checks should be fixed once rather than
three times.
"""

import argparse
import json
import pathlib
import sys

from threevn_ml import features as feat

from threevn_mlops import gates
from threevn_mlops.registry import Registry


def _status(registry):
    """Print what is registered and what is running."""
    production = registry.production_version()
    versions = registry.versions()

    print()
    if not versions:
        print(f'  {registry.root}: empty')
        return 0

    print(f'  {registry.root}')
    print()
    print('    version       test    baseline  frames  registered')
    for version, row in versions.items():
        mark = '->' if version == production else '  '
        score = row.get('test_median_mm')
        base = row.get('baseline_median_mm')
        print('    %s %-12s %6s  %8s  %6s  %s' % (
            mark, version,
            f'{score:.1f}' if score is not None else '-',
            f'{base:.1f}' if base is not None else '-',
            row.get('dataset_frames') or '-',
            row['registered'][:19]))

    print()
    if production:
        print(f'    production: {production}')
    else:
        print('    production: NOTHING PROMOTED. The serving node will '
              'refuse to start.')

    history = registry.history()[-5:]
    if history:
        print()
        print('    recent history')
        for row in history:
            extra = ''
            if row['action'] == 'promote':
                extra = f' (from {row.get("from") or "nothing"})'
                if row.get('forced'):
                    extra += ' FORCED'
            print(f'      {row["at"][:19]}  {row["action"]:9s} '
                  f'{row["version"]}{extra}')
    print()
    return 0


def main(argv=None):
    """Run the registry CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', default='/ws/datasets/registry')
    sub = parser.add_subparsers(dest='command', required=True)

    register = sub.add_parser('register', help='store a trained artifact')
    register.add_argument('--model', default='/ws/datasets/target/model.json')
    register.add_argument('--note', default='')

    promote = sub.add_parser('promote', help='run the gates, then promote')
    promote.add_argument('version', nargs='?',
                         help='defaults to the newest registered')
    promote.add_argument('--reason', default='')
    promote.add_argument('--force', action='store_true',
                         help='promote despite failing gates, and record '
                              'that it was forced')

    sub.add_parser('status', help='what is registered and running')

    rollback = sub.add_parser('rollback', help='put the previous model back')
    rollback.add_argument('--reason', default='')

    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    registry = Registry(args.registry)

    if args.command == 'status':
        return _status(registry)

    if args.command == 'register':
        artifact = json.loads(pathlib.Path(args.model).read_text())
        version = registry.register(artifact, note=args.note)
        print(f'  registered {version}')
        return 0

    if args.command == 'rollback':
        try:
            previous = registry.rollback(reason=args.reason)
        except RuntimeError as exc:
            print(f'  cannot roll back: {exc}')
            return 1
        print(f'  rolled back to {registry.production_version()} '
              f'(was {previous})')
        return 0

    # promote
    version = args.version or next(iter(registry.versions()), None)
    if version is None:
        print('  nothing registered to promote')
        return 1

    artifact = registry.artifact(version)
    incumbent = registry.production()
    failures = gates.check_all(artifact, feat.FEATURE_NAMES, incumbent)

    print()
    for gate in ('validates', 'provenance', 'feature_contract',
                 'beats_baseline', 'no_regression'):
        if gate in failures:
            print(f'    FAIL  {gate}')
            for reason in failures[gate]:
                print(f'          {reason}')
        else:
            print(f'    ok    {gate}')
    print()

    if failures and not args.force:
        print(f'  {version} NOT promoted. Fix the above, or pass --force '
              f'to promote anyway and have it recorded as forced.')
        return 1

    previous = registry.promote(version, reason=args.reason,
                                forced=bool(failures))
    state = 'FORCED past failing gates' if failures else 'promoted'
    print(f'  {version} {state} (previous: {previous or "nothing"})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
