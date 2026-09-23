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
The end-to-end acceptance run, and the report it produces.

Executes the whole sequence against whatever robot is up - home, move,
open, pick, close, place, release, verify - collects telemetry alongside
it, and writes a report that says which software produced the result.

    ros2 run threevn_control acceptance --report /tmp/acceptance

Exit code is 0 only if every scenario passed, so this works as a CI gate
as well as a demo.

Telemetry comes from the dashboard's HTTP API rather than from a second
set of ROS subscriptions. That is deliberate: it exercises the same
endpoint an operator or a monitor would use, so a report that says the
robot was healthy is backed by the thing that would have told anyone
else.
"""

import argparse
import json
import pathlib
import sys
import time
import urllib.error
import urllib.request

import rclpy

from threevn_control.robot_client import RobotClient
from threevn_control.scenarios import arm  # noqa: F401  (registers scenarios)
from threevn_control.scenarios.base import REGISTRY

#: Run in this order. Not alphabetical: cheap checks first, so a broken
#: robot fails in seconds rather than after a full pick-and-place.
SEQUENCE = [
    'home',
    'move_joint',
    'move_to_position',
    'gripper',
    'safety_limit',
    'pick_and_place',
]

DASHBOARD = 'http://127.0.0.1:8107'


def _fetch(path, timeout=3.0):
    """GET a dashboard endpoint, returning None when unavailable."""
    try:
        with urllib.request.urlopen(f'{DASHBOARD}{path}', timeout=timeout) as r:
            return json.loads(r.read())
    except (urllib.error.URLError, urllib.error.HTTPError, OSError,
            json.JSONDecodeError):
        return None


def collect_telemetry():
    """
    Snapshot what the dashboard reports, if it is running.

    Absence is recorded rather than treated as failure: the acceptance
    sequence is about the robot, and a report that refused to exist
    because the dashboard was down would be less useful than one that
    says so.
    """
    version = _fetch('/api/version')
    state = _fetch('/api/state')
    if version is None and state is None:
        return {'available': False,
                'note': 'dashboard not reachable at ' + DASHBOARD}
    telemetry = {'available': True}
    if version:
        telemetry['version'] = version
    if state:
        telemetry['readiness'] = {
            'ready': state.get('ready'),
            'joints_fresh': state.get('joints_fresh'),
            'controllers_fresh': state.get('controllers_fresh'),
            'hardware_fresh': state.get('hardware_fresh'),
            'joint_messages': state.get('joint_messages'),
            'uptime_seconds': state.get('uptime_seconds'),
        }
        telemetry['controllers'] = state.get('controllers', [])
        telemetry['hardware'] = state.get('hardware', [])
    return telemetry


def run_sequence(robot, names):
    """Execute the scenarios in order, returning their results."""
    results = []
    for name in names:
        result = REGISTRY[name](robot).execute()
        print(result.report())
        print()
        results.append(result)
        # Stop after a failure. Continuing would command the arm from a
        # pose nothing verified, which on real hardware breaks things.
        if not result.passed:
            print(f'  stopping: {name} failed\n')
            break
    return results


def build_report(results, telemetry, requested, started_at, seconds):
    """Assemble the machine-readable report."""
    passed = all(r.passed for r in results) and len(results) == len(requested)
    return {
        'verdict': 'PASS' if passed else 'FAIL',
        'started_at': started_at,
        'duration_seconds': round(seconds, 2),
        'scenarios_requested': requested,
        'scenarios_run': [r.name for r in results],
        'scenarios_skipped': requested[len(results):],
        'results': [
            {
                'name': r.name,
                'passed': r.passed,
                'seconds': round(r.seconds, 2),
                'steps': [
                    {'name': s.name, 'passed': s.passed,
                     'seconds': round(s.seconds, 2), 'detail': s.detail}
                    for s in r.steps
                ],
            }
            for r in results
        ],
        'telemetry': telemetry,
    }


def render_markdown(report):
    """Render the report as Markdown for a human or a CI summary."""
    lines = [
        '# 3VN Robotics acceptance report',
        '',
        f'**{report["verdict"]}** in {report["duration_seconds"]:.1f}s '
        f'({report["started_at"]})',
        '',
    ]

    telemetry = report['telemetry']
    if telemetry.get('available') and 'version' in telemetry:
        version = telemetry['version']
        lines += ['## Software under test', '',
                  '| field | value |', '|---|---|']
        for key in ('robot_id', 'software_version', 'git_commit',
                    'firmware_version', 'robot_model', 'config_profile',
                    'hardware_target'):
            lines.append(f'| {key} | `{version.get(key, "unknown")}` |')
        lines.append('')
    else:
        lines += ['> Telemetry unavailable: '
                  + telemetry.get('note', 'dashboard not reachable') + '.',
                  '> The result below still stands, but cannot be tied to a '
                  'specific build.', '']

    lines += ['## Scenarios', '', '| scenario | result | time |', '|---|---|---|']
    for result in report['results']:
        mark = 'pass' if result['passed'] else '**FAIL**'
        lines.append(f'| {result["name"]} | {mark} | {result["seconds"]:.2f}s |')
    for skipped in report['scenarios_skipped']:
        lines.append(f'| {skipped} | skipped | - |')
    lines.append('')

    failures = [r for r in report['results'] if not r['passed']]
    if failures:
        lines += ['## Failures', '']
        for result in failures:
            for step in result['steps']:
                if not step['passed']:
                    lines.append(
                        f'- `{result["name"]}` / {step["name"]}: '
                        f'{step["detail"] or "post-condition not met"}')
        lines.append('')

    readiness = telemetry.get('readiness')
    if readiness:
        lines += ['## Telemetry at the end of the run', '',
                  '| signal | value |', '|---|---|']
        for key, value in readiness.items():
            lines.append(f'| {key} | `{value}` |')
        lines.append('')

    return '\n'.join(lines)


def main(argv=None):
    """Entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', metavar='PREFIX',
                        help='write PREFIX.json and PREFIX.md')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='seconds to wait for the robot interface')
    parser.add_argument('--only', nargs='*', metavar='SCENARIO',
                        help='run only these scenarios, in this order')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    requested = args.only or SEQUENCE
    unknown = [n for n in requested if n not in REGISTRY]
    if unknown:
        print(f'unknown scenario(s): {unknown}', file=sys.stderr)
        return 2

    started_at = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    started = time.monotonic()

    rclpy.init()
    try:
        robot = RobotClient(timeout=args.timeout)
        try:
            robot.connect()
        except Exception as exc:                       # noqa: BLE001
            print(f'\n  cannot reach the robot: {exc}\n'
                  '  Is `make sim` or `make mock` running?\n', file=sys.stderr)
            return 3

        print()
        results = run_sequence(robot, requested)
        telemetry = collect_telemetry()
        robot.destroy_node()
    finally:
        rclpy.shutdown()

    report = build_report(results, telemetry, requested, started_at,
                          time.monotonic() - started)

    if args.report:
        prefix = pathlib.Path(args.report)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        prefix.with_suffix('.json').write_text(json.dumps(report, indent=2))
        prefix.with_suffix('.md').write_text(render_markdown(report))
        print(f'  report: {prefix.with_suffix(".json")}')
        print(f'          {prefix.with_suffix(".md")}')

    print()
    print(f'  {report["verdict"]} -- '
          f'{sum(1 for r in report["results"] if r["passed"])}'
          f'/{len(requested)} scenarios in {report["duration_seconds"]:.1f}s')
    if not telemetry.get('available'):
        print('  (telemetry unavailable: run `make dash` alongside to '
              'tie the result to a build)')
    print()
    return 0 if report['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
