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
Run a 3VN scenario against whatever robot is currently up.

    ros2 run threevn_control run_scenario --list
    ros2 run threevn_control run_scenario home
    ros2 run threevn_control run_scenario --all

It does not care whether that robot is mock, Gazebo or hardware: it talks
to the same topics and actions in every case, which is the point.

Exit code is 0 only if every scenario passed, so this works as a CI step
as well as a demo.
"""

import argparse
import sys

import rclpy

from threevn_control.robot_client import RobotClient
from threevn_control.scenarios import arm  # noqa: F401  (registers scenarios)
from threevn_control.scenarios.base import REGISTRY


def main(argv=None):
    """Entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', nargs='?', help='scenario name')
    parser.add_argument('--list', action='store_true', help='list scenarios')
    parser.add_argument('--all', action='store_true', help='run every scenario')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='seconds to wait for the robot interface')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if args.list:
        print()
        for name in sorted(REGISTRY):
            print(f'  {name:<18} {REGISTRY[name].description}')
        print()
        return 0

    if not args.all and not args.scenario:
        parser.error('give a scenario name, --all, or --list')

    names = sorted(REGISTRY) if args.all else [args.scenario]
    for name in names:
        if name not in REGISTRY:
            print(f'unknown scenario {name!r}; one of {sorted(REGISTRY)}',
                  file=sys.stderr)
            return 2

    rclpy.init()
    failures = 0
    try:
        robot = RobotClient(timeout=args.timeout)
        try:
            robot.connect()
        except Exception as exc:                       # noqa: BLE001
            print(f'\n  cannot reach the robot: {exc}\n'
                  '  Is `make sim` or `make mock` running?\n', file=sys.stderr)
            return 3

        print()
        for name in names:
            result = REGISTRY[name](robot).execute()
            print(result.report())
            print()
            if not result.passed:
                failures += 1
        robot.destroy_node()
    finally:
        rclpy.shutdown()

    total = len(names)
    print(f'  {total - failures}/{total} scenarios passed')
    print()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
