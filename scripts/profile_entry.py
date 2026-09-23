#!/usr/bin/env python3
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
Print the xacro entry point for a robot profile.

Which xacro a profile belongs to is decided by its `meta.kind`, and that
mapping kept getting written out again by hand. Three places had their
own copy -- the pytest conftest, the check_urdf cross-check and the CI
artifact step -- and adding the mobile base broke all three the same
way: each globbed every profile in config/ and fed it to the ARM xacro.
Two of them were fixed separately before anyone noticed it was one bug.

Shell callers share this. Usage:

    ros2 run xacro xacro "$SHARE/urdf/$(profile_entry.py "$profile")" ...
"""

import pathlib
import sys

import yaml

ENTRY_POINTS = {
    'arm': 'threevn_arm.urdf.xacro',
    'base': 'threevn_base.urdf.xacro',
    'mobile_manipulator': 'threevn_mobile_manipulator.urdf.xacro',
}


def main():
    """Resolve one profile path to its entry-point filename."""
    if len(sys.argv) != 2:
        sys.exit('usage: profile_entry.py <profile.yaml>')

    path = pathlib.Path(sys.argv[1])
    try:
        meta = yaml.safe_load(path.read_text())['meta']
    except (OSError, KeyError, yaml.YAMLError) as exc:
        sys.exit(f'{path}: cannot read meta block: {exc}')

    kind = meta.get('kind')
    if kind not in ENTRY_POINTS:
        sys.exit(
            f'{path}: meta.kind={kind!r} has no xacro entry point. '
            f'Known families: {sorted(ENTRY_POINTS)}. Add the new one here '
            f'and in test/conftest.py rather than guessing at the call site.'
        )
    print(ENTRY_POINTS[kind])


if __name__ == '__main__':
    main()
