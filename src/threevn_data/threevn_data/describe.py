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
Say what is in a dataset, and what the two ways of splitting it cost.

    make dataset-describe

The leakage comparison is the part worth running. It takes the SAME
frames, splits them the correct way and the tempting way, and measures
how close each test frame's nearest training neighbour is. One of the
two numbers is a description of a test set; the other is a description
of a mistake.
"""

import argparse
import pathlib
import sys

from threevn_data import dataset as ds
from threevn_data import manifest as manifest_mod


def main(argv=None):
    """Describe a dataset."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', default='/ws/datasets/target')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    directory = pathlib.Path(args.dataset)
    doc = manifest_mod.read(directory)
    problems = manifest_mod.validate(doc)
    frames = ds.load(directory)
    summary = ds.summarise(frames)

    print()
    print(f'  {directory}')
    print(f'    recorded    {doc["created"]}')
    commit = doc['code'].get('commit')
    dirty = ' (dirty tree)' if doc['code'].get('dirty') else ''
    print(f'    code        {commit[:12] if commit else "unknown"}{dirty}')
    print(f'    world       {doc["world"]}')
    print(f'    robot       {doc["robot"]}')
    print(f'    labels      {doc["labels"]["source"]}: '
          f'{doc["labels"]["quantity"]}')
    print(f'    camera      {doc["camera"]["width"]}x{doc["camera"]["height"]} '
          f'fx={doc["camera"]["fx"]:.1f} in {doc["camera"]["frame_id"]}')
    print(f'    manifest    {"OK" if not problems else "PROBLEMS"}')
    for problem in problems:
        print(f'      - {problem}')

    print()
    print(f'    frames      {summary["frames"]} across '
          f'{summary["episodes"]} episodes')
    print('    label range x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f (m)' % (
        summary['label_min'][0], summary['label_max'][0],
        summary['label_min'][1], summary['label_max'][1],
        summary['label_min'][2], summary['label_max'][2]))

    images = sorted((directory / 'images').glob('*.png'))
    size = sum(p.stat().st_size for p in images)
    print(f'    on disk     {size / 1e6:.1f} MB in {len(images)} images '
          f'({size / max(len(images), 1) / 1e3:.0f} kB each)')

    dupes = ds.duplicate_report(frames)
    print(f'    distinct    {dupes["unique"]} unique images of '
          f'{dupes["files"]} files', end='')
    if dupes['unique'] < dupes['files']:
        print(f'  <- {dupes["files"] - dupes["unique"]} are copies; '
              f'largest group {dupes["largest_group"]}')
    else:
        print()

    print()
    print('    how the split is made            nearest train neighbour')
    print('                                     min     median   <10mm')
    for name, splitter in (('by episode (correct)', ds.split_by_episode),
                           ('by frame (leaks)', ds.split_by_frame)):
        try:
            report = ds.leakage_report(splitter(frames))
        except ValueError as exc:
            print(f'    {name:32s} {exc}')
            continue
        print('    %-32s %5.1f mm %6.1f mm %4d/%d' % (
            name, report['min_mm'], report['median_mm'],
            report['within_10mm'], report['test_frames']))

    print()
    print('    A test frame whose nearest training neighbour is millimetres')
    print('    away is not a test. It is a frame the model has effectively')
    print('    already seen, scored as though it had not.')
    print()

    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
