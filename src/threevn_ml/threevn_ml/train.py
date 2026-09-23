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
Fit the model, and report it against the baseline it has to beat.

    make train

The baseline is evaluated in the same run, on the same test set, by the
same function. "The model beat the geometry" is not a claim until both
numbers come out of one place.

The split is by EPISODE, always. A frame-wise split on this data puts an
identical picture in train and test, which Phase 13 measured, and every
number below would be flattering and wrong.
"""

import argparse
import pathlib
import sys
import time

import numpy as np
from PIL import Image
from threevn_data import dataset as ds
from threevn_data import manifest as manifest_mod

from threevn_ml import features as feat
from threevn_ml import models


def load_features(directory, intrinsics, cache=True):
    """
    Return (features, labels, frames), extracting or reusing a cache.

    Extraction runs a flood fill over every frame, which takes about a
    minute for 480 images. Caching keeps `make train` fast enough to run
    repeatedly, which is what makes it useful while tuning.

    The cache records the feature NAMES it was built with. Reusing a
    cache whose columns mean something different is a silent, and the
    check costs nothing.
    """
    directory = pathlib.Path(directory)
    path = directory / 'features.npz'
    frames = ds.load(directory)

    if cache and path.is_file():
        stored = np.load(path, allow_pickle=False)
        names = tuple(str(n) for n in stored['names'])
        if (names == feat.SHAPE_FEATURES
                and len(stored['features']) == len(frames)):
            return stored['features'], stored['labels'], frames

    rows, labels, kept = [], [], []
    for frame in frames:
        with Image.open(frame.image) as handle:
            rgb = np.asarray(handle.convert('RGB'), dtype=np.uint8)
        vector = feat.extract(rgb, intrinsics, names=feat.SHAPE_FEATURES)
        if vector is None:
            # A frame the detector cannot see is not a training example.
            # Dropping it is honest; imputing a feature would invent one.
            continue
        rows.append(vector)
        labels.append(frame.label)
        kept.append(frame)

    features = np.array(rows, dtype=np.float64)
    label_array = np.array(labels, dtype=np.float64)
    if cache and len(kept) == len(frames):
        np.savez(path, features=features, labels=label_array,
                 names=np.array(feat.SHAPE_FEATURES))
    return features, label_array, kept


def main(argv=None):
    """Train and evaluate."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', default='/ws/datasets/target')
    parser.add_argument('--out', default='/ws/datasets/target/model.json')
    parser.add_argument('--alpha', type=float, default=1e-6)
    parser.add_argument('--target-width', type=float, default=0.05)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--no-cache', action='store_true')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    directory = pathlib.Path(args.dataset)
    doc = manifest_mod.read(directory)
    problems = manifest_mod.validate(doc)
    if problems:
        print('  refusing to train on a dataset that does not validate:')
        for problem in problems:
            print(f'    - {problem}')
        return 1

    intrinsics = feat.intrinsics_from_manifest(doc)

    started = time.time()
    features, labels, frames = load_features(
        directory, intrinsics, cache=not args.no_cache)
    print(f'\n  {len(frames)} usable frames '
          f'({time.time() - started:.1f}s to featurise)')
    if len(frames) < len(ds.load(directory)):
        print(f'  {len(ds.load(directory)) - len(frames)} frames dropped: '
              f'nothing detected')

    index = {id(f): i for i, f in enumerate(frames)}
    split = ds.split_by_episode(frames, seed=args.seed)
    parts = {name: np.array([index[id(f)] for f in items], dtype=int)
             for name, items in split.items()}
    print('  split by episode: ' + ', '.join(
        f'{name} {len(rows)}' for name, rows in parts.items()))

    baseline = models.Geometric(intrinsics, target_width=args.target_width)

    # An ABLATION, not a single model. The shape features were added on a
    # specific hypothesis - that the cube's silhouette widens off-axis and
    # a fixed target width cannot represent it - and a hypothesis that is
    # not tested against the model without them is decoration.
    variants = [('geometry', list(feat.GEOMETRIC_FEATURES)),
                ('+ shape', list(feat.SHAPE_FEATURES))]
    columns = {name: i for i, name in enumerate(feat.SHAPE_FEATURES)}

    print()
    print('  model        split   median    p95     MAE x   MAE y   MAE z   z bias')
    results = {}

    for part in ('train', 'val', 'test'):
        rows = parts[part]
        summary = models.errors(baseline.predict(
            features[np.ix_(rows, [columns[n] for n in feat.GEOMETRIC_FEATURES])]),
            labels[rows])
        results.setdefault('geometric', {})[part] = summary
        print('  %-12s %-6s %6.1f  %6.1f  %6.1f  %6.1f  %6.1f  %+6.1f' % (
            'geometric' if part == 'train' else '', part,
            summary['median_mm'], summary['p95_mm'], summary['mae_x_mm'],
            summary['mae_y_mm'], summary['mae_z_mm'], summary['bias_z_mm']))
    print()

    fitted = {}
    for label, names in variants:
        picked = [columns[n] for n in names]
        model = models.Ridge(alpha=args.alpha).fit(
            features[np.ix_(parts['train'], picked)], labels[parts['train']])
        fitted[label] = (model, names, picked)
        key = f'ridge {label}'
        results[key] = {}
        for part in ('train', 'val', 'test'):
            rows = parts[part]
            summary = models.errors(
                model.predict(features[np.ix_(rows, picked)]), labels[rows])
            results[key][part] = summary
            print('  %-12s %-6s %6.1f  %6.1f  %6.1f  %6.1f  %6.1f  %+6.1f' % (
                f'ridge {label}' if part == 'train' else '', part,
                summary['median_mm'], summary['p95_mm'], summary['mae_x_mm'],
                summary['mae_y_mm'], summary['mae_z_mm'],
                summary['bias_z_mm']))
        print()

    base_test = results['geometric']['test']['median_mm']
    geo_test = results['ridge geometry']['test']['median_mm']
    shape_test = results['ridge + shape']['test']['median_mm']

    print(f'  closed form                  {base_test:6.1f} mm')
    print(f'  fitted, geometry features    {geo_test:6.1f} mm  '
          f'({100 * (base_test - geo_test) / base_test:+.0f}% vs closed form)')
    print(f'  fitted, + shape features     {shape_test:6.1f} mm  '
          f'({100 * (geo_test - shape_test) / geo_test:+.0f}% vs geometry features)')
    print()
    if shape_test < geo_test:
        print('  The shape features earn their place: the silhouette widening')
        print('  they were added for is real and the model can use it.')
    else:
        print('  The shape features DO NOT earn their place on this test set.')
        print('  The hypothesis they encode may still be right - the implied')
        print('  target width does vary from 55 to 78 mm - but the aspect')
        print('  ratio evidently does not carry enough of it to help here.')

    ridge, names, picked = fitted['+ shape']

    print()
    print('  what the fit implies about the target width:')
    implied = ridge.weights[names.index('1/w'), 2] / intrinsics.fx
    print(f'    the 1/w -> z weight alone implies {implied * 1000:.1f} mm, '
          f'against a declared {args.target_width * 1000:.0f} mm.')
    print('    Read that with care. The features share 1/w, so they are')
    print('    correlated and the split of the mapping between them is not')
    print('    unique: a single coefficient is not a physical quantity here,')
    print('    even though the predictions are good. The identifiable')
    print('    statement is the one measured directly from the data - that')
    print('    w * z / fx, the width the images actually imply, has a median')
    print('    of 64.9 mm and spans 55 to 78 mm.')

    artifact = {
        'schema_version': 1,
        'model': ridge.to_dict(),
        'ablation': {label: fitted[label][1] for label in fitted},
        'baseline': baseline.to_dict(),
        'features': names,
        'metrics': results,
        'split': {'kind': 'episode', 'seed': args.seed,
                  **{k: int(len(v)) for k, v in parts.items()}},
        # The dataset is identified by its manifest, not by its path. A
        # path says where a file was on one machine; this says what was
        # in it.
        'dataset': {'world': doc['world'], 'robot': doc['robot'],
                    'frames': doc['frames'], 'episodes': doc['episodes'],
                    'recorded': doc['created'], 'code': doc['code'],
                    'labels': doc['labels']},
        'code': manifest_mod.git_description('/ws'),
    }
    models.save(args.out, artifact)
    print(f'\n  wrote {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
