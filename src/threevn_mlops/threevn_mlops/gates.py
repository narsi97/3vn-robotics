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
The conditions a model has to meet before it drives anything.

REFUSING IS THE FEATURE. A registry that accepts whatever it is given is
a directory with extra steps; the value is in the checks, and in their
being the same checks every time rather than whatever the person
promoting remembered to look at.

Each gate exists because of something that can actually go wrong:

    validates        an artifact missing its metrics cannot be compared
                     with anything, now or in six months
    beats_baseline   Phase 12 solves this task in closed form. A model
                     that does not beat it is strictly worse than code
                     that needs no data, no training and no registry
    no_regression    the model already in production is the thing to
                     beat, not the baseline it beat long ago
    feature_contract the serving code computes features in a fixed
                     order. A model fitted on a different order still
                     produces numbers
    provenance       a model that cannot name the data and commit it
                     came from cannot be reproduced, and a robot running
                     one cannot be explained

Gates return reasons rather than raising, so a promotion attempt reports
everything wrong at once instead of one thing per run.
"""

REQUIRED_KEYS = ('schema_version', 'model', 'selected', 'features',
                 'metrics', 'dataset', 'code', 'feature_stats')

#: Promoting a model that is worse than the one running is sometimes
#: right - a small loss for a large gain elsewhere - so this is a
#: tolerance rather than a wall. Above it the promotion needs `--force`
#: and says so in the log.
REGRESSION_TOLERANCE = 1.05


def validates(artifact):
    """Return reasons the artifact is not a usable model record."""
    problems = []
    missing = [key for key in REQUIRED_KEYS if key not in artifact]
    if missing:
        return [f'artifact is missing {sorted(missing)}']

    if artifact['schema_version'] != 1:
        problems.append(
            f'schema_version {artifact["schema_version"]} is not 1')
    for split in ('train', 'test'):
        for name, metrics in artifact['metrics'].items():
            if split not in metrics:
                problems.append(f'{name} has no {split} metrics')
            elif 'median_mm' not in metrics[split]:
                problems.append(f'{name}/{split} has no median_mm')
    if not artifact['features']:
        problems.append('no feature names recorded')
    return problems


def beats_baseline(artifact):
    """
    Return reasons the model does not beat the closed form on test.

    The baseline is in the artifact because Phase 14 evaluated both in
    one run. Recomputing it here would be a second implementation and a
    second chance to disagree.
    """
    metrics = artifact['metrics']
    model_key = selected_key(artifact)
    if model_key is None:
        return ['the artifact does not say which metrics entry its stored '
                'model corresponds to; add a `selected` field naming one']
    if 'geometric' not in metrics:
        return ['no baseline metrics in the artifact: nothing to beat']

    model = metrics[model_key]['test']['median_mm']
    baseline = metrics['geometric']['test']['median_mm']
    if model >= baseline:
        return [f'test median {model:.1f} mm does not beat the closed form '
                f'at {baseline:.1f} mm; the geometry needs no data, no '
                f'training and no registry']
    return []


def no_regression(artifact, incumbent, tolerance=REGRESSION_TOLERANCE):
    """Return reasons this model is worse than the one in production."""
    if incumbent is None:
        return []
    new = selected_key(artifact)
    old = selected_key(incumbent)
    if new is None or old is None:
        return ['cannot compare: one side does not name its selected model']

    new_score = artifact['metrics'][new]['test']['median_mm']
    old_score = incumbent['metrics'][old]['test']['median_mm']
    if new_score > old_score * tolerance:
        return [f'test median {new_score:.1f} mm is worse than the model in '
                f'production at {old_score:.1f} mm by more than '
                f'{(tolerance - 1) * 100:.0f}%']
    return []


def feature_contract(artifact, served_features):
    """
    Return reasons the serving code cannot run this model.

    Order is part of the contract, not a detail. A model fitted on
    columns in one order and fed them in another produces predictions,
    not an error, and the predictions are wrong in a way that looks like
    a badly trained model rather than a bug.
    """
    recorded = list(artifact['features'])
    served = list(served_features)
    if recorded == served:
        return []
    if sorted(recorded) == sorted(served):
        return [f'feature ORDER differs: model expects {recorded}, serving '
                f'computes {served}. Same names, different columns - this '
                f'would run and be silently wrong']
    return [f'feature mismatch: model expects {recorded}, serving computes '
            f'{served}']


def provenance(artifact):
    """Return reasons the model cannot be traced to what produced it."""
    problems = []
    if not artifact.get('code', {}).get('commit'):
        problems.append('no commit recorded for the training run')
    dataset = artifact.get('dataset', {})
    if not dataset.get('code', {}).get('commit'):
        problems.append('no commit recorded for the dataset')
    if not dataset.get('frames'):
        problems.append('no frame count recorded for the dataset')
    if dataset.get('labels', {}).get('source') != 'simulator':
        # Not a failure. A model trained on human-labelled or real data
        # is fine and this says so out loud, because the deployment
        # consequences differ.
        pass
    return problems


def check_all(artifact, served_features, incumbent=None, tolerance=None):
    """Run every gate and return {gate: [reasons]} for the ones that fail."""
    results = {
        'validates': validates(artifact),
        'provenance': provenance(artifact),
        'feature_contract': feature_contract(artifact, served_features),
        'beats_baseline': beats_baseline(artifact),
        'no_regression': no_regression(
            artifact, incumbent,
            tolerance if tolerance is not None else REGRESSION_TOLERANCE),
    }
    return {gate: reasons for gate, reasons in results.items() if reasons}


def selected_key(artifact_or_metrics):
    """
    Return the metrics key for the model that was actually saved.

    THE ONE ANSWER. This decision was independently reimplemented three
    times - in the gates, in the registry summary, and in the serving
    node's startup log - and two of the three got it wrong the same way.

    Taken from the artifact's `selected` field. It was once guessed by
    sorting the keys and taking the last, which picked the wrong entry of
    an ablation - `'ridge geometry'` over `'ridge + shape'`, because '+'
    sorts before 'g' - and reported a model as four times worse than it
    was. A guess that is right for the examples to hand is still a guess.
    """
    if isinstance(artifact_or_metrics, dict) and 'metrics' in artifact_or_metrics:
        selected = artifact_or_metrics.get('selected')
        metrics = artifact_or_metrics['metrics']
        if selected and selected in metrics:
            return selected
    else:
        metrics = artifact_or_metrics
    keys = [k for k in metrics if k != 'geometric']
    if len(keys) == 1:
        return keys[0]
    return None
