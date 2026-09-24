# MLOps

```bash
make registry-register     # store the trained artifact
make registry-promote      # run the gates, then promote
make registry-status       # what is registered, what is running
make serve                 # run whatever is in production
make registry-rollback     # put the previous one back
```

## A model is not a file

It is a file, the data that produced it, the metrics that justify it,
and a contract with the code that will run it. The registry stores all
four together and refuses anything missing one.

The storage is a directory and one pointer — no database, no service, no
daemon. The whole state is readable with `cat`, survives the container,
and copies to a robot with `scp`. Three properties are borrowed from
real registries because they earn their keep:

| | |
|---|---|
| versions are **immutable** | a promoted model cannot change under a running robot; re-registering identical bytes returns the existing version |
| production is a **pointer** | promotion and rollback are the same operation, so rollback is as well-tested as promotion |
| history is **append-only** | "what was running last Tuesday" is the first question after a failure |

Versions are content-addressed. A counter is state that can disagree
with the files; a hash cannot.

## Refusing is the feature

A registry that accepts whatever it is given is a directory with extra
steps. Every promotion runs five gates and reports **all** failures, not
the first — a model failing three checks should be fixed once rather
than three times, each after another training run.

| gate | what goes wrong without it |
|---|---|
| `validates` | an artifact without metrics cannot be compared with anything, now or in six months |
| `provenance` | a robot running an untraceable model cannot be explained after an incident |
| `feature_contract` | the serving code computes features in a fixed **order**; a model fitted on a different one still produces numbers |
| `beats_baseline` | Phase 12 solves this in closed form. A model that cannot beat it is strictly worse than code needing no data, no training and no registry |
| `no_regression` | the model already running is the thing to beat, not the baseline it beat long ago |

Demonstrated, not asserted — a deliberately degraded model was
registered and refused:

```
    ok    validates
    ok    provenance
    ok    feature_contract
    ok    beats_baseline
    FAIL  no_regression
          test median 95.0 mm is worse than the model in production
          at 10.5 mm by more than 5%

  93348d41e4d8 NOT promoted.
```

`no_regression` is a **tolerance, not a wall** (5%). Accepting a
slightly worse median for a gain elsewhere is sometimes right, and a
gate that forbids it gets bypassed routinely — which costs the gate its
meaning for the cases that matter. `--force` promotes anyway and records
the promotion **as forced**, because a log saying a model was promoted
while omitting that every gate objected has lost the most useful fact
about it.

## Serving knows nothing about which model it runs

`make serve` loads whatever the registry points at. Swapping models is
`promote` plus a restart, never an edit — so what changes between a good
deployment and a bad one is data in a directory, not code nobody
reviewed.

It **refuses to start** when nothing is promoted, rather than picking a
file. Picking one is exactly the untracked deployment the registry
exists to prevent.

It also **refuses to start** on a feature-contract mismatch. Checking at
load turns a subtle wrongness into a loud failure before anything moves.

### It publishes what the physics would have said

`/ml/target_pose` is the model. `/ml/target_pose_geometric` is the
closed form, on its own topic, costing one multiplication.

On a robot there is no ground truth, so a learned model that has drifted
away from the physics is otherwise invisible until something misses.
Live, on the simulator:

```
  model  x=+0.005 y=-0.053 z=+0.387
  geom   x=-0.000 y=-0.049 z=+0.360
```

A 28 mm gap — and the model is the one closer to truth, which at this
pose is about 0.390 m. That is the correction Phase 14 fitted, visible
at runtime.

## Drift: noticing the question changed

A model has no way to say *I have not seen anything like this*. It
returns a number for whatever it is given with the same confidence for a
scene it was fitted on and one it has never met. On a robot that is the
dangerous failure — not a crash, but a plausible answer that moves an
arm to the wrong place.

The monitor compares live features against the training distribution
that Phase 14 records in the artifact, reporting a per-feature z-score
and anything outside the training range entirely. It names the feature
responsible: *"something is unusual"* sends someone looking, *"1/w is
nine sigma out"* says the target is a size the model never saw.

Deliberately simple — no distribution distance, no windowed test. Those
need tuning this project has no data to justify, and **an untuned
detector that fires constantly gets switched off, taking the real alarms
with it**. A feature with zero training spread is treated as *cannot
judge* rather than *always drifting*, for the same reason.

It cannot tell you the answer is wrong. Only that the question changed,
which is when a wrong answer becomes likely.

## The same mistake, three times

Which metrics entry corresponds to the stored model was implemented
independently in the gates, the registry summary and the serving
banner. Two of the three guessed it by taking the alphabetically last
key — and with an ablation recording `ridge geometry` and
`ridge + shape`, that picks the wrong one, because `+` sorts before `g`.

The registry reported a model scoring **10.5 mm as scoring 46.5 mm**.
Nothing failed. The number was just wrong, which is worse.

The artifact now *names* its selected entry and `gates.selected_key` is
the single answer. This is the third time in this project that one
decision living in three places produced the same bug three times — see
`scripts/profile_entry.py` from Phase 12.

## What is not here, and why

- **CI does not train or gate.** The dataset is 5 MB of PNGs that are
  correctly not in git, so CI has no data to train on. The gates run
  where the data is. A real setup puts the dataset behind DVC, S3 or an
  artifact store and trains in CI against a pinned version; that is the
  right shape and it is not built here.
- **No canary or shadow deployment.** Production is one pointer. Serving
  two models and comparing them live is the natural next step, and the
  geometric topic is a crude version of it.
- **No automatic rollback.** Drift is reported to the log, not acted on.
  A robot that rolls back its own model without a human is a robot that
  can oscillate between two models in a loop.
- **Metrics are not wired to Phase 9.** The drift rate and the
  model-versus-geometry gap are exactly the numbers that belong on the
  OpenTelemetry stack, and connecting them is a small job that is not
  done.
- **Nothing has been deployed to real hardware**, because no robot has
  been built.
