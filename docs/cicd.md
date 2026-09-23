# CI/CD strategy

`.github/workflows/ci.yml` — the first GitHub Actions workflow in the 3VN
estate.

## Principle: CI runs what you run

CI builds the same `docker/Dockerfile` (the `ci` stage) and executes the
same commands as your machine. A green tick means *the thing you ran
locally* passed, not that a parallel pipeline happens to agree.

## Two architectures

| Job | Runner |
|---|---|
| amd64 | `ubuntu-24.04` |
| arm64 | `ubuntu-24.04-arm` |

Development happens on Apple Silicon; most servers are x86. Running both
is the only way to catch divergence in either direction, and arm64 runners
are free on public repos — which is one reason this repo is public.

## What CI asserts beyond "tests pass"

Three checks encode decisions that would otherwise silently rot:

1. **No mixed Gazebo installation.** Fails if any `gz-*` package resolves
   from `osrfoundation`. Mixing repos gives incompatible `libgz-*.so` and
   a segfault far downstream.
2. **`xacro.load_yaml` still exists.** The single-source-of-truth design
   depends on it. If a future xacro removes it, fail here rather than in a
   confusing expansion error.
3. **The hardware seam is intact.** A `mock` or `esp32` expansion must
   contain no Gazebo reference; a `gz` expansion no ESP32 reference. This
   is the project's central architectural claim, checked as a build
   artifact rather than only as a unit test.

Plus the C++ `check_urdf` cross-check over every profile × target, and the
expanded URDFs uploaded as an artifact so a reviewer can diff the actual
robot a PR produces.

## The five workflows

| Workflow | Trigger | What it does |
|---|---|---|
| `ci.yml` | push, PR | lint, build, 300+ tests, on amd64 **and** arm64 |
| `simulation-tests.yml` | push, PR touching sim | Gazebo integration + acceptance report |
| `docker.yml` | push, tag | builds the runtime image natively per arch, verifies it, publishes to GHCR |
| `release.yml` | tag `v*` | version manifest + GitHub release |
| `hardware-deploy.yml` | **manual only** | deploys to a robot, behind an approval gate |

## Releases are a tag plus evidence

`release.yml` refuses to run if the tag and `VERSION` disagree — a
release whose tag and contents differ is worse than no release, because
every artifact it produces is mislabelled.

It does **not** build its own image. It resolves the one `docker.yml`
already built and verified from that commit, so a release can only ship
something that was actually tested. If the image is not in the registry,
the release fails and says to tag a commit that has been through CI.

The manifest records every version a deployed robot can be asked about —
software, firmware, protocol, commit, joint count, heartbeat timeout —
and the per-architecture digests. Pull by digest for anything that must
be reproducible: a tag can be moved, a digest cannot.

## Hardware deployment is gated, and honest

`hardware-deploy.yml` is `workflow_dispatch` only. Spec §13 is explicit:
do not deploy to hardware automatically. **A robot that moves on every
merged PR is a hazard, not a pipeline** — the thing being updated has
motors, and someone may be standing next to it.

Three gates, deliberately:

1. You must type `DEPLOY` to confirm the arm may move.
2. A GitHub Environment named `robot` requires a **human reviewer**.
3. The tag is resolved to an immutable **digest before** approval, so a
   tag moving between approval and deploy cannot change what lands. What
   was approved is what ships.

The deploy pulls before stopping the running container, so a slow or
failed download never leaves a robot down, and waits on `/readyz` rather
than `/healthz` — liveness only says the process answers.

**It has never run against hardware, because no arm exists.** It fails at
the SSH step with an explanation, which is correct: a deploy pipeline
that reports success against nothing is worse than one that admits it
cannot proceed.

## One CI system, not two

## One CI system, not two

The 3VN estate has a CircleCI config on Interest Optimizer whose deploy
job was never activated. It stays where it is. Running two CI systems
means two places to look when something breaks and two sets of secrets to
rotate.

The shared plan is GitHub Actions **reusable workflows** in a `3vn-ci`
repo, so every product calls one line rather than copying a pipeline.
Robotics is the first consumer — it needs CI in Phase 0 anyway — which
validates the pattern on one repo before it touches the other five.
