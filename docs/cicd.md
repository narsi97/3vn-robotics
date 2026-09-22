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

## What is deliberately not here yet

**Gazebo integration tests.** They arrive in Phase 2 as
`simulation-tests.yml`, where simulation is the deliverable. Starting a
simulator takes tens of seconds; keeping it out of the main workflow keeps
PR feedback fast.

**Deployment.** Phases 6–8. When it comes:

```
git push → CI → test → image → registry → VPS → robot
```

with an **approval gate before anything reaches physical hardware**. A
robot that moves on every merged PR is a hazard, not a pipeline.

## One CI system, not two

The 3VN estate has a CircleCI config on Interest Optimizer whose deploy
job was never activated. It stays where it is. Running two CI systems
means two places to look when something breaks and two sets of secrets to
rotate.

The shared plan is GitHub Actions **reusable workflows** in a `3vn-ci`
repo, so every product calls one line rather than copying a pipeline.
Robotics is the first consumer — it needs CI in Phase 0 anyway — which
validates the pattern on one repo before it touches the other five.
