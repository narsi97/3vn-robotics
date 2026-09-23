# Testing

```bash
make test        # lint + unit, no ROS at all        ~11s
make test-ros    # real ROS graph, mock target       ~135s
make test-sim    # Gazebo physics                    ~100s
make acceptance  # end-to-end sequence + report      ~70s
make lint        # linters + check_urdf, every profile
```

Four tiers, each earning its place by what it can catch that the tier
below cannot:

| Tier | Needs | Catches |
|---|---|---|
| `make test` | nothing | config, structure, kinematics, inertia, readiness logic, the hardware seam |
| `make test-ros` | a ROS graph | node startup, interface contracts, **limit enforcement**, TF, FK-vs-KDL |
| `make test-sim` | Gazebo | spawn, controller activation under physics, real-time factor |
| `make acceptance` | Gazebo + dashboard | the whole sequence, with a report naming the build |

**The 60-second budget is a design constraint, not an aspiration.** A
suite that takes minutes is a suite people stop running before they
commit. Everything that needs a simulator is marked `slow` and excluded
from `make test`.

Current: **~215 fast tests in ~11 s**, **15 ROS integration tests**,
**12 Gazebo integration tests**, and a passing end-to-end acceptance run.

## Layers

| Layer | Needs | Where |
|---|---|---|
| Config schema | nothing | `test_config_schema.py` |
| Description structure | nothing | `test_urdf_structure.py`, `test_inertia_valid.py` |
| Config↔URDF consistency | nothing | `test_limits_match_yaml.py` |
| **Architecture** | nothing | `test_ros2_control_targets.py`, `test_package_dependencies.py` |
| Independent parser | nothing | `scripts/check_description.sh` |
| Scenario framework | nothing | `threevn_control/test/` |
| Forward kinematics | nothing | `threevn_control/test/test_kinematics.py` |
| Readiness logic | nothing | `threevn_dashboard/test/test_robot_state.py` |
| HTTP contracts | nothing | `threevn_dashboard/test/test_http_api.py` |
| ROS interfaces, limits | ROS graph | `threevn_bringup/test/test_ros_integration.py` |
| TF, FK vs KDL | ROS graph | `threevn_bringup/test/test_tf_matches_kinematics.py` |
| Simulation assets | nothing | `threevn_sim/test/test_world.py` |
| Gazebo integration | simulator | `threevn_sim/test/test_gz_spawn.py` |

Note how much is provable with **no simulator and no hardware**. That is
deliberate: those tests run in a second, so they run constantly.

## The tests that matter most

**`test_ros2_control_targets.py`** asserts that a `mock` or `esp32`
expansion contains no `gz_ros2_control` and no `<gazebo>` tag, and that a
`gz` expansion contains no `Esp32SystemInterface`. That assertion *is* the
architecture. If it fails, the application has become coupled to its
execution target.

**`test_package_dependencies.py`** parses `threevn_bringup/package.xml`
and fails on any Gazebo dependency. It also sanity-checks its own matcher,
so it cannot pass by failing to match anything.

**`test_inertia_valid.py`** checks each tensor is symmetric, positive
definite, and satisfies the triangle inequality on its principal moments.
An invalid inertia does not error at load time — it produces NaN on first
contact, which looks like "Gazebo is unstable" and costs hours. Catching
it with no simulator running is the cheapest possible place.

**`test_limits_match_yaml.py`** is non-tautological by design. Revolute
limits are stored in degrees and cross a deg→rad conversion at two
independent sites (the `<limit>` tag and the `<command_interface>`
params). The test checks both against the source YAML.

## Two parsers, on purpose

`urdf_parser_py` (Python, in the unit tests) and `check_urdf` (C++, in
`scripts/check_description.sh`) have different leniencies. Running both
means a disagreement between them is itself a finding.

One trap they expose: `test_no_duplicate_link_names` checks **raw XML**,
not the parsed model. `urdf_parser_py` builds a dict keyed by name, so a
duplicated link silently overwrites the first and the parsed tree looks
perfectly healthy. The duplicate is only visible before parsing.

## Every test runs against every profile

Tests parametrize over `PROFILES` — every `config/threevn_*.yaml` — and
over all three hardware targets. Adding a profile automatically widens the
matrix; nothing needs updating.

## Two implementations that must agree

`threevn_control/kinematics.py` computes forward kinematics with its own
matrix arithmetic. `robot_state_publisher` computes the same transforms
through KDL. `test_tf_matches_kinematics.py` asserts the two agree to a
millimetre, at whatever pose the robot happens to be in.

That is worth more than either alone. A kinematics module tested only
against its own arithmetic proves the arithmetic is self-consistent —
including when it is consistently wrong. It also earned its keep
immediately: it found that the camera mount was tilted **up at the
ceiling** rather than down at the workspace, before any camera existed.

## Why `make test` does not run the Gazebo tests

Not by a pytest marker alone. `colcon test --pytest-args -m "not slow"`
reaches pytest only for `ament_python` packages; in an `ament_cmake`
package pytest is launched by CTest, which never sees the flag — so
`make test` would quietly spend two minutes starting a simulator.

`threevn_sim/CMakeLists.txt` therefore registers **only**
`test/test_world.py` with CTest, and `make test-sim` invokes pytest
directly with `-m slow`. Explicit beats clever here.

## Scenarios are tests and demos

The six scenarios in `threevn_control` are written against `RobotClient`
alone, so they run unchanged against `mock`, against Gazebo and, from
Phase 5, against the physical arm.

```bash
make sim &                      # or: make mock &
make scenario NAME=pick_and_place
make scenario NAME=all
```

`make test-sim` runs the same six against Gazebo and fails the build on
any of them. Writing them once for both is what stops the demo drifting
away from the thing that is actually verified.

**A scenario must pass on both targets.** One that passes under Gazebo
but not under mock means the abstraction has become simulator-specific —
which is precisely how `safety_limit` uncovered that nothing below the
hardware seam enforces joint limits. See [safety.md](safety.md).

## Writing a new test

Use the `expanded` fixture from `conftest.py`. It expands the same xacro
entry point, with the same arguments, that `ros2 launch` uses at runtime,
and memoises the result. A test that built its URDF some other way would
be testing itself.

```python
def test_something(expanded):
    robot = URDF.from_xml_string(expanded(target='mock'))
    ...
```
