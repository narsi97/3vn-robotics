# Testing

```bash
make test       # lint + unit, no simulator.  Budget: under 60s
make test-sim   # Gazebo integration.         Slow, opt-in
make lint       # linters + check_urdf on every profile
```

**The 60-second budget is a design constraint, not an aspiration.** A
suite that takes minutes is a suite people stop running before they
commit. Everything that needs a simulator is marked `slow` and excluded
from `make test`.

Current: **166 fast tests in ~7 s**, plus **12 Gazebo integration
tests in ~106 s** under `make test-sim`.

## Layers

| Layer | Needs | Where |
|---|---|---|
| Config schema | nothing | `test_config_schema.py` |
| Description structure | nothing | `test_urdf_structure.py`, `test_inertia_valid.py` |
| Config↔URDF consistency | nothing | `test_limits_match_yaml.py` |
| **Architecture** | nothing | `test_ros2_control_targets.py`, `test_package_dependencies.py` |
| Independent parser | nothing | `scripts/check_description.sh` |
| Scenario framework | nothing | `threevn_control/test/` |
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
