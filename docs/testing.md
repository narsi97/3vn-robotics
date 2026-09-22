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

Current: **122 tests, ~7 seconds.**

## Layers

| Layer | Needs | Where |
|---|---|---|
| Config schema | nothing | `test_config_schema.py` |
| Description structure | nothing | `test_urdf_structure.py`, `test_inertia_valid.py` |
| Config↔URDF consistency | nothing | `test_limits_match_yaml.py` |
| **Architecture** | nothing | `test_ros2_control_targets.py`, `test_package_dependencies.py` |
| Independent parser | nothing | `scripts/check_description.sh` |
| Gazebo integration | simulator | `threevn_sim/test/` — Phase 2 |

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
