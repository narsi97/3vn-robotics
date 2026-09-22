# ADR-0001: ROS package naming — the `threevn_` prefix

**Status:** accepted · **Date:** 2026-09-22

## Context

The brand is "3VN Robotics". The obvious package names — `3vn_robot_description`,
`3vn_bringup` — are **illegal**.

[REP-144](https://ros.org/reps/rep-0144.html) requires a package name to
consist of lowercase alphanumerics and underscores and to **start with an
alphabetic character**, because the name becomes a C++ namespace, a Python
module name and a CMake target, none of which may begin with a digit.

The same constraint binds more than package names. ROS 2 topic, node,
namespace and parameter name tokens map onto DDS entity names, which also
forbid a leading digit in a token. So a configuration value like
`model: 3vn_arm_v1` is a **latent runtime failure**: it parses fine as a
YAML string and works right up until something derives a namespace, a
controller name or a TF frame prefix from it, at which point it fails
inside DDS with an opaque error far from its cause.

## Decision

ROS packages use the **`threevn_`** prefix: `threevn_robot_description`,
`threevn_bringup`, `threevn_sim`.

The robot model identifier is `threevn_arm_v1`, with a separate
`display_name: "3VN Arm v1"` for documentation and UI. `test_config_schema.py`
asserts the model identifier matches `[a-z][a-z0-9_]*` so this cannot
regress.

The brand remains visible everywhere a leading digit is legal:

| Surface | Value |
|---|---|
| Repository / GitHub | `3vn-robotics` |
| Docker image | `threevn-robotics-dev` |
| Docs, README, course | 3VN Robotics |
| Config `display_name` | "3VN Arm v1" |
| ROS packages, model id | `threevn_*` |

## Alternatives considered

- **`tvn_`** — "3VN" is a stylisation of **T**ri**V**e**N**i, so `tvn` is
  the same consonant skeleton in a legal character set, and three
  characters keeps topic names short. Rejected in favour of `threevn_`
  because spelling the number keeps the brand legible to a reader who has
  not been told the Triveni connection.
- **`robot3vn_`** — leads with "robot", which is meaningless when every
  package is a robot package, and buries the brand mid-token.

## Consequences

Topic and node names are slightly longer. Renaming ROS packages later is
expensive (every `package.xml`, `CMakeLists.txt`, import and launch
reference), which is why this is settled once, now, rather than left to
drift.
