# ADR-0005: YAML is the single source of physical truth

**Status:** accepted · **Date:** 2026-09-22

## Context

The specification requires that where exact physical values are unknown,
they are isolated into configuration; and that hardware-specific values
are not hardcoded throughout the application. It also asserts that xacro
cannot read YAML natively and proposes a generator script.

**That assertion is false.** `xacro/__init__.py` on the `ros2` branch
defines `load_yaml()` and exposes it into the expression symbol table, so
`${xacro.load_yaml(path)}` works inside any `${...}`, and `$(find pkg)`
paths resolve. This removes the entire generator-script design branch.

## Decision

The top-level xacro loads a YAML file and reads every physical value from
it. The **path** is the only parameter:

```xml
<xacro:arg name="params_file" default="$(find threevn_robot_description)/config/threevn_arm_v1.yaml"/>
<xacro:property name="cfg" value="${xacro.load_yaml('$(arg params_file)')}"/>
```

Swapping profiles is a one-argument change. Changing a number never
requires editing a xacro. `threevn_arm_v1_long_reach.yaml` exists
specifically to prove this: it shares every line of xacro with the
default profile and differs only in numbers.

### Alternatives rejected

- **Generator script producing a committed URDF.** A second artifact that
  drifts; someone will eventually edit the generated file, and the
  consistency test becomes vacuous. Expansion stays a gitignored CI
  artifact for humans who want plain XML.
- **Every value as a xacro argument.** Around 60 numbers for a 4-DOF arm.
  Does not survive the mobile base.
- **Load YAML in the launch file, pass the dict as one argument.**
  Serialising a dict through a string argument is brittle quoting, and it
  makes the xacro unusable standalone — which would break the tests.

### Units live in key names

Revolute limits are stored in **degrees** (`lower_deg`), because servo
datasheets and humans both work in degrees. The deg→rad conversion
happens inside xacro. This is deliberate: it makes
`test_limits_match_yaml.py` a *real* test rather than a tautology,
because the value crosses two independent conversion sites (the `<limit>`
tag and the `<command_interface>` params) on its way into the URDF.

## Consequences

- **Trap:** CMake's `xacro_add_files()` does not work with
  `xacro.load_yaml()` ([ros/xacro#298](https://github.com/ros/xacro/issues/298)).
  Expansion happens at launch and test time only; `CMakeLists.txt`
  installs directories and nothing else.
- `$(find ...)` resolves to `install/share`, not `src`. With
  `--symlink-install` the installed YAML symlinks back to `src`, so live
  editing works. Tests read the YAML from the **same share directory**
  xacro used, so a test and the URDF cannot disagree about which file is
  authoritative.
