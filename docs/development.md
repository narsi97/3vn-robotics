# Development

Everything runs in one container. You need Docker and nothing else.

```bash
make              # command list
make shell        # shell inside the container, ROS sourced
make build        # colcon build --symlink-install
make test         # lint + unit
make urdf         # print the expanded URDF
make view         # RViz -> http://localhost:8106/vnc.html
```

## The edit loop

`--symlink-install` means the installed files are symlinks back into
`src/`, so **editing a YAML, xacro, launch file or RViz config takes
effect with no rebuild** — relaunch and you see the change.

**Adding a file still needs `make build`**, because the symlink for it
does not exist yet. This is the single most common "why didn't my change
apply?" in this repo.

## Changing the robot

Never edit a dimension in a `.xacro`. Every physical number lives in
`src/threevn_robot_description/config/threevn_arm_v1.yaml`:

```bash
vim src/threevn_robot_description/config/threevn_arm_v1.yaml
make test        # limits, inertia and schema tests run against the change
make view        # look at it
```

To try a variant without touching the default, copy the YAML and pass it:

```bash
make urdf PROFILE=threevn_arm_v1_long_reach
make view PROFILE=threevn_arm_v1_long_reach
```

Every test automatically parametrizes over every `config/threevn_*.yaml`,
so a new profile widens the test matrix for free.

## Adding a link or joint

1. Add it to `config/*.yaml` under `links:` / `joints:` / `frames:`, with
   a `provenance:` field on any mass.
2. Instantiate it in the relevant macro under `urdf/arm/`.
3. If it should be actuated, add a `threevn_joint_ifaces` line in
   `urdf/ros2_control/arm.ros2_control.xacro` and list it in
   `threevn_bringup/config/controllers.yaml`.
4. `make build && make test`.

If you added a required frame, add it to `REQUIRED_FRAMES` in
`test_urdf_structure.py` so its deletion fails a test later.

## Style

ROS house conventions, enforced by `make lint`:

- **Single quotes** for Python strings (flake8-quotes). Use double quotes
  only when the content contains a single quote.
- **Docstring summary on the second line** (D213 — `ament_pep257` ignores
  D212 and enforces D213).
- Apache-2.0 copyright header on every Python file.
- Max line length 99.
- Imports alphabetical within the group (`import-order-style = google`).

Two XML traps worth knowing:

- **`--` is illegal inside an XML comment.** Use a single hyphen. This
  fails with a bare "not well-formed (invalid token)" that does not
  mention comments.
- **Use `${'$(arg target)' == 'gz'}`, not `${target == 'gz'}`.** The
  latter substitutes a bare token and raises `NameError`.

## Ports

8106 is this project's. 8083/8085/8087/8100/8102/8104 belong to the other
3VN products.

## Cleaning up

```bash
make clean   # build/install/log
make nuke    # + image, volumes, build cache
```

The image is 5–6 GB and build cache grows quickly; `docker system df`
shows where the space went.
