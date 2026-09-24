# cad/

Parametric CAD: the printed parts, generated from the same YAML that
drives the simulation.

```bash
make cad                    # generate STL + STEP for both mechanisms
make cad-test               # geometry and printability checks
```

**Not a ROS package, and not under `src/`.** It imports `build123d`, not
`rclpy`, and nothing on the robot runs it. Keeping it out of `src/` keeps
it out of `colcon build`, so a workspace build does not depend on
OpenCascade.

It runs under `$CAD_PYTHON` (a venv in the image), because build123d
wants a newer `typing_extensions` than Debian ships and pip cannot
uninstall a dpkg-managed package. See the `cad` stage in
`docker/Dockerfile`.

## Why generated, not drawn

`threevn_arm_v1.yaml` already holds every length, and the URDF reads it
directly. If the CAD were drawn by hand in a GUI, the printed part and
the simulated model would be two independent statements about the same
robot, and they would disagree within a week — silently, because neither
knows about the other.

`test_matches_the_urdf.py` asserts they agree.
