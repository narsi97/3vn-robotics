# syntax=docker/dockerfile:1.7
#
# 3VN Robotics images.
#
# Multi-arch by construction. ros:jazzy-ros-base publishes a native
# linux/arm64 manifest, so this builds natively on an Apple Silicon Mac,
# natively on x86 CI, and natively on an arm64 SBC next to the robot. Do
# NOT add --platform: pinning one architecture forces QEMU on the others.
#
#   base     ROS 2 + ros2_control. No simulator, no GUI, no build tools.
#   simdeps  base + Gazebo Harmonic. Only things that simulate need this.
#   ci       simdeps + test tooling + software GL. What CI runs.
#   dev      ci + RViz + a virtual display over noVNC. What you run.
#   builder  simdeps + a built workspace, robot packages only.
#   runtime  base + that build. What ships to the robot.
#
# The split is not cosmetic. `runtime` inherits from `base`, which has no
# Gazebo -- and that is only possible because threevn_bringup and
# threevn_hardware genuinely do not depend on it. The architecture rule
# enforced since Phase 1 by test_package_dependencies.py is what lets the
# robot image drop the simulator entirely.
#
# It also settles a licence question: x11vnc (GPL-2.0) and websockify
# (LGPL-3.0) exist ONLY in `dev`. Nothing distributed to a robot contains
# them. See THIRD_PARTY.md.
ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------- base --
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

# Everything the ROBOT needs, and nothing else.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-urdfdom-py \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-tools \
      liburdfdom-tools \
 && rm -rf /var/lib/apt/lists/*

# `docker compose exec` bypasses ENTRYPOINT, so every `bash -lc` the
# Makefile runs has to find ROS on its own. A profile.d script covers
# both interactive and non-interactive login shells; without it,
# `make build` works and `make shell` mysteriously does not.
RUN printf '%s\n' \
  "source /opt/ros/${ROS_DISTRO}/setup.bash" \
  '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
  'true' \
  > /etc/profile.d/10-ros.sh

# ------------------------------------------------------------- simdeps --
FROM base AS simdeps
ARG ROS_DISTRO

# Gazebo Harmonic arrives through the ROS *vendor* packages on
# packages.ros.org, which are the officially paired Harmonic build for
# Jazzy. We deliberately do NOT add packages.osrfoundation.org: mixing
# the two repositories yields two incompatible sets of libgz-*.so, which
# apt resolves happily and which surfaces much later as a `gz sim`
# segfault rather than an apt error. CI asserts this.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-gz-ros2-control \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------ ci --
FROM simdeps AS ci
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pytest python3-pytest-cov python3-numpy python3-yaml \
      ros-${ROS_DISTRO}-ament-lint-auto \
      ros-${ROS_DISTRO}-ament-lint-common \
      ros-${ROS_DISTRO}-ament-cmake-pytest \
      ros-${ROS_DISTRO}-ament-cmake-gmock \
      ros-${ROS_DISTRO}-launch-testing \
      ros-${ROS_DISTRO}-launch-testing-ros \
      libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

# Mesa software rendering, so `gz sim --headless-rendering` finds an
# OpenGL implementation with no GPU and no X server.
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe

WORKDIR /ws
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]

# --------------------------------------------------------------- cad --
# Parametric CAD, so the same YAML that drives the simulation drives the
# printed part and the two cannot disagree.
#
# IN A VENV, not system Python. build123d wants a newer typing_extensions
# than Debian ships, and pip cannot uninstall a dpkg-managed package
# (no RECORD file), so a plain `pip install` fails part-way and leaves
# the system Python in a state neither tool owns. `--system-site-packages`
# keeps yaml, numpy and the ROS packages visible, so one interpreter has
# both.
#
# IT COSTS 760 MB, essentially all of it OpenCascade via cadquery-ocp,
# and there is no lighter path to real B-rep solids with STEP export.
# That is about 15% on top of a ~5 GB image and it is in the CI stage
# deliberately: CAD tests that only run on one laptop are CAD tests that
# rot, and the geometry checks here are exactly the kind that catch a
# part nobody can print.
FROM ci AS cad
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-venv python3-pip \
 && rm -rf /var/lib/apt/lists/* \
 && python3 -m venv --system-site-packages /opt/cadenv \
 && /opt/cadenv/bin/pip install --no-cache-dir --quiet build123d==0.13.0 \
 && /opt/cadenv/bin/python -c "from build123d import Box; v = Box(1,1,1).volume; assert abs(v - 1.0) < 1e-9, v"
# The smoke test uses a TOLERANCE. It first asserted the volume of a
# unit cube was exactly 1.0, which failed the build after a ten-minute
# install: OpenCascade integrates the solid and returns
# 0.9999999999999998. Exact float equality against a geometry kernel is
# wrong in the same way it is wrong anywhere else, and here it cost a
# full image rebuild to discover.
#
# Pinned: OpenCascade is exactly the kind of dependency where a minor
# bump silently changes a fillet and a part stops fitting.
ENV CAD_PYTHON=/opt/cadenv/bin/python

# ----------------------------------------------------------------- dev --
FROM cad AS dev
ARG ROS_DISTRO
ARG UID=10001

# RViz2 needs a real X server with OpenGL 3.3+. Xvfb + Mesa llvmpipe
# gives GL 4.5, which is why this works where XQuartz (indirect GLX,
# capped near 1.4) does not -- see ros2/rviz#929 and ADR-0002.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      xvfb x11vnc novnc websockify supervisor fluxbox \
      mesa-utils curl vim less bash-completion \
 && rm -rf /var/lib/apt/lists/* \
 && ln -sf vnc.html /usr/share/novnc/index.html

# Non-root numeric UID, per house style. Adapted for a dev container:
# unlike the app images this one needs a WRITABLE HOME, because colcon,
# rviz and the gz fuel cache all write there -- so `-H` would be wrong.
RUN groupadd -g ${UID} dev \
 && useradd -m -u ${UID} -g ${UID} -s /bin/bash dev \
 && mkdir -p /ws/src /ws/build /ws/install /ws/log \
 && chown -R ${UID}:${UID} /ws

COPY docker/supervisord.conf /etc/supervisor/supervisord.conf
USER ${UID}
WORKDIR /ws
ENV HOME=/home/dev \
    DISPLAY=:1

# ------------------------------------------------------------- builder --
FROM simdeps AS builder
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-numpy python3-yaml \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /ws/src

# ONLY the packages the robot runs.
#
# threevn_sim is excluded deliberately, and excluding it is what lets the
# runtime image drop Gazebo. It works because threevn_bringup and
# threevn_hardware have no simulator dependency -- the property
# test_package_dependencies.py has been asserting since Phase 1.
#
# --merge-install produces one flat prefix rather than one per package,
# which is smaller and simpler to copy into the runtime stage.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
      --merge-install \
      --packages-up-to threevn_bringup threevn_hardware threevn_dashboard \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# ------------------------------------------------------------- runtime --
FROM base AS runtime
ARG ROS_DISTRO
ARG UID=10001

# Version identity, injected at build time. The dashboard reports these
# at /api/version, which is what ties an observed behaviour back to a
# commit. Defaults are honest rather than plausible: a robot that cannot
# say what it is running should say so.
ARG THREEVN_GIT_COMMIT=unknown
ARG THREEVN_GIT_BRANCH=unknown
ARG THREEVN_BUILD_TIME=unknown
ENV THREEVN_GIT_COMMIT=${THREEVN_GIT_COMMIT} \
    THREEVN_GIT_BRANCH=${THREEVN_GIT_BRANCH} \
    THREEVN_DEPLOYED_AT=${THREEVN_BUILD_TIME} \
    THREEVN_ROBOT_MODEL=threevn_arm_v1 \
    THREEVN_PROFILE=threevn_arm_v1

# OCI labels so `docker inspect` answers the same question as
# /api/version, without starting anything.
LABEL org.opencontainers.image.source="https://github.com/narsi97/3vn-robotics" \
      org.opencontainers.image.licenses="Apache-2.0" \
      org.opencontainers.image.revision="${THREEVN_GIT_COMMIT}" \
      org.opencontainers.image.created="${THREEVN_BUILD_TIME}" \
      org.opencontainers.image.title="3VN Robotics runtime" \
      org.opencontainers.image.description="ROS 2 control stack for the 3VN arm. No simulator."

# curl for the healthcheck only. No build tools, no test tooling, no GUI.
RUN apt-get update && apt-get install -y --no-install-recommends curl \
 && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install /opt/threevn

RUN printf '%s\n' \
  "source /opt/ros/${ROS_DISTRO}/setup.bash" \
  'source /opt/threevn/setup.bash' \
  'true' \
  > /etc/profile.d/20-threevn.sh

# Non-root, numeric UID -- house style. But UNLIKE the other 3VN app
# images this one needs a writable HOME: ros2 launch creates a log
# directory under $HOME/.ros before it will start, and with
# --no-create-home it dies in launch.logging.get_logger() with a
# traceback that never mentions permissions.
#
# ROS_LOG_DIR is set explicitly so the location is predictable rather
# than dependent on how HOME happens to resolve.
RUN adduser --disabled-password --gecos "" --uid ${UID} robot \
 && mkdir -p /home/robot/.ros/log \
 && chown -R ${UID}:${UID} /home/robot
ENV HOME=/home/robot \
    ROS_LOG_DIR=/home/robot/.ros/log
USER ${UID}
WORKDIR /opt/threevn

# 127.0.0.1, never localhost. The dashboard binds IPv4 and a container's
# /etc/hosts resolves localhost to ::1 first, so a localhost healthcheck
# fails against a perfectly healthy service.
#
# /readyz, not /healthz: liveness only says this process answers.
# Readiness says the ROBOT is usable, which is what an orchestrator
# should restart or route on.
HEALTHCHECK --interval=10s --timeout=3s --start-period=30s --retries=5 \
  CMD curl -fsS http://127.0.0.1:8107/readyz || exit 1

COPY docker/runtime-entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["ros2", "launch", "threevn_bringup", "robot.launch.py", "target:=esp32"]
