# syntax=docker/dockerfile:1.7
#
# 3VN Robotics development image.
#
# Multi-arch by construction. ros:jazzy-ros-base publishes a native
# linux/arm64 manifest, so this builds natively on an Apple Silicon Mac
# AND natively on x86 CI runners. Do NOT add --platform here: pinning one
# architecture forces QEMU emulation on the other, which turns a 3-minute
# colcon build into a 30-minute one.
#
# Stages:
#   base -> ROS 2 Jazzy + Gazebo Harmonic. No GUI. Nothing optional.
#   ci   -> base + test tooling + software GL. What CI runs.
#   dev  -> ci + RViz + a virtual display served over noVNC. What you run.
#
ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------- base --
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

# Gazebo Harmonic arrives through the ROS *vendor* packages on
# packages.ros.org (gz_sim_vendor, gz_tools_vendor, ...), which are the
# officially paired Harmonic build for Jazzy.
#
# We deliberately do NOT add packages.osrfoundation.org. Mixing the two
# repositories is the standard way to end up with two incompatible sets of
# libgz-*.so; apt resolves it happily and the failure surfaces much later
# as a `gz sim` segfault or a gz_ros2_control symbol-not-found. CI asserts
# that nothing gz-* resolves from osrfoundation. See docs/docker.md.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-gz-ros2-control \
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
# Makefile runs has to find ROS on its own. A profile.d script is the only
# thing that covers interactive shells and non-interactive login shells
# alike. Without this, `make build` works and `make shell` mysteriously
# does not (or vice versa).
RUN printf '%s\n' \
  "source /opt/ros/${ROS_DISTRO}/setup.bash" \
  '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
  'true' \
  > /etc/profile.d/10-ros.sh

# ------------------------------------------------------------------ ci --
FROM base AS ci
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pytest python3-pytest-cov python3-numpy python3-yaml \
      ros-${ROS_DISTRO}-ament-lint-auto \
      ros-${ROS_DISTRO}-ament-lint-common \
      ros-${ROS_DISTRO}-ament-cmake-pytest \
      ros-${ROS_DISTRO}-launch-testing \
      ros-${ROS_DISTRO}-launch-testing-ros \
      libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

# Mesa software rendering. Required for `gz sim --headless-rendering` to
# find an OpenGL implementation when there is no GPU and no X server.
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe

WORKDIR /ws
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]

# ----------------------------------------------------------------- dev --
FROM ci AS dev
ARG ROS_DISTRO
ARG UID=10001

# RViz2 needs a real X server with OpenGL 3.3+. Xvfb + Mesa llvmpipe
# provides GL 4.5, which is why this works where XQuartz (indirect GLX,
# capped near 1.4) does not -- see ros2/rviz#929 and ADR-0002.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      xvfb x11vnc novnc websockify supervisor fluxbox \
      mesa-utils curl vim less bash-completion \
 && rm -rf /var/lib/apt/lists/* \
 && ln -sf vnc.html /usr/share/novnc/index.html

# House style is a non-root numeric UID. Adapted for a dev container:
# unlike the app images this one needs a WRITABLE HOME, because colcon,
# rviz and the gz fuel cache all write there -- so `-H` would be wrong.
# UID is a build arg so a Linux host can match the checkout's owner and
# avoid root-owned files appearing in the bind-mounted source tree.
RUN groupadd -g ${UID} dev \
 && useradd -m -u ${UID} -g ${UID} -s /bin/bash dev \
 && mkdir -p /ws/src /ws/build /ws/install /ws/log \
 && chown -R ${UID}:${UID} /ws

COPY docker/supervisord.conf /etc/supervisor/supervisord.conf

USER ${UID}
WORKDIR /ws
ENV HOME=/home/dev \
    DISPLAY=:1
