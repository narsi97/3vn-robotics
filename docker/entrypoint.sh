#!/usr/bin/env bash
# Sources ROS for anything started via ENTRYPOINT (docker compose run/up).
# `docker compose exec` bypasses this, which is why /etc/profile.d/10-ros.sh
# exists as well. Both are needed; neither is redundant.
set -e
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi
exec "$@"
