#!/usr/bin/env bash
# Runtime entrypoint. Sources ROS and the baked-in workspace, then execs.
#
# Separate from the dev entrypoint because the runtime image has no /ws:
# the workspace is installed at /opt/threevn and there is no source tree.
set -e
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source /opt/threevn/setup.bash
exec "$@"
