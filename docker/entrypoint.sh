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
# macOS Screen Sharing -- and most native VNC clients -- refuse a server
# that offers only the "None" security type: they sit at "Connecting..."
# forever rather than reporting anything. So the native-facing x11vnc on
# 5901 needs a password file, which supervisord points it at.
#
# This is NOT a security boundary and is not treated as a secret. The
# port is published to 127.0.0.1 only, so anything that can reach it is
# already running as you on this machine. It exists purely to make the
# client offer an auth method it is willing to complete.
if command -v x11vnc >/dev/null 2>&1; then
  mkdir -p "${HOME}/.vnc"
  x11vnc -storepasswd "${THREEVN_VNC_PASSWORD:-threevn}" \
         "${HOME}/.vnc/passwd" >/dev/null 2>&1 || true
fi

exec "$@"
