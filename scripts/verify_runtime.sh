#!/usr/bin/env bash
# Verify the robot image is what it claims to be.
#
# Two questions, and both matter:
#   1. Does it carry anything it should not - a simulator, GPL/LGPL
#      tooling, the sim package?
#   2. Can it actually start the control stack?
#
# The second is not a formality. An image that is small and licence-clean
# but cannot run is worse than a large one that works, and only the first
# question is easy to check.
set -uo pipefail

IMAGE="${1:-threevn-robotics-runtime:local}"
FAIL=0

ok()  { printf '  \033[32mok\033[0m    %s\n' "$1"; }
bad() { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; FAIL=1; }

echo ""
echo "Verifying ${IMAGE}"
echo ""

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  bad "image not found; run 'make runtime' first"
  exit 1
fi

# --- what must NOT be in it -------------------------------------------
echo "  contents"
for pkg in ros-jazzy-ros-gz-sim ros-jazzy-gz-sim-vendor \
           ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-bridge; do
  if docker run --rm "$IMAGE" bash -lc "dpkg -l 2>/dev/null | grep -q ' $pkg '"; then
    bad "$pkg is in the robot image"
  else
    ok "no $pkg"
  fi
done

# x11vnc is GPL-2.0 and websockify LGPL-3.0. They exist only in the dev
# stage; nothing distributed may carry them. See THIRD_PARTY.md.
for pkg in x11vnc websockify novnc; do
  if docker run --rm "$IMAGE" bash -lc "dpkg -l 2>/dev/null | grep -q ' $pkg '"; then
    bad "$pkg (copyleft, dev-only) is in a distributed image"
  else
    ok "no $pkg"
  fi
done

if docker run --rm "$IMAGE" bash -lc 'test -d /opt/threevn/share/threevn_sim'; then
  bad "threevn_sim was installed into the robot image"
else
  ok "threevn_sim not installed"
fi

# --- version identity --------------------------------------------------
echo ""
echo "  identity"
COMMIT=$(docker run --rm "$IMAGE" bash -lc 'echo -n "$THREEVN_GIT_COMMIT"')
if [ -z "$COMMIT" ] || [ "$COMMIT" = "unknown" ]; then
  bad "THREEVN_GIT_COMMIT is unset; the image cannot say what it is running"
else
  ok "git commit ${COMMIT:0:12}"
fi

# --- can it run? -------------------------------------------------------
echo ""
echo "  behaviour"
NAME="threevn-verify-$$"
docker run --rm -d --name "$NAME" \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  "$IMAGE" ros2 launch threevn_bringup robot.launch.py target:=mock \
  >/dev/null 2>&1

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT

STARTED=0
for _ in $(seq 1 30); do
  if docker exec "$NAME" bash -lc 'ros2 control list_controllers 2>/dev/null' \
     2>/dev/null | grep -q 'arm_controller.*active'; then
    STARTED=1
    break
  fi
  sleep 2
done

if [ "$STARTED" -eq 1 ]; then
  ok "control stack reaches active"
  if docker exec "$NAME" bash -lc \
       'ros2 run threevn_control run_scenario home 2>&1' | grep -q 'PASS'; then
    ok "the home scenario passes inside the image"
  else
    bad "the home scenario failed inside the image"
  fi
else
  bad "the control stack never reached active"
  docker logs "$NAME" 2>&1 | tail -15 | sed 's/^/        /'
fi

echo ""
if [ "$FAIL" -ne 0 ]; then
  echo "  Runtime image verification FAILED"
  echo ""
  exit 1
fi
docker images "$IMAGE" --format '  verified: {{.Repository}}:{{.Tag}}  {{.Size}}'
echo ""
