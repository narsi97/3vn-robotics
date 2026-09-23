#!/usr/bin/env bash
# Expand every robot profile against every hardware target and run each
# result through check_urdf -- the C++ urdfdom parser.
#
# This is deliberately a SECOND parser. The Python urdf_parser_py used in
# the unit tests and this C++ one have different leniencies; a file that
# one accepts and the other rejects is a real finding, not noise.
set -euo pipefail

SHARE="$(ros2 pkg prefix threevn_robot_description)/share/threevn_robot_description"
FAIL=0

# Each profile is expanded through the xacro for ITS robot family, read
# from meta.kind. This script used to hardcode the arm entry point and
# glob every profile in config/, so adding the mobile base made it try to
# build an arm out of base parameters and fail on a missing shoulder.
entry_for() {
  python3 - "$1" <<'PYEOF'
import pathlib, sys, yaml
kind = yaml.safe_load(pathlib.Path(sys.argv[1]).read_text())['meta']['kind']
entries = {'arm': 'threevn_arm.urdf.xacro', 'base': 'threevn_base.urdf.xacro'}
if kind not in entries:
    sys.exit(f'unknown robot family {kind!r} in {sys.argv[1]}')
print(entries[kind])
PYEOF
}

for profile in "${SHARE}"/config/threevn_*.yaml; do
  XACRO="${SHARE}/urdf/$(entry_for "$profile")"
  for target in mock gz esp32; do
    name="$(basename "$profile" .yaml):${target}"
    out="/tmp/$(basename "$profile" .yaml)_${target}.urdf"
    if ! ros2 run xacro xacro "$XACRO" \
           params_file:="$profile" target:="$target" > "$out" 2>/tmp/xacro.err; then
      echo "FAIL  ${name}: xacro expansion failed"
      sed 's/^/        /' /tmp/xacro.err
      FAIL=1
      continue
    fi
    if check_urdf "$out" > /tmp/check.out 2>&1; then
      echo "ok    ${name}  ($(grep -c '<link' "$out") links, $(grep -c '<joint' "$out") joints)"
    else
      echo "FAIL  ${name}: check_urdf rejected the output"
      sed 's/^/        /' /tmp/check.out
      FAIL=1
    fi
  done
done

exit "$FAIL"
