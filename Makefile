# 3VN Robotics -- one entry point for every workflow.
#
# Every target runs inside the dev container. Nothing here requires ROS,
# colcon, Gazebo or Python on the host. Only Docker.
#
# Written for GNU Make 3.81 (macOS stock): no .ONESHELL, no $(file ...).

SHELL   := /bin/bash
COMPOSE := docker compose -f docker/compose.yml
SVC     := ros
# The container has git but no repository: .git is deliberately not
# mounted. Dataset provenance needs the commit anyway, so the host
# resolves it and passes it in. A manifest with no commit cannot be
# tied to the code that produced it, and the validator says so.
GIT_COMMIT := $(shell git rev-parse HEAD 2>/dev/null)
GIT_DIRTY  := $(shell test -n "$$(git status --porcelain 2>/dev/null)" && echo 1 || echo 0)
GITENV     := THREEVN_GIT_COMMIT=$(GIT_COMMIT) THREEVN_GIT_DIRTY=$(GIT_DIRTY)

RUN     := $(COMPOSE) exec -T $(SVC) bash -lc
RUN_TTY := $(COMPOSE) exec    $(SVC) bash -lc
SHARE   := /ws/install/threevn_robot_description/share/threevn_robot_description
PROFILE ?= threevn_arm_v1
BASE_PROFILE ?= threevn_base_v1
MM_PROFILE ?= threevn_mm_v1
# The world to spawn into. bench_with_target carries the green cube
# the perception phase looks for.
WORLD ?= empty_bench
TARGET  ?= mock
GUI     ?= 0
NOVNC   := http://localhost:8106/vnc.html
DASH    := http://localhost:8107/
# Raw VNC for a native viewer, and the Foxglove WebSocket. Both are
# bound to 127.0.0.1 by compose; neither is reachable off this machine.
VNCURL  := vnc://127.0.0.1:5906
VNCPASS ?= threevn
FOXURL  := ws://127.0.0.1:8765
# RVIZ=0 drops RViz from `make play`. Use it when you are watching
# through Foxglove and nothing is looking at the container's display.
# It plausibly frees CPU for the physics, but the real-time factor on
# this box is too noisy to prove that -- see docs/viewing.md.
RVIZ    ?= 1

.DEFAULT_GOAL := help
.PHONY: help doctor setup up down shell build test test-sim lint urdf \
        view mock sim robot stop scenario dash acceptance clean nuke test-ros \
        cad cad-test cad-masses play drive vnc foxglove workspace grasp \
        so101 so101-fetch site site-serve course-check

help:
	@echo ""
	@echo "  3VN Robotics"
	@echo ""
	@echo "  make doctor    Check this machine can run the stack"
	@echo "  make setup     Build the image and the workspace (run this first)"
	@echo "  make up/down   Start / stop the dev container"
	@echo "  make shell     Interactive shell inside it, ROS already sourced"
	@echo ""
	@echo "  make build     colcon build --symlink-install"
	@echo "  make test      Fast tests, no simulator           (budget: <60s)"
	@echo "  make test-ros  ROS integration tests, mock target  (~60s)"
	@echo "  make test-sim  Gazebo integration tests           (slow)"
	@echo "  make lint      ament linters + check_urdf on every profile"
	@echo ""
	@echo "  make urdf      Expand the xacro and print the URDF"
	@echo "  make workspace Where the gripper can reach (PITCH=90 for side-on)"
	@echo "  make grasp     Grasp the cube and prove it (needs WORLD=grasp_bench)"
	@echo "  make so101     SPIKE: run our stack against TheRobotStudio SO-101"
	@echo "  make site      Render the docs into a static course site"
	@echo "  make course-check  Validate course/*.yaml without rendering"
	@echo "  make view      RViz + joint sliders            -> $(NOVNC)"
	@echo "  make play      WHOLE ROBOT + RViz, watchable   -> $(NOVNC)"
	@echo "  make drive     Teleop: drive it from the keyboard (2nd terminal)"
	@echo "  make vnc       Open the desktop in a NATIVE viewer (faster than the browser)"
	@echo "  make foxglove  How to attach Foxglove   -> $(FOXURL)"
	@echo "  make mock      Run the stack: no simulator, no hardware"
	@echo "  make sim       Run the stack in Gazebo (GUI=1 for pixels)"
	@echo "  make base-sim  Run the MOBILE BASE in Gazebo (GUI=1 for pixels)"
	@echo "  make robot     Run against real hardware        (Phase 5)"
	@echo ""
	@echo "  make dash      Live dashboard             -> $(DASH)"
	@echo "  make scenario  Run a scenario (NAME=home, or NAME=all)"
	@echo "  make acceptance  Full end-to-end run + report"
	@echo "  make stop      Stop any running robot/sim stack"
	@echo ""
	@echo "  make runtime         Build the robot image (no simulator)"
	@echo "  make runtime-verify  Prove it is clean and that it runs"
	@echo ""
	@echo "  make clean     Remove build/install/log"
	@echo "  make nuke      clean + drop the image and volumes"
	@echo ""
	@echo "  Vars: PROFILE=$(PROFILE)  TARGET=$(TARGET)  GUI=$(GUI)"
	@echo ""

doctor:
	@bash scripts/doctor.sh

setup: doctor
	TVN_UID=$$(id -u) $(COMPOSE) build
	TVN_UID=$$(id -u) $(COMPOSE) up -d
	@$(MAKE) --no-print-directory build
	@echo ""
	@echo "  Ready. Try:  make test   then   make view   then open $(NOVNC)"
	@echo ""

# Silent on stdout: targets like `make urdf` are piped and redirected, and
# an echoed command line there is a line of not-XML. Compose reports its
# own progress on stderr.
up:
	@TVN_UID=$$(id -u) $(COMPOSE) up -d

down:
	$(COMPOSE) down

shell: up
	$(RUN_TTY) "exec bash"

# --symlink-install never removes a link whose source has gone. Delete a
# profile, a world or a launch file and its link stays behind in
# install/, dangling: the test suite discovers profiles from that
# directory and dies on the dead one with a FileNotFoundError naming a
# file you deliberately deleted. Found by running the course's own
# "design your own arm" project. Pruning after every build removes the
# whole class of problem.
PRUNE := find /ws/install -xtype l -printf '  removed stale link %p\n' -delete

build: up
	$(RUN) "colcon build --symlink-install --event-handlers console_direct+ && $(PRUNE)"

# -m 'not slow' excludes the Gazebo tests. The 60s budget is a design
# constraint, not an aspiration: it is what makes people actually run it.
# The rm is not housekeeping. `colcon test-result` reports every XML
# under build/, including results from runs that are no longer relevant,
# so a fixed failure kept being reported and the test count drifted
# upward. Clearing first makes the summary describe THIS run.
test: build
	$(RUN) "rm -rf /ws/build/*/test_results /ws/build/*/pytest.xml /ws/build/*/Testing ; colcon test --event-handlers console_direct+ --pytest-args -m 'not slow' && colcon test-result --verbose"

# ROS integration tier: a real ROS graph and real controllers, no
# physics. Sits between the fast suite (no ROS at all) and the Gazebo
# suite (two minutes), and covers every interface contract in about a
# minute.
test-ros: build stop
	$(RUN) "cd /ws/src/threevn_bringup && python3 -m pytest test -m ros -v"

# Runs pytest DIRECTLY rather than through colcon: the slow marker is
# excluded at the CMake level (see threevn_sim/CMakeLists.txt), so colcon
# would skip exactly the tests this target exists to run.
test-sim: build stop
	$(RUN) "cd /ws/src/threevn_sim && python3 -m pytest test -m slow -v"

# `colcon test-result` reports every XML under build/, not just the tests
# this invocation ran -- so a failure from an earlier `make test` was
# still being reported here long after the code was fixed, and a green
# lint run looked red. Clear the results first so the report describes
# THIS run.
lint: build
	$(RUN) "rm -rf /ws/build/*/test_results /ws/build/*/pytest.xml /ws/build/*/Testing ; colcon test --ctest-args -R 'lint|copyright|flake8|xmllint' ; colcon test-result --verbose"
	$(RUN) "bash /ws/scripts/check_description.sh"

# ROBOT selects which description to expand: arm (default) or base.
# Before Phase 10 this target hardcoded the arm, so `ROBOT=base` was
# accepted and silently printed the arm -- the same class of quiet
# wrong-robot bug the description tests now guard against.
ROBOT   ?= arm
ifeq ($(ROBOT),base)
URDF_ENTRY   := threevn_base.urdf.xacro
URDF_PROFILE := $(BASE_PROFILE)
else ifeq ($(ROBOT),arm)
URDF_ENTRY   := threevn_arm.urdf.xacro
URDF_PROFILE := $(PROFILE)
else ifeq ($(ROBOT),mm)
URDF_ENTRY   := threevn_mobile_manipulator.urdf.xacro
URDF_PROFILE := $(MM_PROFILE)
else
$(error ROBOT must be 'arm', 'base' or 'mm', not '$(ROBOT)')
endif

# Prints the expanded description and NOTHING else on stdout, so that
# `make urdf > robot.urdf` is valid XML and `make urdf | grep -c ...`
# counts only the robot. It used to depend on `build`, whose colcon log
# and make's own echoed command lines landed on stdout ahead of the XML.
# The build still runs first; its log goes to stderr, where you see it.
urdf: up
	@$(RUN) "(colcon build --symlink-install --event-handlers console_direct+ && $(PRUNE)) 1>&2"
	@$(RUN) "ros2 run xacro xacro $(SHARE)/urdf/$(URDF_ENTRY) params_file:=$(SHARE)/config/$(URDF_PROFILE).yaml target:=$(TARGET)"

# The course site. Rendered in a throwaway python image rather than the
# dev container, which has no markdown and does not need one. The
# curriculum is validated first, and a lesson pointing at a Make target
# or a doc that does not exist fails the build.
site:
	@docker run --rm -v "$(PWD)":/src -w /src \
	  -e THREEVN_GIT_COMMIT=$(GIT_COMMIT) python:3.12-alpine sh -lc \
	  "pip install --quiet --no-cache-dir --root-user-action=ignore --disable-pip-version-check markdown==3.7 pyyaml==6.0.2 && \
	   python3 scripts/build_site.py --root /src --out /src/site --clean"

course-check:
	@docker run --rm -v "$(PWD)":/src:ro -w /src python:3.12-alpine sh -lc \
	  "pip install --quiet --no-cache-dir --root-user-action=ignore --disable-pip-version-check pyyaml==6.0.2 && \
	   python3 scripts/build_site.py --root /src --check"

site-serve: site
	@echo ""
	@echo "  course site -> http://localhost:8109/"
	@docker run --rm -p 127.0.0.1:8109:80 \
	  -v "$(PWD)/site":/usr/share/nginx/html:ro nginx:alpine

# A spike: somebody else's robot, our controllers. See the launch file.
so101-fetch:
	@./scripts/fetch_so101.sh

so101: build stop
	$(RUN_TTY) "ros2 launch threevn_sim so101_sim.launch.py"

# Needs a running: make play WORLD=grasp_bench
grasp: build
	$(RUN_TTY) "python3 /ws/scripts/verify_grasp.py"

# Pure geometry from the URDF: no simulator, no robot, nothing running.
workspace: build
	$(RUN_TTY) "ros2 run threevn_control workspace \
	            --pitch-deg $(if $(PITCH),$(PITCH),180)"

view: build
	@echo ""
	@echo "  RViz will appear at $(NOVNC)"
	@echo ""
	$(RUN_TTY) "ros2 launch threevn_robot_description view_robot.launch.py profile:=$(PROFILE)"

# ---------------------------------------------------------------------
# The three targets below map 1:1 onto the three hardware_interface
# plugins. Same launch file, same controllers, same topics and actions.
# Switching between them is a launch argument, never a code change.
# This IS the architecture -- see docs/decisions/0003-hardware-seam.md.
# ---------------------------------------------------------------------
mock: build stop
	$(RUN_TTY) "ros2 launch threevn_bringup robot.launch.py target:=mock profile:=$(PROFILE)"

sim: build stop
	@echo "  Gazebo server headless. GUI=1 adds the gz GUI at $(NOVNC) (slow: llvmpipe)."
	$(RUN_TTY) "ros2 launch threevn_sim sim.launch.py profile:=$(PROFILE) gui:=$(if $(filter 1,$(GUI)),true,false)"

# Checks for the device first. Without it, controller_manager aborts
# with an uncaught runtime_error and exit code -6, which reads as a crash
# rather than "nothing is plugged in".
ESP32_PORT ?= /dev/ttyUSB0
# The mobile base, standalone. Phase 11 composes it with the arm; until
# then they are separate robots with separate controllers.
base-sim: build stop
	@echo "  Gazebo server headless. GUI=1 shows it at $(NOVNC) (slow: llvmpipe)."
	$(RUN_TTY) "ros2 launch threevn_sim base_sim.launch.py profile:=$(BASE_PROFILE) gui:=$(if $(filter 1,$(GUI)),true,false)"


base-drive: build
	@echo '  measuring against a running: make base-sim'
	$(RUN) "python3 /ws/scripts/verify_base_drive.py"

# The composed robot: arm on base, one controller_manager hosting both.
play: build stop
	@echo ""
	@echo "  Whole robot, headless Gazebo + RViz -> $(NOVNC)"
	@echo "  Then, in a SECOND terminal:  make drive"
	@echo ""
	$(RUN_TTY) "ros2 launch threevn_sim play.launch.py \
	            profile:=$(MM_PROFILE) world:=$(WORLD) \
	            rviz:=$(if $(filter 0,$(RVIZ)),false,true)"

# Neither of these starts or stops anything: they are ways to LOOK at
# a session that is already running.
vnc: up
	@echo ""
	@echo "  Opening $(VNCURL)"
	@echo "  Password:  $(VNCPASS)"
	@echo ""
	@echo "  There is a password only because native clients (macOS"
	@echo "  Screen Sharing included) refuse a server that offers none --"
	@echo "  they hang on 'Connecting...' forever. The port is bound to"
	@echo "  127.0.0.1, so it is not protecting anything. Tick 'Remember'"
	@echo "  and you will not be asked again."
	@echo ""
	@open $(VNCURL) 2>/dev/null || echo "  open it manually: $(VNCURL)"

foxglove: up
	@echo ""
	@echo "  1. Install Foxglove:  https://foxglove.dev/download"
	@echo "  2. Open connection:   $(FOXURL)"
	@echo ""
	@echo "  This renders on THIS Mac's GPU -- no framebuffer is shipped,"
	@echo "  so the view stays smooth however slowly the sim is running."
	@echo "  Pair it with:  make play RVIZ=0"
	@echo ""

# No `stop` here: drive ATTACHES to whatever play started. A stop
# prerequisite would kill the simulator this is meant to drive.
drive: build
	@echo ""
	@echo "  needs a running: make play   (or make mm-sim / make mock)"
	@echo ""
	$(RUN_TTY) "ros2 run threevn_control teleop"

mm-sim: build stop
	@echo '  Gazebo server headless. GUI=1 shows it at $(NOVNC) (slow: llvmpipe).'
	$(RUN_TTY) "ros2 launch threevn_sim mm_sim.launch.py profile:=$(MM_PROFILE) world:=$(WORLD) gui:=$(if $(filter 1,$(GUI)),true,false)"


mm-verify: build
	@echo '  measuring against a running: make mm-sim'
	$(RUN) "python3 /ws/scripts/verify_mm.py"


# Perception. Run against a sim started with WORLD=bench_with_target,
# which carries the green cube; against any other world the node runs
# and finds nothing, which is the correct behaviour and a boring demo.
perception: build
	@echo '  needs a running: make mm-sim WORLD=bench_with_target'
	$(RUN_TTY) "ros2 run threevn_perception find_target"

perception-verify: build
	@echo '  needs a running sim AND a running: make perception'
	$(RUN) "python3 /ws/scripts/verify_perception.py"


# Dataset collection. Needs a running: make mm-sim WORLD=bench_with_target
EPISODES ?= 6
dataset: build
	@echo '  needs a running: make mm-sim WORLD=bench_with_target'
	$(RUN) "$(GITENV) ros2 run threevn_data collect --episodes $(EPISODES)"

dataset-describe: build
	$(RUN) "ros2 run threevn_data describe"


# ALPHA overrides the ridge penalty. Its use in the course is to make a
# WORSE model on purpose (ALPHA=10), so the promotion gates have
# something real to refuse. Always through here rather than `ros2 run`
# in a shell: GITENV is what gives the artefact its provenance, and the
# provenance gate refuses a model without it.
train: build
	$(RUN) "$(GITENV) ros2 run threevn_ml train $(if $(ALPHA),--alpha $(ALPHA),)"


# The model registry. Versions are content-addressed and immutable;
# promotion runs the gates and rollback is the same operation backwards.
registry-register: build
	$(RUN) "ros2 run threevn_mlops registry register"

# FORCE=1 promotes past failing gates, and the registry records it as
# forced, with REASON. It exists so a rollback can be rehearsed before
# it is needed -- not as a way round the gates.
registry-promote: build
	$(RUN) "ros2 run threevn_mlops registry promote $(VERSION) $(if $(FORCE),--force --reason '$(or $(REASON),forced from make)',)"

registry-status: build
	$(RUN) "ros2 run threevn_mlops registry status"

registry-rollback: build
	$(RUN) "ros2 run threevn_mlops registry rollback"

serve: build
	@echo '  needs a running: make mm-sim WORLD=bench_with_target'
	$(RUN_TTY) "ros2 run threevn_mlops serve"


heading: build
	@echo '  needs a running: make mm-sim WORLD=bench_with_target'
	$(RUN) "python3 /ws/scripts/verify_heading.py"


fused-odom: build
	@echo '  needs a running: make mm-sim WORLD=bench_with_target'
	$(RUN_TTY) "ros2 run threevn_navigation fused_odom"


# Parametric CAD. Runs under the venv python from the `cad` Dockerfile
# stage, because build123d cannot be installed into Debian's system
# python without fighting dpkg over typing_extensions.
CAD_RUN := $(COMPOSE) exec -T $(SVC) bash -lc

cad:
	$(CAD_RUN) "cd /ws/cad && \$$CAD_PYTHON -m threevn_cad.export"

cad-masses:
	$(CAD_RUN) "cd /ws/cad && \$$CAD_PYTHON -c \"from threevn_cad import masses, profile; cfg=profile.load(); [print('  %-26s %7.1f g declared   %7.1f g computed' % (l, d*1000, c*1000)) for l,d,c in masses.compare(cfg)]\""

cad-test:
	$(CAD_RUN) "cd /ws/cad && \$$CAD_PYTHON -m pytest tests -q"


robot: build stop
	@$(RUN) "test -e $(ESP32_PORT)" 2>/dev/null || { \
	  echo ""; \
	  echo "  No device at $(ESP32_PORT) inside the container."; \
	  echo ""; \
	  echo "  Plug the ESP32 in, then pass it through to Docker. On macOS,"; \
	  echo "  Docker Desktop cannot forward USB directly - see docs/firmware.md."; \
	  echo "  Override the path with:  make robot ESP32_PORT=/dev/ttyACM0"; \
	  echo ""; \
	  exit 1; }
	$(RUN_TTY) "ros2 launch threevn_bringup robot.launch.py target:=esp32 profile:=$(PROFILE)"

# Stop any previously launched stack.
#
# Every run target depends on this. Leaving a robot_state_publisher from a
# previous `make mock` alive means TWO publishers on /robot_description,
# and whichever the simulator reads first wins - a genuinely confusing
# failure. One cheap target removes the whole class of problem.
#
# The bracket in the pattern stops pkill matching its own command line,
# which would otherwise kill the shell running it.
#
# The /ws/install/*/lib/ pattern catches every executable this workspace
# builds -- the dashboard, perception, serving, fusion. Killing `ros2
# launch` with -9 orphans its children, and an orphaned dashboard kept
# port 8107: the next `make dash` died with "Address already in use".
# Found by running the course's Tier 5 lessons after `make play`.
# The dashboard observes whatever robot is already running. It does NOT
# depend on `stop`, so it can be started alongside `make sim` / `make mock`
# without killing them.
#
# THREEVN_GIT_COMMIT is passed through so the version block is real rather
# than "unknown"; in production CI injects it at image build time.
dash: build
	@echo ""
	@echo "  dashboard -> $(DASH)"
	@echo ""
	$(RUN_TTY) "THREEVN_GIT_COMMIT=$(shell git rev-parse --short HEAD 2>/dev/null || echo unknown) \
	            THREEVN_GIT_BRANCH=$(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown) \
	            THREEVN_TARGET=$(TARGET) \
	            THREEVN_PROFILE=$(PROFILE) \
	            ros2 launch threevn_dashboard dashboard.launch.py"

# The full end-to-end acceptance run (spec section 50). Needs a robot
# already up -- `make sim &` or `make mock &` -- and reads telemetry from
# the dashboard if that is running too, so the report can name the build
# that produced it.
acceptance: build
	$(RUN_TTY) "ros2 run threevn_control acceptance --report /ws/log/acceptance"
	@echo ""
	@echo "  report written inside the container at /ws/log/acceptance.{json,md}"
	@echo "  copy it out with:  make acceptance-report"

acceptance-report:
	@docker compose -f docker/compose.yml cp $(SVC):/ws/log/acceptance.md ./acceptance.md 2>/dev/null \
	  && echo "  -> ./acceptance.md" || echo "  no report yet; run 'make acceptance' first"


# The image that ships to a robot. Built from `base`, which has no
# Gazebo -- possible only because threevn_bringup and threevn_hardware
# genuinely do not depend on it.
runtime:
	docker build -f docker/Dockerfile --target runtime \
	  --build-arg THREEVN_GIT_COMMIT=$(shell git rev-parse HEAD 2>/dev/null || echo unknown) \
	  --build-arg THREEVN_GIT_BRANCH=$(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown) \
	  --build-arg THREEVN_BUILD_TIME=$(shell date -u +%Y-%m-%dT%H:%M:%SZ) \
	  -t threevn-robotics-runtime:local .
	@echo ""
	@docker images threevn-robotics-runtime:local --format '  built: {{.Repository}}:{{.Tag}}  {{.Size}}'
	@echo ""

# Asserts the robot image carries no simulator and no GPL/LGPL tooling,
# then proves it can actually start the control stack. Small and clean
# but unable to run would be worse than large and working.
runtime-verify: runtime
	@bash scripts/verify_runtime.sh


stop: up
	-@$(RUN) "pkill -9 -f 'ros2 laun[c]h' ; pkill -9 -f '[g]z sim' ; \
	          pkill -9 -f 'robot_state_pub[l]isher' ; pkill -9 -f '[r]viz2' ; \
	          pkill -9 -f 'joint_state_pub[l]isher' ; pkill -9 -f 'ros2_control_no[d]e' ; \
	          pkill -9 -f 'parameter_brid[g]e' ; \
	          pkill -9 -f 'foxglove_brid[g]e' ; \
	          pkill -9 -f '/ws/instal[l]/[a-z_]*/lib/' ; sleep 2 ; \
	          pgrep -f '[r]os2 ' >/dev/null || rm -f /dev/shm/fastrtps_* ; \
	          true" 2>/dev/null
	@echo "  stopped"

# Run a scenario against whatever stack is currently up.
scenario: build
	$(RUN_TTY) "ros2 run threevn_control run_scenario $(if $(filter all,$(NAME)),--all,$(NAME))"

clean:
	-$(RUN) "rm -rf /ws/build/* /ws/install/* /ws/log/*"

nuke: down
	-docker volume rm 3vn-robotics_build 3vn-robotics_install 3vn-robotics_log
	-docker image rm threevn-robotics-dev:local
	-docker builder prune -f
