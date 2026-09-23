# 3VN Robotics -- one entry point for every workflow.
#
# Every target runs inside the dev container. Nothing here requires ROS,
# colcon, Gazebo or Python on the host. Only Docker.
#
# Written for GNU Make 3.81 (macOS stock): no .ONESHELL, no $(file ...).

SHELL   := /bin/bash
COMPOSE := docker compose -f docker/compose.yml
SVC     := ros
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

.DEFAULT_GOAL := help
.PHONY: help doctor setup up down shell build test test-sim lint urdf \
        view mock sim robot stop scenario dash acceptance clean nuke test-ros

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
	@echo "  make view      RViz + joint sliders            -> $(NOVNC)"
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

up:
	TVN_UID=$$(id -u) $(COMPOSE) up -d

down:
	$(COMPOSE) down

shell: up
	$(RUN_TTY) "exec bash"

build: up
	$(RUN) "colcon build --symlink-install --event-handlers console_direct+"

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

urdf: build
	$(RUN) "ros2 run xacro xacro $(SHARE)/urdf/$(URDF_ENTRY) params_file:=$(SHARE)/config/$(URDF_PROFILE).yaml target:=$(TARGET)"

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
	          pkill -9 -f 'parameter_brid[g]e' ; sleep 2 ; true" 2>/dev/null
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
