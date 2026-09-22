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
TARGET  ?= mock
GUI     ?= 0
NOVNC   := http://localhost:8106/vnc.html

.DEFAULT_GOAL := help
.PHONY: help doctor setup up down shell build test test-sim lint urdf \
        view mock sim robot clean nuke

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
	@echo "  make test-sim  Gazebo integration tests           (slow)"
	@echo "  make lint      ament linters + check_urdf on every profile"
	@echo ""
	@echo "  make urdf      Expand the xacro and print the URDF"
	@echo "  make view      RViz + joint sliders            -> $(NOVNC)"
	@echo "  make mock      Run the stack: no simulator, no hardware"
	@echo "  make sim       Run the stack in Gazebo (GUI=1 for pixels)"
	@echo "  make robot     Run against real hardware        (Phase 5)"
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
test: build
	$(RUN) "colcon test --event-handlers console_direct+ --pytest-args -m 'not slow' && colcon test-result --verbose"

test-sim: build
	$(RUN) "colcon test --packages-select threevn_sim --event-handlers console_direct+ && colcon test-result --verbose"

lint: build
	$(RUN) "colcon test --ctest-args -R 'lint|copyright|flake8|xmllint' ; colcon test-result --verbose"
	$(RUN) "bash /ws/scripts/check_description.sh"

urdf: build
	$(RUN) "ros2 run xacro xacro $(SHARE)/urdf/threevn_arm.urdf.xacro params_file:=$(SHARE)/config/$(PROFILE).yaml target:=$(TARGET)"

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
mock: build
	$(RUN_TTY) "ros2 launch threevn_bringup robot.launch.py target:=mock profile:=$(PROFILE)"

sim: build
	@echo "  Gazebo server headless. GUI=1 adds the gz GUI at $(NOVNC) (slow: llvmpipe)."
	$(RUN_TTY) "ros2 launch threevn_sim sim.launch.py profile:=$(PROFILE) gui:=$(if $(filter 1,$(GUI)),true,false)"

robot: build
	$(RUN_TTY) "ros2 launch threevn_bringup robot.launch.py target:=esp32 profile:=$(PROFILE)"

clean:
	-$(RUN) "rm -rf /ws/build/* /ws/install/* /ws/log/*"

nuke: down
	-docker volume rm 3vn-robotics_build 3vn-robotics_install 3vn-robotics_log
	-docker image rm threevn-robotics-dev:local
	-docker builder prune -f
