# Copyright 2026 3VN Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Aggregated robot state, and the health it implies.

Collects what is scattered across several ROS interfaces into one
snapshot: joint positions, controller states, and how recently any of it
arrived.

The liveness/readiness split is the load-bearing idea here, and it is not
bureaucracy:

  healthy  this process is running and able to answer.
  ready    the ROBOT is actually usable - joint state is arriving, the
           controllers that move it are active, and no joint is straining
           against something it cannot move or running hot.

They differ exactly when it matters. A dashboard that answers cheerfully
while the arm has been disconnected for five minutes is worse than no
dashboard, because it converts an outage into a silent one. Readiness is
what a deployment gate, an alert, or an operator should look at.

The last two conditions need a robot that reports its own effort, and
not every robot does. The SO-101's bus servos report load and
temperature; the 3VN arm's hobby servos report nothing but where they
were told to go. A joint with no effort reading, or no known limit, is
shown as `not reported` and is never counted as straining: a check that
cannot be made is said to be missing, not passed off as passed.
"""

import threading
import time
import xml.etree.ElementTree as ET

#: Beyond this, joint state is treated as stale rather than merely old.
#: The controllers publish at 50 Hz, so a whole second is ~50 missed
#: messages - comfortably past 'a slow moment' and into 'something broke'.
STALE_AFTER_SECONDS = 1.0

#: Beyond this, the controller list is treated as unknown rather than
#: current.
#:
#: This exists because of a measured failure. Killing ros2_control_node
#: left the dashboard reporting all three controllers "active" for a
#: control stack that no longer existed: rclpy's service_is_ready() keeps
#: returning True after the server dies, so the poll kept succeeding-ish
#: and the last good list was never replaced.
#:
#: Readiness happened to stay correct only because joint state went stale
#: at the same moment. Relying on that would be relying on a coincidence,
#: so controller data ages out on its own terms. Three poll cycles, to
#: tolerate one dropped reply without flapping.
CONTROLLER_STALE_AFTER_SECONDS = 6.0

#: Controllers that must be active for the robot to be considered usable.
REQUIRED_CONTROLLERS = ('joint_state_broadcaster', 'arm_controller')

#: A joint is STRAINING when its effort stays at or above this share of
#: its effort limit. The limit is what the joint's servo can deliver
#: (for the SO-101, its stall torque derated for the supply), so 90% of
#: it is a servo pushing as hard as it can. Holding any named pose takes
#: far less.
STRAIN_FRACTION = 0.9

#: ...for at least this long. A move starts with a short burst of effort
#: as the joint accelerates; that is not a robot in trouble. Something
#: in the way is still in the way half a second later.
STRAIN_AFTER_SECONDS = 0.5

#: A servo at or above this temperature makes the robot not ready.
#: The STS3215 cuts its own torque at 70 C, its factory overheat limit.
#: Stopping work five degrees short of that is a choice made here rather
#: than by the servo, mid-motion.
TEMPERATURE_LIMIT_C = 65.0


def effort_limits_from_urdf(urdf_xml):
    """
    Return {joint: effort limit} from a URDF, for joints that declare one.

    The limit is read from the description the robot is actually running
    (`/robot_description`), not from a table here, so a profile change
    cannot leave the dashboard judging the arm by another arm's numbers.
    """
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError:
        return {}
    limits = {}
    # findall, not iter: a <transmission> holds <joint> elements too.
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        if limit is None or limit.get('effort') is None:
            continue
        try:
            effort = float(limit.get('effort'))
        except ValueError:
            continue
        if effort > 0.0:
            limits[joint.get('name')] = effort
    return limits


def gripper_joints_from_urdf(urdf_xml):
    """
    Return the gripper joint(s) the description's semantics name.

    A gripper closed on an object is pushing at its limit ON PURPOSE:
    that is what holding is. Its load is shown, but never counted as
    straining. Read from the <threevn_semantics> block that both robots'
    descriptions carry; empty if there is none.
    """
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError:
        return set()
    return {
        gripper.get('joint')
        for gripper in root.findall('threevn_semantics/gripper')
        if gripper.get('joint')
    }


class RobotState:
    """
    Thread-safe store of the latest robot state.

    Written by ROS callbacks on the executor thread, read by HTTP handlers
    on the server thread, so every access takes the lock. The snapshots
    handed out are plain dicts, deliberately: a caller cannot then hold a
    reference into mutable state it does not own.
    """

    def __init__(self, clock=time.monotonic):
        self._lock = threading.Lock()
        self._clock = clock
        self._started = clock()
        self._joints = {}
        self._joint_stamp = None
        self._controllers = []
        self._controller_stamp = None
        self._hardware = []
        self._hardware_stamp = None
        self._joint_messages = 0
        self._effort_limits = {}
        self._holding_joints = set()
        # When each joint's effort first reached the strain threshold,
        # by joint. A joint below it has no entry.
        self._strain_since = {}
        self._temperatures = {}
        self._temperature_stamp = None

    # -- writers (ROS callbacks) ---------------------------------------

    def update_joints(self, names, positions, velocities=(), efforts=()):
        """Record a joint state message."""
        velocities = list(velocities)
        efforts = list(efforts)
        with self._lock:
            now = self._clock()
            self._joints = {
                name: {
                    'position': float(position),
                    'velocity': float(velocities[i]) if i < len(velocities) else None,
                    'effort': float(efforts[i]) if i < len(efforts) else None,
                }
                for i, (name, position) in enumerate(zip(names, positions))
            }
            self._joint_stamp = now
            self._joint_messages += 1
            for name, joint in self._joints.items():
                load = self._load(name, joint['effort'])
                if (load is not None and load >= STRAIN_FRACTION
                        and name not in self._holding_joints):
                    self._strain_since.setdefault(name, now)
                else:
                    self._strain_since.pop(name, None)

    def set_effort_limits(self, limits, holding=()):
        """
        Record each joint's effort limit, as {joint: N.m or N}.

        `holding` names joints that push at their limit by design -- a
        gripper closed on something. Their load is shown, never judged.
        """
        with self._lock:
            self._effort_limits = {
                name: float(value) for name, value in limits.items() if value > 0}
            self._holding_joints = set(holding)
            for name in self._holding_joints:
                self._strain_since.pop(name, None)

    def update_temperatures(self, temperatures):
        """Record servo temperatures, as {joint: degrees C}."""
        with self._lock:
            self._temperatures = {
                name: float(value) for name, value in temperatures.items()}
            self._temperature_stamp = self._clock()

    def update_controllers(self, controllers):
        """Record controller states as (name, state) pairs."""
        with self._lock:
            self._controllers = [
                {'name': name, 'state': state} for name, state in controllers
            ]
            self._controller_stamp = self._clock()

    def update_hardware(self, components):
        """Record hardware components as (name, state) pairs."""
        with self._lock:
            self._hardware = [
                {'name': name, 'state': state} for name, state in components
            ]
            self._hardware_stamp = self._clock()

    # -- readers -------------------------------------------------------

    def _age(self, stamp):
        return None if stamp is None else self._clock() - stamp

    def _load(self, name, effort):
        """Return |effort| as a share of the joint's limit, or None if unknown."""
        limit = self._effort_limits.get(name)
        if effort is None or not limit:
            return None
        return abs(effort) / limit

    def snapshot(self):
        """Return a plain-dict snapshot of everything known."""
        with self._lock:
            joint_age = self._age(self._joint_stamp)
            controller_age = self._age(self._controller_stamp)

            controllers_fresh = (
                controller_age is not None
                and controller_age <= CONTROLLER_STALE_AFTER_SECONDS)

            # Hardware ages out on the same clock and for the same reason.
            # Clearing these lists when service_is_ready() goes false does
            # NOT work: rclpy keeps reporting a dead service as ready, so
            # the call is issued, never completes, and the previous answer
            # survives indefinitely. Ageing is the only signal that holds.
            hardware_age = self._age(self._hardware_stamp)
            hardware_fresh = (
                hardware_age is not None
                and hardware_age <= CONTROLLER_STALE_AFTER_SECONDS)

            # A stale list is not evidence of anything. Treating it as
            # current is how a dead control stack reports itself healthy.
            active = (
                {c['name'] for c in self._controllers if c['state'] == 'active'}
                if controllers_fresh else set()
            )
            missing = [c for c in REQUIRED_CONTROLLERS if c not in active]
            joints_fresh = (
                joint_age is not None and joint_age <= STALE_AFTER_SECONDS)

            now = self._clock()
            temperature_age = self._age(self._temperature_stamp)
            temperatures_fresh = (
                temperature_age is not None
                and temperature_age <= STALE_AFTER_SECONDS)

            joints = {}
            for name, joint in self._joints.items():
                entry = dict(joint)
                load = self._load(name, joint.get('effort'))
                entry['effort_limit'] = self._effort_limits.get(name)
                entry['load'] = None if load is None else round(load, 3)
                entry['temperature'] = (
                    self._temperatures.get(name) if temperatures_fresh else None)
                joints[name] = entry

            # Straining needs FRESH joint state: an old effort reading says
            # nothing about now, and a robot whose state has stopped is
            # already not ready for that reason.
            straining = []
            if joints_fresh:
                for name, since in sorted(self._strain_since.items()):
                    held = now - since
                    if held >= STRAIN_AFTER_SECONDS:
                        straining.append({
                            'joint': name,
                            'effort': joints[name]['effort'],
                            'effort_limit': joints[name]['effort_limit'],
                            'load': joints[name]['load'],
                            'seconds': round(held, 1),
                        })
            # A temperature that has stopped arriving is not evidence of a
            # cool servo, but it is not evidence of a hot one either. It is
            # shown as missing (None) and does not count.
            hot = [
                {'joint': name, 'temperature': value}
                for name, value in sorted(self._temperatures.items())
                if temperatures_fresh and value >= TEMPERATURE_LIMIT_C
            ]

            return {
                'uptime_seconds': round(now - self._started, 1),
                'joints': joints,
                'load_reported': any(
                    j['load'] is not None for j in joints.values()),
                'joint_state_age_seconds': (
                    None if joint_age is None else round(joint_age, 3)),
                'joint_messages': self._joint_messages,
                'joints_fresh': joints_fresh,
                'controllers': list(self._controllers),
                'controllers_fresh': controllers_fresh,
                'controller_state_age_seconds': (
                    None if controller_age is None else round(controller_age, 1)),
                'hardware': list(self._hardware),
                'hardware_fresh': hardware_fresh,
                'hardware_state_age_seconds': (
                    None if hardware_age is None else round(hardware_age, 1)),
                'missing_controllers': missing,
                'straining': straining,
                'hot': hot,
                'ready': joints_fresh and not missing and not straining and not hot,
            }

    def is_ready(self):
        """
        Return True when the robot is actually usable.

        Requires fresh joint state, every required controller active,
        and, where the robot reports them, no joint straining and no
        servo running hot. The first two alone are misleading: active
        controllers with no state means nothing is being reported, and
        fresh state with a dead arm_controller means nothing can be
        commanded. The last two are what a robot with servos that report
        their own load can add: a joint pushing at its limit is not doing
        what it was told, whatever its controller says.
        """
        return self.snapshot()['ready']

    def readiness_detail(self):
        """Return (ready, reasons) explaining any failure."""
        snap = self.snapshot()
        reasons = []
        if snap['joint_state_age_seconds'] is None:
            reasons.append('no joint state received')
        elif not snap['joints_fresh']:
            reasons.append(
                f'joint state is {snap["joint_state_age_seconds"]:.1f}s old '
                f'(stale beyond {STALE_AFTER_SECONDS:.1f}s)')
        if (snap['controller_state_age_seconds'] is not None
                and not snap['controllers_fresh']):
            reasons.append(
                f'controller list is {snap["controller_state_age_seconds"]:.0f}s '
                'old -- controller_manager is not answering')
        for name in snap['missing_controllers']:
            reasons.append(f'controller {name!r} is not active')
        for strain in snap['straining']:
            reasons.append(
                f'joint {strain["joint"]!r} is straining: '
                f'{abs(strain["effort"]):.2f} of {strain["effort_limit"]:.2f} '
                f'({strain["load"]:.0%} of its limit) for {strain["seconds"]:.1f}s '
                '-- something is in its way, or the load is more than it can move')
        for servo in snap['hot']:
            reasons.append(
                f'servo on {servo["joint"]!r} is at {servo["temperature"]:.0f} C '
                f'(limit {TEMPERATURE_LIMIT_C:.0f} C)')
        return snap['ready'], reasons
