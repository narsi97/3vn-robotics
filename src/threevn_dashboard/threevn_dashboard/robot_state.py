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
  ready    the ROBOT is actually usable - joint state is arriving and the
           controllers that move it are active.

They differ exactly when it matters. A dashboard that answers cheerfully
while the arm has been disconnected for five minutes is worse than no
dashboard, because it converts an outage into a silent one. Readiness is
what a deployment gate, an alert, or an operator should look at.
"""

import threading
import time

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

    # -- writers (ROS callbacks) ---------------------------------------

    def update_joints(self, names, positions, velocities=()):
        """Record a joint state message."""
        velocities = list(velocities)
        with self._lock:
            self._joints = {
                name: {
                    'position': float(position),
                    'velocity': float(velocities[i]) if i < len(velocities) else None,
                }
                for i, (name, position) in enumerate(zip(names, positions))
            }
            self._joint_stamp = self._clock()
            self._joint_messages += 1

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

            return {
                'uptime_seconds': round(self._clock() - self._started, 1),
                'joints': dict(self._joints),
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
                'ready': joints_fresh and not missing,
            }

    def is_ready(self):
        """
        Return True when the robot is actually usable.

        Requires BOTH fresh joint state and every required controller
        active. Either alone is misleading: active controllers with no
        state means nothing is being reported, and fresh state with a
        dead arm_controller means nothing can be commanded.
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
        return snap['ready'], reasons
