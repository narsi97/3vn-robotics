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
The dashboard node: subscribes to the robot, serves HTTP.

Runs wherever the robot runs - not on a remote server. A dashboard that
needs the internet to tell you the arm is fine is a dashboard that lies
to you during an outage, and the project's rule is that losing the
network must degrade observability, never motion.

  ros2 run threevn_dashboard dashboard
  ros2 launch threevn_dashboard dashboard.launch.py port:=8107
"""

import sys

from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from threevn_dashboard.fleet_reporter import FleetReporter
from threevn_dashboard.http_server import make_server, serve_in_background
from threevn_dashboard.robot_state import RobotState
from threevn_dashboard.version import missing_fields, version_info

#: Controller and hardware lists change rarely, so they are polled rather
#: than subscribed. 2 s is fast enough to notice a controller dying and
#: slow enough to be invisible next to a 50 Hz control loop.
POLL_PERIOD = 2.0
SERVICE_TIMEOUT = 1.0


class DashboardNode(Node):
    """Collects robot state and serves it over HTTP."""

    def __init__(self):
        super().__init__('threevn_dashboard')

        self.declare_parameter('port', 8107)
        self.declare_parameter('host', '0.0.0.0')
        port = self.get_parameter('port').value
        host = self.get_parameter('host').value

        self.state = RobotState()

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        # Service calls must not share a callback group with the timer
        # that makes them, or the timer blocks waiting on itself and the
        # node deadlocks with no error.
        #
        # NOTE the attribute name. rclpy.Node keeps its own service list
        # in self._services, so assigning to that name shadows it and the
        # executor dies with "'MutuallyExclusiveCallbackGroup' object is
        # not iterable" - a message that points nowhere near the cause.
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=self._service_group)
        self._hardware_client = self.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components',
            callback_group=self._service_group)
        self.create_timer(
            POLL_PERIOD, self._poll, callback_group=self._service_group)

        self._server = make_server(
            state=self.state,
            version_info=version_info,
            readiness_detail=self.state.readiness_detail,
            host=host,
            port=port,
        )
        serve_in_background(self._server)

        self.get_logger().info(f'dashboard on http://{host}:{port}/')

        # Opt-in. Unset THREEVN_FLEET_URL means nothing starts and
        # nothing is sent -- a robot on a bench should not phone home.
        self._reporter = FleetReporter.from_env(
            state_fn=self._fleet_state,
            version_fn=version_info,
            logger=self.get_logger(),
        )
        if self._reporter is not None:
            self._reporter.start()

        absent = missing_fields()
        if absent:
            # Loud on purpose. A robot that cannot say which commit it is
            # running is a robot nobody can debug later, and the time to
            # notice is at startup rather than during an incident.
            self.get_logger().warn(
                'version fields not set: ' + ', '.join(absent)
                + ' -- set THREEVN_GIT_COMMIT etc. at build/deploy time')

    def _fleet_state(self):
        """Summarise state for the fleet view, with readiness reasons."""
        snapshot = self.state.snapshot()
        _, reasons = self.state.readiness_detail()
        snapshot['reasons'] = reasons
        return snapshot

    # -- ROS callbacks -------------------------------------------------

    def _on_joint_state(self, msg):
        self.state.update_joints(msg.name, msg.position, msg.velocity)

    def _poll(self):
        """Refresh controller and hardware lists."""
        if self._controllers_client.service_is_ready():
            future = self._controllers_client.call_async(ListControllers.Request())
            future.add_done_callback(self._on_controllers)
        else:
            # controller_manager is not up. Clearing rather than keeping
            # the last known list matters: stale 'active' controllers
            # would make /readyz claim the robot is usable after the whole
            # control stack has gone away.
            self.state.update_controllers([])

        if self._hardware_client.service_is_ready():
            future = self._hardware_client.call_async(ListHardwareComponents.Request())
            future.add_done_callback(self._on_hardware)
        else:
            # Same reasoning as the controller list above, and originally
            # missing here: a hardware component left reading "active"
            # after controller_manager has gone is a confident wrong
            # answer about the one thing that touches the servos.
            self.state.update_hardware([])

    def _on_controllers(self, future):
        try:
            response = future.result()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().debug(f'list_controllers failed: {exc}')
            return
        self.state.update_controllers(
            [(c.name, c.state) for c in response.controller])

    def _on_hardware(self, future):
        try:
            response = future.result()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().debug(f'list_hardware_components failed: {exc}')
            return
        self.state.update_hardware(
            [(c.name, c.state.label) for c in response.component])

    def shutdown(self):
        """Stop the fleet reporter and the HTTP server."""
        if self._reporter is not None:
            self._reporter.stop()
        self._server.shutdown()
        self._server.server_close()


def main(argv=None):
    """Entry point."""
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = DashboardNode()
    # Multi-threaded so the poll timer's service calls do not block the
    # joint-state subscription, which is what keeps /readyz honest.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
