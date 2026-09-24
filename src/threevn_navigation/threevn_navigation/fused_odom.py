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
Publish odometry whose heading comes from the gyroscope.

    make fused-odom

Publishes `/odometry/filtered` and does NOT publish TF by default.

That default is deliberate. `diff_drive_controller` already owns
odom -> base_footprint, and two publishers of one transform is a fight
nobody wins: TF consumers see whichever arrived last, so the robot
appears to jitter between two answers and nothing in any log says why.
Taking TF over means setting `enable_odom_tf: false` on the controller
in the same change, which is why `publish_tf` here is a parameter and
starts false.
"""

import math

import geometry_msgs.msg
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu
import tf2_ros

from threevn_navigation.fusion import FusedPose, YawFuser, wrap


class FusedOdom(Node):
    """Combine wheel distance with gyro heading."""

    def __init__(self):
        super().__init__('fused_odom')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('moving_threshold', 0.01)

        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.moving_threshold = self.get_parameter('moving_threshold').value

        self.fuser = YawFuser()
        self.pose = FusedPose()
        self.wheel_distance = 0.0
        self.moving = False
        self._last_wheel = None
        self._warned_calibrating = False

        self.pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.broadcaster = (tf2_ros.TransformBroadcaster(self)
                            if self.publish_tf else None)

        self.create_subscription(
            Odometry, '/base_controller/odom', self._on_odom, 10)
        self.create_subscription(
            Imu, '/imu', self._on_imu, QoSPresetProfiles.SENSOR_DATA.value)

        self.get_logger().info(
            'heading from /imu, distance from /base_controller/odom; '
            f'publish_tf={self.publish_tf}')
        if self.publish_tf:
            self.get_logger().warning(
                'publishing odom -> base_footprint. The base controller '
                'must have enable_odom_tf:=false, or two publishers will '
                'fight over one transform and TF consumers will see '
                'whichever arrived last.')

    def _on_odom(self, msg):
        """
        Accumulate distance travelled, and decide whether we are moving.

        Distance comes from the wheel odometry's own position delta:
        straight-line wheel odometry measured exact against ground truth
        in Phase 10, so there is nothing to improve there.
        """
        point = msg.pose.pose.position
        current = (point.x, point.y)
        if self._last_wheel is not None:
            self.wheel_distance += math.dist(current, self._last_wheel)
        self._last_wheel = current

        speed = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.angular.z)
        self.moving = speed > self.moving_threshold

    def _on_imu(self, msg):
        """Integrate heading and publish."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = self.fuser.observe(msg.angular_velocity.z, stamp, self.moving)

        if not self.fuser.calibrated:
            if not self._warned_calibrating:
                self.get_logger().info(
                    'estimating gyro bias while stationary; keep the robot '
                    'still for a moment. Integrating an uncorrected bias is '
                    'the difference between degrees per minute and degrees '
                    'per second of drift.')
                self._warned_calibrating = True
            return

        x, y = self.pose.advance(self.wheel_distance, yaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Heading is now the WELL known quantity and position is the one
        # that accumulates, which is the reverse of the wheel-only case.
        # Saying so is what lets a consumer weigh this against anything
        # else it has.
        out.pose.covariance[0] = 0.02
        out.pose.covariance[7] = 0.02
        out.pose.covariance[35] = 0.002
        out.twist.twist.angular.z = msg.angular_velocity.z - self.fuser.bias
        self.pub.publish(out)

        if self.broadcaster is not None:
            transform = geometry_msgs.msg.TransformStamped()
            transform.header = out.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation = out.pose.pose.orientation
            self.broadcaster.sendTransform(transform)

    def heading_degrees(self):
        """Current heading, for logging and tests."""
        return math.degrees(wrap(self.fuser.yaw))


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = FusedOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
