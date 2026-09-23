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
Publish where the target is.

    ros2 run threevn_perception find_target

This node is deliberately thin. Every decision it makes lives in
detector.py, which is tested on images drawn by the tests themselves;
what remains here is subscribing, transforming and publishing, which is
the part that genuinely needs a ROS graph.

Two outputs, and the difference between them is the point of the phase:

    /perception/target_pose      in the camera's OPTICAL frame
    /perception/target_in_base   in base_link, via TF

The first is what the camera measured. The second is what anything
wanting to drive or reach can use, and getting there requires the
optical-frame rotation to be right. A pipeline with that rotation wrong
publishes both topics, at the right rate, with plausible numbers, and
sends the arm to the wrong place.
"""

import geometry_msgs.msg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped with tf2)
import tf2_ros

from threevn_perception.detector import (
    as_rgb,
    detect,
    intrinsics_from_camera_info,
)


class FindTarget(Node):
    """Find the coloured target in each frame and publish its position."""

    def __init__(self):
        super().__init__('find_target')

        self.declare_parameter('target_width', 0.05)
        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('colour_channel', 1)
        self.declare_parameter('minimum', 60)
        self.declare_parameter('dominance', 30)

        self.target_width = self.get_parameter('target_width').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.mask_kwargs = {
            'channel': self.get_parameter('colour_channel').value,
            'minimum': self.get_parameter('minimum').value,
            'dominance': self.get_parameter('dominance').value,
        }

        self.intrinsics = None

        self.pose_pub = self.create_publisher(
            geometry_msgs.msg.PoseStamped, '/perception/target_pose', 10)
        self.base_pub = self.create_publisher(
            geometry_msgs.msg.PoseStamped, '/perception/target_in_base', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # SENSOR_DATA, not the default. Images are large and late ones are
        # worthless: a reliable queue makes the node fall further behind
        # rather than skip, which on a 15 Hz camera turns a slow frame
        # into a growing backlog of stale detections.
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._on_info,
            QoSPresetProfiles.SENSOR_DATA.value)

        self._warned = False
        self.get_logger().info(
            f'looking for a {self.target_width * 1000:.0f} mm target, '
            f'reporting in {self.reference_frame!r}')

    def _on_info(self, msg):
        """Latch the intrinsics from the camera itself."""
        if self.intrinsics is None:
            self.intrinsics = intrinsics_from_camera_info(msg)
            self.get_logger().info(
                f'intrinsics fx={self.intrinsics.fx:.1f} '
                f'cx={self.intrinsics.cx:.1f} cy={self.intrinsics.cy:.1f}')

    def _on_image(self, msg):
        """Detect, transform and publish."""
        if self.intrinsics is None:
            # No CameraInfo yet. Guessing intrinsics from the image size
            # would work and would be wrong the first time the field of
            # view changed, so wait instead.
            return

        try:
            rgb = as_rgb(msg.data, msg.height, msg.width, msg.encoding)
        except ValueError as exc:
            if not self._warned:
                self.get_logger().error(str(exc))
                self._warned = True
            return

        found = detect(rgb, self.intrinsics, self.target_width,
                       **self.mask_kwargs)
        if found is None:
            return
        _, (x, y, z) = found

        pose = geometry_msgs.msg.PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Orientation is not estimated. A colour blob carries no
        # rotation, and publishing an identity quaternion as though it
        # were measured is how a pose that is 3 numbers pretends to be 6.
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        try:
            in_base = self.tf_buffer.transform(
                pose, self.reference_frame,
                timeout=rclpy.duration.Duration(seconds=0.2))
        except tf2_ros.TransformException as exc:
            self.get_logger().warning(
                f'cannot transform {msg.header.frame_id!r} to '
                f'{self.reference_frame!r}: {exc}',
                throttle_duration_sec=5.0)
            return
        self.base_pub.publish(in_base)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = FindTarget()
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
