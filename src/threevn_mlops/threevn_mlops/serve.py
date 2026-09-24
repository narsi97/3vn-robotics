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
Run whatever model the registry says is in production.

    make serve

The node has no idea which model it is running, and that is the point.
Swapping models is `registry promote` plus a restart, never an edit to
this file - so the thing that changes between a good deployment and a
bad one is data in a directory rather than code nobody reviewed.

It REFUSES TO START if the feature contract does not match. The serving
code computes features in a fixed order; a model fitted on a different
order still produces numbers, and they are wrong in a way that looks
like a badly trained model rather than a bug. Checking at load turns a
subtle wrongness into a loud failure before anything moves.

It also publishes what the closed-form geometry would have said, on its
own topic. A learned model that has quietly drifted away from the
physics is visible as a growing gap between the two, without waiting for
anyone to notice the robot missing.
"""

import geometry_msgs.msg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from threevn_ml import features as feat
from threevn_ml import models
from threevn_perception.detector import as_rgb, colour_mask, largest_blob

from threevn_mlops.drift import DriftMonitor
from threevn_mlops.gates import feature_contract, selected_key
from threevn_mlops.registry import Registry


class Serve(Node):
    """Predict the target's position with the production model."""

    def __init__(self):
        super().__init__('serve')
        self.declare_parameter('registry', '/ws/datasets/registry')
        self.declare_parameter('target_width', 0.05)
        self.declare_parameter('report_every', 30)

        registry = Registry(self.get_parameter('registry').value)
        version = registry.production_version()
        if version is None:
            raise RuntimeError(
                'nothing is promoted to production in '
                f'{registry.root}. Register and promote a model first; '
                'serving an arbitrary file would be exactly the untracked '
                'deployment the registry exists to prevent.')

        artifact = registry.artifact(version)

        problems = feature_contract(artifact, feat.FEATURE_NAMES)
        if problems:
            raise RuntimeError(
                f'model {version} cannot be served by this build: '
                + '; '.join(problems))

        self.version = version
        self.model = models.Ridge.from_dict(artifact['model'])
        self.drift = DriftMonitor(artifact['feature_stats'])
        self.intrinsics = None
        self.width = self.get_parameter('target_width').value
        self.report_every = int(self.get_parameter('report_every').value)
        self.frames = 0

        self.pred_pub = self.create_publisher(
            geometry_msgs.msg.PoseStamped, '/ml/target_pose', 10)
        self.geom_pub = self.create_publisher(
            geometry_msgs.msg.PoseStamped, '/ml/target_pose_geometric', 10)

        self.create_subscription(
            Image, '/camera/image_raw', self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._on_info,
            QoSPresetProfiles.SENSOR_DATA.value)

        # Which metrics entry describes the model being served is a
        # question with one answer, in gates. Guessing it here printed a
        # startup banner claiming 46.5 mm for a model scoring 10.5 - the
        # third independent copy of the same mistake.
        test = artifact['metrics'][selected_key(artifact)]['test']
        self.get_logger().info(
            f'serving {version}: test median {test["median_mm"]:.1f} mm, '
            f'p95 {test["p95_mm"]:.1f} mm, fitted on '
            f'{artifact["dataset"]["frames"]} frames')

    def _on_info(self, msg):
        if self.intrinsics is None:
            from threevn_perception.detector import intrinsics_from_camera_info
            self.intrinsics = intrinsics_from_camera_info(msg)

    def _on_image(self, msg):
        """Detect, predict, publish, and watch for drift."""
        if self.intrinsics is None:
            return
        try:
            rgb = as_rgb(msg.data, msg.height, msg.width, msg.encoding)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=10.0)
            return

        blob = largest_blob(colour_mask(rgb))
        if blob is None:
            return

        vector = feat.blob_features(blob, self.intrinsics, feat.FEATURE_NAMES)
        verdict = self.drift.check(vector)
        if verdict['drifting']:
            self.get_logger().warning(
                f'input unlike training data: {verdict["worst_feature"]} is '
                f'{verdict["max_z"]:.1f} sigma out. The prediction is still '
                f'a number; it is no longer a supported one.',
                throttle_duration_sec=5.0)

        predicted = self.model.predict(vector[None, :])[0]
        self._publish(self.pred_pub, msg.header, predicted)

        # What the physics would have said, for comparison. Costs one
        # multiplication and is the only warning available when the
        # learned model drifts away from the geometry on a robot with no
        # ground truth.
        geometric = models.Geometric(self.intrinsics, self.width).predict(
            feat.blob_features(blob, self.intrinsics,
                               feat.GEOMETRIC_FEATURES)[None, :])[0]
        self._publish(self.geom_pub, msg.header, geometric)

        self.frames += 1
        if self.report_every and self.frames % self.report_every == 0:
            summary = self.drift.summary()
            gap = float(np.linalg.norm(predicted - geometric))
            self.get_logger().info(
                f'{summary["frames"]} frames, '
                f'{summary["fraction_drifting"] * 100:.0f}% unlike training, '
                f'model-vs-geometry gap {gap * 1000:.0f} mm')

    @staticmethod
    def _publish(publisher, header, position):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header = header
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        # No orientation is estimated, here as in Phase 12. An identity
        # quaternion is not a measurement.
        pose.pose.orientation.w = 1.0
        publisher.publish(pose)


def main(args=None):
    """Run the serving node."""
    rclpy.init(args=args)
    try:
        node = Serve()
    except RuntimeError as exc:
        print(f'  refusing to serve: {exc}')
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
