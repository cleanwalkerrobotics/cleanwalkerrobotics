#!/usr/bin/env python3
"""
Terrain Segmentation Node — CleanWalker CW-1

Runs SegFormer-B0 (TRT FP16, ~25ms) at 5 Hz to classify terrain into
8 classes and publishes a terrain mask for the Nav2 costmap layer.

Subscribes:
  /oak/rgb/image     (sensor_msgs/Image)     — 30 Hz RGB from OAK-D Pro

Publishes:
  /terrain/mask      (sensor_msgs/Image)     — 8-class segmentation mask (5 Hz)
  /terrain/costmap   (nav_msgs/OccupancyGrid) — traversability costmap layer

Architecture reference: docs/technical/perception-pipeline-architecture.md
  Pipeline 2: OAK-D Pro RGB -> SegFormer-B0 (TRT FP16, ~25ms @ 5 Hz) -> /terrain/mask
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

# 8-class terrain taxonomy from autonomy-stack-architecture.md Section 2.3
TERRAIN_CLASSES = {
    0: ('paved_path', 0),       # Full speed — concrete, asphalt, brick
    1: ('grass', 40),           # Reduced speed (0.3 m/s)
    2: ('gravel_dirt', 50),     # Reduced speed
    3: ('stairs_curb', 254),    # Avoid (unless step-capable)
    4: ('water_mud', 254),      # Avoid
    5: ('vegetation', 200),     # Avoid — bushes, flower beds
    6: ('road_parking', 254),   # Avoid — vehicle traffic
    7: ('unknown', 128),        # Uncertain — slow and cautious
}


class SegmentationNode(Node):
    """SegFormer-B0 terrain segmentation node.

    Runs at 5 Hz (every 6th frame at 30 Hz input) to minimize GPU load.
    Terrain changes slowly, so high framerate is unnecessary.

    GPU budget: ~10% of Orin Nano Super at 512x512 FP16.
    """

    def __init__(self):
        super().__init__('segmentation_node')

        # Parameters
        self.declare_parameter('model_path', '/opt/models/segmentation/segformer_b0_terrain_v1.engine')
        self.declare_parameter('input_width', 512)
        self.declare_parameter('input_height', 512)
        self.declare_parameter('inference_rate', 5.0)  # Hz
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('enabled', True)  # Can disable during picking

        self.model_path = self.get_parameter('model_path').value
        self.input_w = self.get_parameter('input_width').value
        self.input_h = self.get_parameter('input_height').value
        self.target_rate = self.get_parameter('inference_rate').value
        self.enabled = self.get_parameter('enabled').value

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriber
        self.rgb_sub = self.create_subscription(
            Image,
            '/oak/rgb/image',
            self.rgb_callback,
            sensor_qos,
        )

        # Publishers
        self.mask_pub = self.create_publisher(Image, '/terrain/mask', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/terrain/costmap', 10)

        # Rate limiting: only process at target_rate Hz
        self.last_inference_time = self.get_clock().now()
        self.min_interval_ns = int(1e9 / self.target_rate)

        # TODO: Load TensorRT engine
        # self.model = self._load_tensorrt_engine(self.model_path)

        self.get_logger().info(
            f'SegmentationNode initialized — model: {self.model_path}, '
            f'input: {self.input_w}x{self.input_h}, rate: {self.target_rate} Hz'
        )

    def rgb_callback(self, msg: Image):
        """Process RGB frame through SegFormer-B0 at limited rate."""
        if not self.enabled:
            return

        # Rate limiting — only run at target_rate Hz
        now = self.get_clock().now()
        elapsed = (now - self.last_inference_time).nanoseconds
        if elapsed < self.min_interval_ns:
            return
        self.last_inference_time = now

        # TODO: Convert ROS Image to numpy via cv_bridge
        # frame = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # TODO: Preprocess — resize to 512x512, normalize with ImageNet stats
        # input_tensor = self._preprocess(frame)

        # TODO: Run TensorRT FP16 inference (~25ms)
        # seg_logits = self.model.infer(input_tensor)

        # TODO: Argmax to get class mask (HxW, uint8)
        # seg_mask = np.argmax(seg_logits, axis=0).astype(np.uint8)

        # TODO: Publish segmentation mask
        # mask_msg = bridge.cv2_to_imgmsg(seg_mask, encoding='mono8')
        # mask_msg.header = msg.header
        # self.mask_pub.publish(mask_msg)

        # TODO: Convert mask to Nav2 costmap layer
        # self._publish_costmap(seg_mask, msg.header)

        self.get_logger().debug('Terrain segmentation inference complete')

    def _publish_costmap(self, seg_mask, header):
        """Convert segmentation mask to OccupancyGrid for Nav2.

        TODO: Implement:
          1. Map each terrain class to a costmap value via TERRAIN_CLASSES
          2. Resize to costmap resolution (e.g., 0.05 m/cell)
          3. Transform from camera frame to map frame via tf2
          4. Publish as OccupancyGrid
        """
        costmap = OccupancyGrid()
        costmap.header = header
        costmap.header.frame_id = 'map'
        # TODO: Fill costmap data from seg_mask
        self.costmap_pub.publish(costmap)

    def _load_tensorrt_engine(self, engine_path: str):
        """Load SegFormer-B0 TensorRT FP16 engine.

        TODO: Implement using tensorrt Python bindings.
        Note: SegFormer-B0 does NOT support INT8 via TAO as of 2025.
        Must use FP16 precision.
        """
        raise NotImplementedError('TensorRT engine loading not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
