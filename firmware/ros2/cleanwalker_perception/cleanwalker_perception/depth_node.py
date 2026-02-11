#!/usr/bin/env python3
"""
Stereo Depth Processing Node — CleanWalker CW-1

Wraps the OAK-D Pro hardware stereo depth and (Phase 2) Isaac ROS ESS
to produce metric depth maps with confidence filtering.

Subscribes:
  /oak/stereo/depth  (sensor_msgs/Image)     — raw stereo depth from OAK-D VPU
  /oak/stereo/left   (sensor_msgs/Image)     — left rectified (for ESS)
  /oak/stereo/right  (sensor_msgs/Image)     — right rectified (for ESS)

Publishes:
  /depth/metric      (sensor_msgs/Image)     — filtered metric depth map (30 Hz)
  /depth/confidence  (sensor_msgs/Image)     — per-pixel confidence (Phase 2, ESS)

Architecture reference: docs/technical/perception-pipeline-architecture.md
  Pipeline 1: OAK-D Pro Stereo -> Isaac ROS ESS (~20ms) -> /depth/metric
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class DepthNode(Node):
    """Stereo depth processing and filtering node.

    Phase 1: Pass through OAK-D Pro hardware stereo depth (runs on VPU, 0% GPU).
    Phase 2: Add Isaac ROS ESS for learned stereo with confidence maps.
    """

    def __init__(self):
        super().__init__('depth_node')

        # Parameters
        self.declare_parameter('use_ess', False)  # Phase 2: enable Isaac ROS ESS
        self.declare_parameter('min_depth', 0.2)  # meters
        self.declare_parameter('max_depth', 10.0)  # meters
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('median_filter_size', 5)

        self.use_ess = self.get_parameter('use_ess').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value

        # QoS for sensor streams
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers — hardware stereo depth from OAK-D Pro
        self.depth_raw_sub = self.create_subscription(
            Image,
            '/oak/stereo/depth',
            self.depth_raw_callback,
            sensor_qos,
        )

        # Phase 2: stereo pair for ESS
        # self.left_sub = self.create_subscription(
        #     Image, '/oak/stereo/left', self.left_callback, sensor_qos)
        # self.right_sub = self.create_subscription(
        #     Image, '/oak/stereo/right', self.right_callback, sensor_qos)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/oak/rgb/camera_info',
            self.camera_info_callback,
            10,
        )

        # Publishers
        self.depth_pub = self.create_publisher(Image, '/depth/metric', 10)
        self.confidence_pub = self.create_publisher(Image, '/depth/confidence', 10)

        # State
        self.camera_info = None
        self.frame_count = 0

        mode = 'ESS (learned stereo)' if self.use_ess else 'OAK-D Pro VPU (hardware stereo)'
        self.get_logger().info(
            f'DepthNode initialized — mode: {mode}, '
            f'range: [{self.min_depth}, {self.max_depth}]m'
        )

    def depth_raw_callback(self, msg: Image):
        """Process raw stereo depth from OAK-D Pro VPU."""
        self.frame_count += 1

        # TODO: Convert to numpy via cv_bridge
        # bridge = CvBridge()
        # depth_raw = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # TODO: Apply depth filtering
        # 1. Clip to [min_depth, max_depth] range
        # 2. Apply median filter to reduce noise
        # 3. Fill small holes via interpolation
        # 4. Convert mm -> meters if needed (OAK-D outputs uint16 mm)

        # TODO: Generate confidence map (Phase 1: binary valid/invalid)
        # confidence = (depth_raw > min_depth) & (depth_raw < max_depth)

        # Publish filtered depth
        filtered_msg = msg  # TODO: replace with actual filtered depth
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(filtered_msg)

        if self.frame_count % 30 == 0:
            self.get_logger().debug(f'Depth frame {self.frame_count} processed')

    def camera_info_callback(self, msg: CameraInfo):
        """Cache camera intrinsics for depth projection."""
        if self.camera_info is None:
            self.get_logger().info(
                f'Camera info received: {msg.width}x{msg.height}, '
                f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}'
            )
        self.camera_info = msg

    def _run_ess_inference(self, left_img, right_img):
        """Run Isaac ROS ESS learned stereo depth.

        TODO (Phase 2): Implement using isaac_ros_ess_bm package or
        direct TensorRT inference on ESS model (~20ms, FP16).

        ESS provides:
          - Dense metric depth map (higher quality than VPU stereo)
          - Per-pixel confidence map (filter unreliable depths)

        Requires: isaac_ros_dnn_stereo_depth package
        Model: /opt/models/depth/ess_stereo_v1.engine
        """
        raise NotImplementedError('Isaac ROS ESS not yet integrated (Phase 2)')


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
