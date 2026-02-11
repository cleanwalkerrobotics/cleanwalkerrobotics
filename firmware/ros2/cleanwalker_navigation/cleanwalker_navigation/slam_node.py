#!/usr/bin/env python3
"""
SLAM Node — CleanWalker CW-1

Hybrid cuVSLAM + RTAB-Map SLAM pipeline:
  - cuVSLAM: GPU-accelerated visual odometry (9% GPU, 30 Hz)
  - RTAB-Map: CPU-based mapping + loop closure (5 Hz, 2 cores)
  - robot_localization EKF: fuses VO + IMU + wheel odom (50 Hz)

This node wraps the cuVSLAM visual odometry component and publishes
visual odometry for the EKF. RTAB-Map runs as a separate node.

Subscribes:
  /oak/stereo/left   (sensor_msgs/Image)     — left rectified image
  /oak/stereo/right  (sensor_msgs/Image)     — right rectified image
  /oak/imu           (sensor_msgs/Imu)       — 400 Hz IMU from OAK-D Pro
  /livox/pointcloud  (sensor_msgs/PointCloud2) — 10 Hz Livox Mid-360

Publishes:
  /visual_odom       (nav_msgs/Odometry)     — cuVSLAM visual odometry (30 Hz)
  /map               (nav_msgs/OccupancyGrid) — occupancy grid (via RTAB-Map)

Architecture reference: docs/technical/perception-pipeline-architecture.md
  Pipeline 3: cuVSLAM (GPU, 9%) + RTAB-Map (CPU, 5 Hz) + EKF fusion
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped
import tf2_ros


class SlamNode(Node):
    """cuVSLAM visual odometry wrapper node.

    Wraps NVIDIA Isaac ROS Visual SLAM (cuVSLAM) for GPU-accelerated
    stereo visual-inertial odometry. Feeds into robot_localization EKF
    alongside IMU and wheel/leg odometry.

    GPU budget: ~9% of Orin Nano Super.
    CPU budget: minimal (GPU does the work).
    """

    def __init__(self):
        super().__init__('slam_node')

        # Parameters
        self.declare_parameter('enable_imu_fusion', True)
        self.declare_parameter('enable_localization_n_mapping', True)
        self.declare_parameter('publish_odom_to_base_tf', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.left_sub = self.create_subscription(
            Image, '/oak/stereo/left', self.left_callback, sensor_qos)
        self.right_sub = self.create_subscription(
            Image, '/oak/stereo/right', self.right_callback, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/oak/imu', self.imu_callback, sensor_qos)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/livox/pointcloud', self.lidar_callback, sensor_qos)

        # Publishers
        self.vo_pub = self.create_publisher(Odometry, '/visual_odom', 10)

        # TF broadcaster for odom -> base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # State
        self.latest_left = None
        self.latest_right = None
        self.latest_imu = None
        self.is_initialized = False
        self.frame_count = 0

        self.get_logger().info(
            f'SlamNode initialized — cuVSLAM wrapper, '
            f'frames: {self.map_frame} -> {self.odom_frame} -> {self.base_frame}'
        )

    def left_callback(self, msg: Image):
        """Cache left stereo image and trigger VO if pair is ready."""
        self.latest_left = msg
        self._try_process_stereo_pair()

    def right_callback(self, msg: Image):
        """Cache right stereo image."""
        self.latest_right = msg

    def imu_callback(self, msg: Imu):
        """Cache latest IMU reading for visual-inertial fusion."""
        self.latest_imu = msg

    def lidar_callback(self, msg: PointCloud2):
        """Forward LiDAR data to RTAB-Map for scan matching.

        TODO: RTAB-Map subscribes directly to /livox/pointcloud.
        This callback is for any pre-processing we need (e.g., downsampling).
        """
        pass

    def _try_process_stereo_pair(self):
        """Run cuVSLAM on synchronized stereo pair + IMU."""
        if self.latest_left is None or self.latest_right is None:
            return

        self.frame_count += 1

        # TODO: Feed stereo pair + IMU to cuVSLAM via Isaac ROS
        #
        # In production, cuVSLAM runs as isaac_ros_visual_slam node.
        # This wrapper manages lifecycle and parameter passing.
        #
        # cuVSLAM outputs:
        #   - Visual odometry (30 Hz, /visual_odom)
        #   - odom -> base_link TF transform
        #   - Tracking status and quality metrics

        # Publish visual odometry (placeholder)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.latest_left.header.stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # TODO: Fill with actual cuVSLAM output
        # odom_msg.pose.pose = cuslam_result.pose
        # odom_msg.twist.twist = cuslam_result.velocity

        self.vo_pub.publish(odom_msg)

        # Publish TF: odom -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.latest_left.header.stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        # TODO: Fill transform from cuVSLAM output
        self.tf_broadcaster.sendTransform(tf_msg)

        # Clear consumed frames
        self.latest_left = None

        if self.frame_count % 30 == 0:
            self.get_logger().debug(f'VO frame {self.frame_count} processed')


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
