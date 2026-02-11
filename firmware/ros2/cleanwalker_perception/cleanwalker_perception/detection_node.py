#!/usr/bin/env python3
"""
YOLO26 Litter Detection Node — CleanWalker CW-1

Subscribes to /oak/rgb/image (30 Hz) from OAK-D Pro camera and runs
YOLO26s inference via TensorRT INT8 to detect 10 classes of outdoor litter.

Publishes:
  /detections  (vision_msgs/Detection3DArray)  — 3D detections with depth
  /litter/items (cleanwalker_perception/LitterItem[]) — tracked litter items

Architecture reference: docs/technical/perception-pipeline-architecture.md
  Pipeline 1: OAK-D Pro RGB -> YOLO26s (TRT INT8, 5-10ms) -> /detections
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseStamped

# TODO: Import custom messages once built
# from cleanwalker_perception.msg import Detection, LitterItem

# Litter classes matching our TACO + RoLID-11K training taxonomy
LITTER_CLASSES = [
    'plastic_bottle',
    'can',
    'cigarette_butt',
    'paper',
    'plastic_bag',
    'food_wrapper',
    'glass_bottle',
    'cardboard',
    'styrofoam',
    'other_trash',
]


class DetectionNode(Node):
    """YOLO26s litter detection node.

    Runs TensorRT INT8 inference on incoming RGB frames from OAK-D Pro.
    Target latency: 5-10ms per frame at 640x640 input resolution.
    """

    def __init__(self):
        super().__init__('detection_node')

        # Parameters
        self.declare_parameter('model_path', '/opt/models/detection/yolo26s_litter_v1.engine')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('device', 'cuda:0')

        self.model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.nms_thresh = self.get_parameter('nms_threshold').value
        self.input_size = self.get_parameter('input_size').value

        # QoS: best-effort for high-frequency sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/oak/rgb/image',
            self.rgb_callback,
            sensor_qos,
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/oak/stereo/depth',
            self.depth_callback,
            sensor_qos,
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection3DArray,
            '/detections',
            10,
        )

        # State
        self.latest_depth = None
        self.model = None
        self.frame_count = 0

        # TODO: Load TensorRT engine
        # self.model = self._load_tensorrt_engine(self.model_path)

        self.get_logger().info(
            f'DetectionNode initialized — model: {self.model_path}, '
            f'conf: {self.conf_thresh}, input: {self.input_size}x{self.input_size}'
        )

    def rgb_callback(self, msg: Image):
        """Process incoming RGB frame through YOLO26s pipeline."""
        self.frame_count += 1

        # TODO: Convert ROS Image to numpy via cv_bridge
        # bridge = CvBridge()
        # frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # TODO: Preprocess — resize to 640x640, normalize, convert to tensor
        # input_tensor = self._preprocess(frame)

        # TODO: Run TensorRT inference (target: 5-10ms)
        # detections_raw = self.model.infer(input_tensor)

        # TODO: Post-process — YOLO26 is NMS-free, extract boxes + classes + confs
        # detections = self._postprocess(detections_raw)

        # TODO: Project 2D detections to 3D using depth map
        # if self.latest_depth is not None:
        #     detections_3d = self._project_to_3d(detections, self.latest_depth)

        # Publish detections
        det_array = Detection3DArray()
        det_array.header = msg.header

        # TODO: Populate with real detections
        # for det in detections_3d:
        #     d3d = Detection3D()
        #     hyp = ObjectHypothesisWithPose()
        #     hyp.hypothesis.class_id = str(det.class_id)
        #     hyp.hypothesis.score = det.confidence
        #     d3d.results.append(hyp)
        #     d3d.bbox.center.position.x = det.x
        #     d3d.bbox.center.position.y = det.y
        #     d3d.bbox.center.position.z = det.z
        #     det_array.detections.append(d3d)

        self.detection_pub.publish(det_array)

        if self.frame_count % 30 == 0:
            self.get_logger().debug(
                f'Frame {self.frame_count}: published {len(det_array.detections)} detections'
            )

    def depth_callback(self, msg: Image):
        """Cache latest depth frame for 2D->3D projection."""
        self.latest_depth = msg

    def _load_tensorrt_engine(self, engine_path: str):
        """Load a TensorRT engine from disk.

        TODO: Implement using tensorrt Python bindings:
          1. Deserialize engine from .engine file
          2. Create execution context
          3. Allocate input/output device buffers
          4. Return inference callable
        """
        self.get_logger().info(f'Loading TensorRT engine from {engine_path}')
        raise NotImplementedError('TensorRT engine loading not yet implemented')

    def _preprocess(self, frame):
        """Preprocess RGB frame for YOLO26s inference.

        TODO: Implement:
          1. Resize to input_size x input_size (letterbox padding)
          2. BGR -> RGB
          3. Normalize to [0, 1]
          4. HWC -> CHW -> NCHW
          5. Copy to GPU (cuda memcpy)
        """
        raise NotImplementedError

    def _postprocess(self, raw_output):
        """Post-process YOLO26s output.

        YOLO26 is NMS-free (STAL architecture), so no NMS step needed.

        TODO: Implement:
          1. Parse raw tensor output -> bounding boxes
          2. Filter by confidence threshold
          3. Map class indices to LITTER_CLASSES
          4. Return list of Detection objects
        """
        raise NotImplementedError

    def _project_to_3d(self, detections_2d, depth_msg):
        """Project 2D detections to 3D using depth map and camera intrinsics.

        TODO: Implement:
          1. For each detection bbox center, sample depth value
          2. Deproject (u, v, depth) -> (x, y, z) using camera intrinsics
          3. Transform from camera_optical_frame to base_link via tf2
        """
        raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
