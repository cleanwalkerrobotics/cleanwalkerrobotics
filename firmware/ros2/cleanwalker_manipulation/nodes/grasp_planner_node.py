#!/usr/bin/env python3
"""
Grasp Planner Node — CleanWalker CW-1

Phase 1: Rule-based grasp primitives per litter class.
Phase 2: GR-ConvNet v2 learned grasp prediction (TRT INT8, 5-10ms).

Subscribes:
  /detections        (vision_msgs/Detection3DArray) — 3D litter detections
  /depth/metric      (sensor_msgs/Image)            — metric depth map

Publishes:
  /grasp_pose        (geometry_msgs/PoseStamped)    — target grasp pose for arm
  /grasp/status      (std_msgs/String)              — grasp planner status

Architecture reference: docs/technical/perception-pipeline-architecture.md
  Pipeline 1 (grasp stage): ROI Crop -> GR-ConvNet v2 (TRT, 5-10ms) -> grasp pose
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from std_msgs.msg import String


# Rule-based grasp primitives per litter class
# Phase 1: simple approach angles and apertures
GRASP_PRIMITIVES = {
    'plastic_bottle':  {'strategy': 'top_down',  'approach_angle': 0.0,    'aperture': 0.070},
    'can':             {'strategy': 'top_down',  'approach_angle': 0.0,    'aperture': 0.065},
    'glass_bottle':    {'strategy': 'top_down',  'approach_angle': 0.0,    'aperture': 0.075},
    'cigarette_butt':  {'strategy': 'pinch',     'approach_angle': 0.785,  'aperture': 0.015},
    'paper':           {'strategy': 'scoop',     'approach_angle': 1.396,  'aperture': 0.060},
    'plastic_bag':     {'strategy': 'scoop',     'approach_angle': 1.396,  'aperture': 0.080},
    'food_wrapper':    {'strategy': 'scoop',     'approach_angle': 1.222,  'aperture': 0.060},
    'cardboard':       {'strategy': 'edge',      'approach_angle': 0.524,  'aperture': 0.100},
    'styrofoam':       {'strategy': 'top_down',  'approach_angle': 0.0,    'aperture': 0.080},
    'other_trash':     {'strategy': 'top_down',  'approach_angle': 0.0,    'aperture': 0.070},
}


class GraspPlannerNode(Node):
    """Grasp planning node with rule-based primitives and future GR-ConvNet v2.

    Phase 1: Maps litter class -> pre-defined grasp approach (strategy, angle, aperture).
    Phase 2: Runs GR-ConvNet v2 on 224x224 RGB-D crop for learned grasp prediction.

    GR-ConvNet v2 specs: 1.9M params, 5-10ms TRT INT8, 95.4% success on Cornell dataset.
    """

    def __init__(self):
        super().__init__('grasp_planner_node')

        # Parameters
        self.declare_parameter('use_learned_grasping', False)  # Phase 2
        self.declare_parameter('model_path', '/opt/models/grasping/grconvnet_v2_grasp_v1.engine')
        self.declare_parameter('grasp_height_offset', 0.05)  # meters above object
        self.declare_parameter('pre_grasp_distance', 0.10)   # meters above grasp pose
        self.declare_parameter('max_grasp_attempts', 3)

        self.use_learned = self.get_parameter('use_learned_grasping').value
        self.model_path = self.get_parameter('model_path').value
        self.height_offset = self.get_parameter('grasp_height_offset').value
        self.pre_grasp_dist = self.get_parameter('pre_grasp_distance').value
        self.max_attempts = self.get_parameter('max_grasp_attempts').value

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection3DArray, '/detections', self.detection_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/depth/metric', self.depth_callback, 10)

        # Publishers
        self.grasp_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.status_pub = self.create_publisher(String, '/grasp/status', 10)

        # State
        self.latest_depth = None
        self.current_target = None
        self.attempt_count = 0

        mode = 'GR-ConvNet v2 (learned)' if self.use_learned else 'rule-based primitives'
        self.get_logger().info(f'GraspPlannerNode initialized — mode: {mode}')

    def detection_callback(self, msg: Detection3DArray):
        """Select best litter target and plan grasp.

        Selection criteria:
          1. Highest confidence detection
          2. Within arm workspace (reachable)
          3. Not previously failed (skip after max_attempts)
        """
        if len(msg.detections) == 0:
            return

        # TODO: Select best target from detections
        # best = max(msg.detections, key=lambda d: d.results[0].hypothesis.score)
        # class_name = LITTER_CLASSES[int(best.results[0].hypothesis.class_id)]
        # position = best.bbox.center.position

        # TODO: Check if target is within arm workspace
        # if not self._is_reachable(position):
        #     return

        # Plan grasp
        # if self.use_learned:
        #     grasp_pose = self._plan_learned_grasp(best, self.latest_depth)
        # else:
        #     grasp_pose = self._plan_rule_based_grasp(class_name, position)
        #
        # self.grasp_pub.publish(grasp_pose)

        status = String()
        status.data = 'grasp_planned'
        self.status_pub.publish(status)

    def depth_callback(self, msg: Image):
        """Cache latest depth map for grasp planning."""
        self.latest_depth = msg

    def _plan_rule_based_grasp(self, class_name: str, position) -> PoseStamped:
        """Plan grasp using rule-based primitives.

        TODO: Implement:
          1. Look up GRASP_PRIMITIVES[class_name]
          2. Compute approach vector based on strategy:
             - top_down: vertical approach, gripper aligned with object major axis
             - pinch: 45-degree approach for small objects
             - scoop: near-horizontal approach with ground contact
             - edge: angled approach to grip edge of flat objects
          3. Set pre-grasp pose (offset above grasp point)
          4. Return PoseStamped in base_link frame
        """
        primitive = GRASP_PRIMITIVES.get(class_name, GRASP_PRIMITIVES['other_trash'])

        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.header.frame_id = 'base_link'

        # TODO: Compute actual grasp pose from primitive + object position
        # grasp_pose.pose.position = position
        # grasp_pose.pose.position.z += self.height_offset
        # grasp_pose.pose.orientation = angle_to_quaternion(primitive['approach_angle'])

        self.get_logger().info(
            f'Planned {primitive["strategy"]} grasp for {class_name} '
            f'(aperture: {primitive["aperture"]*1000:.0f}mm)'
        )
        return grasp_pose

    def _plan_learned_grasp(self, detection, depth_msg) -> PoseStamped:
        """Plan grasp using GR-ConvNet v2 on RGB-D crop.

        TODO (Phase 2): Implement:
          1. Crop 224x224 RGB-D region around detection center
          2. Run GR-ConvNet v2 TensorRT inference (5-10ms)
          3. Extract best grasp: (x, y, z, theta, width)
          4. Project from image to base_link frame
          5. Return as PoseStamped

        Model: 1.9M params, trained on Cornell/Jacquard grasping datasets.
        Note: May need fine-tuning on outdoor litter (trained on tabletop objects).
        """
        raise NotImplementedError('GR-ConvNet v2 not yet integrated (Phase 2)')

    def _is_reachable(self, position) -> bool:
        """Check if target position is within arm workspace.

        Arm specs from URDF:
          - Turret yaw: ±180° (full rotation)
          - Shoulder pitch: -45° to +180°
          - Elbow pitch: 0° to 150°
          - Wrist pitch: ±90°
          - Max reach: ~0.46m (0.18 upper + 0.18 forearm + 0.10 gripper)
          - Arm base: (0.15, 0, 0.06) from body center
        """
        # TODO: Implement workspace check using arm kinematics
        max_reach = 0.46  # meters
        # distance = sqrt(position.x**2 + position.y**2 + position.z**2)
        # return distance <= max_reach
        return True


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
