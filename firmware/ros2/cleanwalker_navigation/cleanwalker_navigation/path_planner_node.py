#!/usr/bin/env python3
"""
Coverage Path Planner Node — CleanWalker CW-1

Generates coverage paths over the operating area using Boustrophedon
Cell Decomposition (BCD). Sends waypoints to Nav2 for execution.

Subscribes:
  /map               (nav_msgs/OccupancyGrid) — SLAM occupancy grid
  /odometry/filtered (nav_msgs/Odometry)      — robot pose from EKF
  /detections        (vision_msgs/Detection3DArray) — litter detections (interrupts)

Publishes:
  /coverage/path     (nav_msgs/Path)          — planned coverage path
  /goal_pose         (geometry_msgs/PoseStamped) — next waypoint for Nav2

Architecture reference: docs/technical/autonomy-stack-architecture.md Section 4
  Coverage: BCD (Fields2Cover library) -> waypoints -> Nav2 NavigateToPose
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray


class PathPlannerNode(Node):
    """Boustrophedon Cell Decomposition coverage planner.

    Decomposes free space from the SLAM map into trapezoidal cells,
    generates back-and-forth sweep paths within each cell, and connects
    cells via shortest-path tour.

    Replanning rate: ~0.1 Hz (on area completion or new obstacle).
    """

    def __init__(self):
        super().__init__('path_planner_node')

        # Parameters
        self.declare_parameter('sweep_width', 0.8)      # meters (sensor FOV width)
        self.declare_parameter('overlap_ratio', 0.1)     # 10% overlap between sweeps
        self.declare_parameter('robot_radius', 0.3)      # meters (for inflation)
        self.declare_parameter('replan_on_detection', True)
        self.declare_parameter('operating_bounds', [0.0, 0.0, 50.0, 50.0])  # [x_min, y_min, x_max, y_max]

        self.sweep_width = self.get_parameter('sweep_width').value
        self.overlap = self.get_parameter('overlap_ratio').value
        self.robot_radius = self.get_parameter('robot_radius').value

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.detection_sub = self.create_subscription(
            Detection3DArray, '/detections', self.detection_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/coverage/path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # State
        self.current_map = None
        self.current_pose = None
        self.coverage_path = []
        self.current_waypoint_idx = 0
        self.is_executing = False

        # Timer for waypoint advancement (check at 1 Hz)
        self.waypoint_timer = self.create_timer(1.0, self.advance_waypoint)

        self.get_logger().info(
            f'PathPlannerNode initialized — sweep: {self.sweep_width}m, '
            f'overlap: {self.overlap * 100}%, robot radius: {self.robot_radius}m'
        )

    def map_callback(self, msg: OccupancyGrid):
        """Receive updated occupancy grid and replan coverage path."""
        self.current_map = msg

        if not self.is_executing:
            self._generate_coverage_path()

    def odom_callback(self, msg: Odometry):
        """Track robot pose for waypoint advancement."""
        self.current_pose = msg.pose.pose

    def detection_callback(self, msg: Detection3DArray):
        """Handle litter detection interrupts.

        When litter is detected, the behavior tree takes over control
        (approach + grasp). This node pauses coverage and resumes
        from the last waypoint after the behavior tree returns control.
        """
        if len(msg.detections) > 0:
            self.get_logger().info(
                f'Litter detected ({len(msg.detections)} items) — '
                'behavior tree will handle approach/grasp'
            )

    def advance_waypoint(self):
        """Check if robot has reached current waypoint, send next one."""
        if not self.is_executing or not self.coverage_path:
            return

        if self.current_pose is None:
            return

        # TODO: Check distance to current waypoint
        # if distance(self.current_pose, self.coverage_path[self.current_waypoint_idx]) < 0.5:
        #     self.current_waypoint_idx += 1
        #     if self.current_waypoint_idx >= len(self.coverage_path):
        #         self.get_logger().info('Coverage path complete!')
        #         self.is_executing = False
        #         return
        #     self._send_next_waypoint()

        pass

    def _generate_coverage_path(self):
        """Generate BCD coverage path from occupancy grid.

        TODO: Implement Boustrophedon Cell Decomposition:
          1. Extract free space polygon from occupancy grid
          2. Decompose into trapezoidal cells using vertical slices
          3. Generate boustrophedon (back-and-forth) path within each cell
             - Sweep spacing = sweep_width * (1 - overlap_ratio)
          4. Connect cells via shortest-path tour (TSP approximation)
          5. Store as list of PoseStamped waypoints

        Libraries:
          - Fields2Cover (C++, has Python bindings): BCD + swath generation
          - full_coverage_path_planner (ROS2 Nav2 plugin)
          - Custom implementation on the occupancy grid
        """
        if self.current_map is None:
            return

        self.get_logger().info('Generating coverage path via BCD...')

        # TODO: Implement BCD algorithm
        # cells = boustrophedon_decomposition(self.current_map)
        # for cell in cells:
        #     waypoints = generate_sweeps(cell, self.sweep_width, self.overlap)
        #     self.coverage_path.extend(waypoints)

        # Publish path for visualization
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        # TODO: Fill path_msg.poses with coverage_path waypoints
        self.path_pub.publish(path_msg)

        if self.coverage_path:
            self.current_waypoint_idx = 0
            self.is_executing = True
            self._send_next_waypoint()

    def _send_next_waypoint(self):
        """Publish next waypoint as Nav2 goal."""
        if self.current_waypoint_idx >= len(self.coverage_path):
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        # TODO: Fill from coverage_path[self.current_waypoint_idx]
        self.goal_pub.publish(goal)

        self.get_logger().info(
            f'Sent waypoint {self.current_waypoint_idx + 1}/{len(self.coverage_path)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
