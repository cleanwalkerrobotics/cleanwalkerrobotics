#!/usr/bin/env python3
"""
Arm Controller Node — CleanWalker CW-1

Analytical IK solver for the 5-DOF arm (turret_yaw, shoulder_pitch,
elbow_pitch, wrist_pitch, gripper) and joint trajectory execution.

Subscribes:
  /grasp_pose         (geometry_msgs/PoseStamped) — target grasp pose from planner
  /joint_states       (sensor_msgs/JointState)    — current joint positions

Publishes:
  /arm/joint_commands (trajectory_msgs/JointTrajectory) — joint trajectory commands
  /arm/status         (std_msgs/String)                 — arm controller status

Architecture reference: docs/technical/autonomy-stack-architecture.md Section 5.3
  Phase 1: Analytical IK for 5-DOF arm (~100 lines, <1ms solve time)
  Phase 2: MoveIt2 for collision-aware motion planning
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from builtin_interfaces.msg import Duration

# Arm link lengths from URDF (meters)
ARM_TURRET_HEIGHT = 0.05    # turret cylinder height
ARM_UPPER_LENGTH = 0.18     # upper arm link
ARM_FOREARM_LENGTH = 0.18   # forearm link
ARM_WRIST_LENGTH = 0.05     # wrist link
ARM_GRIPPER_LENGTH = 0.10   # gripper link
ARM_BASE_OFFSET = [0.15, 0.0, 0.06]  # arm base relative to body center

# Joint names matching URDF
ARM_JOINTS = [
    'arm_turret_yaw_joint',
    'arm_shoulder_pitch_joint',
    'arm_elbow_pitch_joint',
    'arm_wrist_pitch_joint',
    'arm_gripper_joint',
]

# Joint limits from URDF (radians)
JOINT_LIMITS = {
    'arm_turret_yaw_joint':      (-math.pi, math.pi),
    'arm_shoulder_pitch_joint':  (-0.7854, math.pi),
    'arm_elbow_pitch_joint':     (0.0, 2.6180),
    'arm_wrist_pitch_joint':     (-1.5708, 1.5708),
    'arm_gripper_joint':         (0.0, 1.0472),
}


class ArmControllerNode(Node):
    """5-DOF arm controller with analytical inverse kinematics.

    Solves IK for the arm in two stages:
      1. Turret yaw: atan2(y, x) to point arm at target
      2. Shoulder + elbow: 2-link planar IK in the vertical plane
      3. Wrist pitch: align gripper approach angle
      4. Gripper: open/close based on grasp command

    Solve time: <1ms (closed-form analytical solution).
    """

    def __init__(self):
        super().__init__('arm_controller_node')

        # Parameters
        self.declare_parameter('max_velocity', 1.0)      # rad/s
        self.declare_parameter('trajectory_duration', 2.0)  # seconds
        self.declare_parameter('gripper_open_angle', 1.0)   # radians
        self.declare_parameter('gripper_close_angle', 0.0)  # radians

        self.max_velocity = self.get_parameter('max_velocity').value
        self.traj_duration = self.get_parameter('trajectory_duration').value
        self.gripper_open = self.get_parameter('gripper_open_angle').value
        self.gripper_close = self.get_parameter('gripper_close_angle').value

        # Subscribers
        self.grasp_sub = self.create_subscription(
            PoseStamped, '/grasp_pose', self.grasp_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm/joint_commands', 10)
        self.status_pub = self.create_publisher(String, '/arm/status', 10)

        # State
        self.current_joints = {}
        self.is_moving = False

        self.get_logger().info('ArmControllerNode initialized — analytical IK, 5-DOF')

    def grasp_callback(self, msg: PoseStamped):
        """Receive grasp pose, solve IK, and execute trajectory."""
        if self.is_moving:
            self.get_logger().warn('Arm is already moving, ignoring new grasp command')
            return

        self.get_logger().info(
            f'Grasp target received at ({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})'
        )

        # Solve IK
        joint_angles = self._solve_ik(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

        if joint_angles is None:
            self.get_logger().error('IK solution not found — target unreachable')
            status = String()
            status.data = 'ik_failed'
            self.status_pub.publish(status)
            return

        # Execute trajectory: pre-grasp -> grasp -> close gripper
        self._execute_grasp_sequence(joint_angles)

    def joint_state_callback(self, msg: JointState):
        """Track current arm joint positions."""
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINTS:
                self.current_joints[name] = position

    def _solve_ik(self, x: float, y: float, z: float):
        """Solve analytical IK for 5-DOF arm.

        Coordinate frame: arm base at (0.15, 0, 0.06) from body center.
        Input: target (x, y, z) in base_link frame.
        Output: [turret_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, gripper]

        TODO: Implement full analytical IK:
          1. Transform target from base_link to arm_base frame
          2. Turret yaw = atan2(target_y, target_x)
          3. Project to vertical plane for 2-link IK:
             - horizontal_dist = sqrt(x^2 + y^2) - turret_offset
             - vertical_dist = z - arm_base_height
          4. 2-link planar IK:
             - L1 = ARM_UPPER_LENGTH, L2 = ARM_FOREARM_LENGTH + ARM_WRIST_LENGTH
             - cos(elbow) = (h^2 + v^2 - L1^2 - L2^2) / (2*L1*L2)
             - shoulder = atan2(v, h) - atan2(L2*sin(elbow), L1 + L2*cos(elbow))
          5. Wrist pitch = desired approach angle - shoulder - elbow
          6. Validate all joints within limits
        """
        # Transform to arm base frame
        ax = x - ARM_BASE_OFFSET[0]
        ay = y - ARM_BASE_OFFSET[1]
        az = z - ARM_BASE_OFFSET[2] - ARM_TURRET_HEIGHT

        # Step 1: Turret yaw
        turret_yaw = math.atan2(ay, ax)

        # Step 2: Project to vertical plane
        horizontal_dist = math.sqrt(ax**2 + ay**2)
        vertical_dist = az

        # Step 3: 2-link planar IK (shoulder + elbow for upper arm + forearm)
        L1 = ARM_UPPER_LENGTH
        L2 = ARM_FOREARM_LENGTH  # wrist adds to end-effector, not IK chain

        reach_sq = horizontal_dist**2 + vertical_dist**2
        reach = math.sqrt(reach_sq)

        # Check reachability
        if reach > (L1 + L2) or reach < abs(L1 - L2):
            self.get_logger().warn(f'Target at distance {reach:.3f}m is unreachable')
            return None

        # Elbow angle (law of cosines)
        cos_elbow = (reach_sq - L1**2 - L2**2) / (2 * L1 * L2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))  # clamp for numerical safety
        elbow_pitch = math.acos(cos_elbow)

        # Shoulder angle
        beta = math.atan2(vertical_dist, horizontal_dist)
        alpha = math.atan2(L2 * math.sin(elbow_pitch), L1 + L2 * math.cos(elbow_pitch))
        shoulder_pitch = beta + alpha

        # Wrist pitch: point gripper downward for top-down grasp
        wrist_pitch = -(shoulder_pitch + elbow_pitch) + math.pi / 2

        # Clamp to joint limits
        joints = [turret_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, self.gripper_open]
        for i, (name, angle) in enumerate(zip(ARM_JOINTS, joints)):
            lo, hi = JOINT_LIMITS[name]
            if angle < lo or angle > hi:
                self.get_logger().warn(
                    f'Joint {name} angle {math.degrees(angle):.1f}° '
                    f'outside limits [{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°]'
                )
                joints[i] = max(lo, min(hi, angle))

        return joints

    def _execute_grasp_sequence(self, joint_angles: list):
        """Execute pre-grasp -> grasp -> close gripper trajectory.

        TODO: Implement multi-point trajectory:
          1. Move to pre-grasp pose (10cm above grasp point)
          2. Move to grasp pose
          3. Close gripper
          4. Retract to safe position
        """
        self.is_moving = True

        # Build trajectory message
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS

        # Point 1: target joint angles (open gripper)
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = Duration(sec=int(self.traj_duration), nanosec=0)
        traj.points.append(point)

        # Point 2: close gripper
        close_angles = joint_angles.copy()
        close_angles[4] = self.gripper_close  # close gripper
        point2 = JointTrajectoryPoint()
        point2.positions = close_angles
        point2.time_from_start = Duration(sec=int(self.traj_duration + 1), nanosec=0)
        traj.points.append(point2)

        self.trajectory_pub.publish(traj)

        status = String()
        status.data = 'executing_grasp'
        self.status_pub.publish(status)

        self.get_logger().info('Grasp trajectory published')

        # TODO: Monitor trajectory execution and update status
        # For now, mark as complete after publishing
        self.is_moving = False


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
