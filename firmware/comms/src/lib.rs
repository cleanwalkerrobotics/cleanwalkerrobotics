// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

//! CleanWalker CW-1 Communications Module
//!
//! Provides communication interfaces for the CW-1 robot:
//! - ROS2 bridge for sensor/actuator data exchange
//! - 4G LTE cellular connectivity
//! - Mesh networking for multi-robot coordination
//!
//! # ROS2 Integration
//!
//! The ROS2 bridge uses `rclrs` (ROS2 Rust client library) for type-safe
//! pub/sub communication. To enable ROS2 support, add `rclrs` as a
//! dependency:
//!
//! ```toml
//! [dependencies]
//! # rclrs = "0.4"  # Uncomment when ROS2 workspace is configured
//! ```
//!
//! ## Topics
//!
//! ### Subscribers
//! - `/arm/joint_commands` ([`ArmJointCommand`]) - Joint position targets for the 5-DOF arm
//! - `/coverage/path` ([`CoveragePath`]) - Planned coverage path waypoints
//!
//! ### Publishers
//! - `/arm/status` ([`ArmStatus`]) - Current arm joint positions and gripper state
//! - `/grasp/status` ([`GraspStatus`]) - Grasp detection and completion status
//! - `/odom` ([`Odometry`]) - Robot odometry (position, orientation, velocity)

// ─── ROS2 Message Types ──────────────────────────────────────────
//
// These types mirror standard ROS2 message definitions. When rclrs is
// added as a dependency, replace these with the generated types from:
// - std_msgs, geometry_msgs, sensor_msgs, nav_msgs
// - cleanwalker_msgs (custom package)

/// 3D vector (mirrors geometry_msgs/Vector3).
#[derive(Debug, Clone, Copy, Default)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Quaternion orientation (mirrors geometry_msgs/Quaternion).
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
}

/// Pose in 3D space (mirrors geometry_msgs/Pose).
#[derive(Debug, Clone, Copy, Default)]
pub struct Pose {
    pub position: Vector3,
    pub orientation: Quaternion,
}

/// Twist: linear + angular velocity (mirrors geometry_msgs/Twist).
#[derive(Debug, Clone, Copy, Default)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

/// Timestamp (mirrors builtin_interfaces/Time).
#[derive(Debug, Clone, Copy, Default)]
pub struct Timestamp {
    pub sec: i32,
    pub nanosec: u32,
}

/// Message header (mirrors std_msgs/Header).
#[derive(Debug, Clone, Default)]
pub struct Header {
    pub stamp: Timestamp,
    pub frame_id: String,
}

// ─── Custom Message Types ────────────────────────────────────────

/// Arm joint position command message.
///
/// Subscribed on `/arm/joint_commands`.
/// Sent by the manipulation planner to command arm joint positions.
#[derive(Debug, Clone, Default)]
pub struct ArmJointCommand {
    pub header: Header,
    /// Target positions in radians: [turret_yaw, shoulder_pitch,
    /// elbow_pitch, wrist_pitch, gripper].
    pub positions: [f64; 5],
    /// Target velocities in rad/s (0.0 = use default speed).
    pub velocities: [f64; 5],
}

/// Arm status feedback message.
///
/// Published on `/arm/status`.
/// Reports current arm joint positions and gripper state.
#[derive(Debug, Clone, Default)]
pub struct ArmStatus {
    pub header: Header,
    /// Current joint positions in radians.
    pub positions: [f64; 5],
    /// Current joint motor currents in amps.
    pub currents: [f64; 5],
    /// Whether the gripper is currently holding an object.
    pub gripper_holding: bool,
}

/// Grasp state enumeration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GraspState {
    /// No grasp in progress.
    #[default]
    Idle,
    /// Arm approaching target.
    Approaching,
    /// Gripper closing on target.
    Closing,
    /// Object successfully grasped.
    Holding,
    /// Grasp failed (object missed or dropped).
    Failed,
}

/// Grasp status feedback message.
///
/// Published on `/grasp/status`.
/// Reports grasp detection and completion.
#[derive(Debug, Clone)]
pub struct GraspStatus {
    pub header: Header,
    /// Current grasp state.
    pub state: GraspState,
    /// Confidence of grasp detection (0.0 to 1.0).
    pub confidence: f64,
    /// Label of the grasped object (from perception pipeline).
    pub object_label: String,
}

impl Default for GraspStatus {
    fn default() -> Self {
        Self {
            header: Header::default(),
            state: GraspState::default(),
            confidence: 0.0,
            object_label: String::new(),
        }
    }
}

/// Coverage path message.
///
/// Subscribed on `/coverage/path`.
/// Sent by the navigation planner with waypoints for area coverage.
#[derive(Debug, Clone, Default)]
pub struct CoveragePath {
    pub header: Header,
    /// Ordered list of waypoints to visit.
    pub waypoints: Vec<Pose>,
    /// Whether to loop back to the first waypoint on completion.
    pub loop_path: bool,
}

/// Odometry message (mirrors nav_msgs/Odometry).
///
/// Published on `/odom`.
/// Reports robot position and velocity in the odom frame.
#[derive(Debug, Clone, Default)]
pub struct Odometry {
    pub header: Header,
    /// Child frame (typically "base_link").
    pub child_frame_id: String,
    /// Robot pose in the odom frame.
    pub pose: Pose,
    /// Robot velocity in the base_link frame.
    pub twist: Twist,
}

// ─── ROS2 Bridge ─────────────────────────────────────────────────

/// Callback type for received arm joint commands.
pub type ArmCommandCallback = Box<dyn Fn(ArmJointCommand) + Send>;

/// Callback type for received coverage paths.
pub type CoveragePathCallback = Box<dyn Fn(CoveragePath) + Send>;

/// ROS2 bridge for the CW-1 robot firmware.
///
/// Manages subscriptions and publishers for communication with the
/// ROS2 navigation, perception, and manipulation stacks.
///
/// # Example
///
/// ```rust,no_run
/// use cw_comms::Ros2Bridge;
///
/// let mut bridge = Ros2Bridge::new("cw1_firmware");
///
/// bridge.set_arm_command_callback(|cmd| {
///     println!("Received arm command: {:?}", cmd.positions);
/// });
///
/// // In the main loop:
/// // bridge.spin_once();
/// // bridge.publish_arm_status(&status);
/// // bridge.publish_odometry(&odom);
/// ```
///
/// # ROS2 Dependency
///
/// This module defines the interface but does not depend on `rclrs` at
/// compile time. To enable actual ROS2 communication:
///
/// 1. Install ROS2 Humble and the `rclrs` crate
/// 2. Add to Cargo.toml: `rclrs = "0.4"`
/// 3. Replace the stub implementations with rclrs node/pub/sub calls
pub struct Ros2Bridge {
    /// ROS2 node name.
    node_name: String,
    /// Latest arm status for publishing.
    latest_arm_status: Option<ArmStatus>,
    /// Latest grasp status for publishing.
    latest_grasp_status: Option<GraspStatus>,
    /// Latest odometry for publishing.
    latest_odometry: Option<Odometry>,
    /// Callback for arm joint commands.
    arm_command_cb: Option<ArmCommandCallback>,
    /// Callback for coverage path.
    coverage_path_cb: Option<CoveragePathCallback>,
}

impl Ros2Bridge {
    /// Create a new ROS2 bridge with the given node name.
    ///
    /// # Arguments
    /// * `node_name` - ROS2 node name (e.g., "cw1_firmware")
    ///
    /// When rclrs is available, this will call `rclrs::init()` and
    /// create a node with the given name.
    pub fn new(node_name: &str) -> Self {
        // TODO: Initialize rclrs node
        // let context = rclrs::Context::new(std::env::args()).unwrap();
        // let node = rclrs::Node::new(&context, node_name).unwrap();
        Self {
            node_name: node_name.to_string(),
            latest_arm_status: None,
            latest_grasp_status: None,
            latest_odometry: None,
            arm_command_cb: None,
            coverage_path_cb: None,
        }
    }

    /// Returns the ROS2 node name.
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    // ── Subscriber Setup ─────────────────────────────────────────

    /// Register a callback for `/arm/joint_commands` messages.
    ///
    /// When rclrs is available, this creates a subscription on the
    /// `/arm/joint_commands` topic with QoS profile RELIABLE.
    pub fn set_arm_command_callback<F>(&mut self, callback: F)
    where
        F: Fn(ArmJointCommand) + Send + 'static,
    {
        // TODO: Create rclrs subscription
        // self.node.create_subscription("/arm/joint_commands", qos, callback);
        self.arm_command_cb = Some(Box::new(callback));
    }

    /// Register a callback for `/coverage/path` messages.
    ///
    /// When rclrs is available, this creates a subscription on the
    /// `/coverage/path` topic with QoS profile RELIABLE.
    pub fn set_coverage_path_callback<F>(&mut self, callback: F)
    where
        F: Fn(CoveragePath) + Send + 'static,
    {
        // TODO: Create rclrs subscription
        // self.node.create_subscription("/coverage/path", qos, callback);
        self.coverage_path_cb = Some(Box::new(callback));
    }

    // ── Publishers ───────────────────────────────────────────────

    /// Publish arm status on `/arm/status`.
    pub fn publish_arm_status(&mut self, status: &ArmStatus) {
        // TODO: Publish via rclrs publisher
        // self.arm_status_pub.publish(status);
        self.latest_arm_status = Some(status.clone());
    }

    /// Publish grasp status on `/grasp/status`.
    pub fn publish_grasp_status(&mut self, status: &GraspStatus) {
        // TODO: Publish via rclrs publisher
        // self.grasp_status_pub.publish(status);
        self.latest_grasp_status = Some(status.clone());
    }

    /// Publish odometry on `/odom`.
    pub fn publish_odometry(&mut self, odom: &Odometry) {
        // TODO: Publish via rclrs publisher
        // self.odom_pub.publish(odom);
        self.latest_odometry = Some(odom.clone());
    }

    // ── Spin ─────────────────────────────────────────────────────

    /// Process pending ROS2 callbacks (non-blocking).
    ///
    /// Call this in the main control loop to process incoming messages.
    /// When rclrs is available, this calls `rclrs::spin_some()`.
    pub fn spin_once(&self) {
        // TODO: rclrs::spin_some(&self.node);
    }

    /// Get the latest published arm status (for testing/inspection).
    pub fn latest_arm_status(&self) -> Option<&ArmStatus> {
        self.latest_arm_status.as_ref()
    }

    /// Get the latest published grasp status (for testing/inspection).
    pub fn latest_grasp_status(&self) -> Option<&GraspStatus> {
        self.latest_grasp_status.as_ref()
    }

    /// Get the latest published odometry (for testing/inspection).
    pub fn latest_odometry(&self) -> Option<&Odometry> {
        self.latest_odometry.as_ref()
    }
}

// ─── Communication Channels ──────────────────────────────────────

/// Communication channel type.
pub enum CommChannel {
    /// 4G LTE cellular connection.
    Cellular,
    /// Mesh network for robot-to-robot communication.
    Mesh,
}

/// Manages communication channels for the robot.
pub struct CommsManager {
    active_channel: Option<CommChannel>,
}

impl CommsManager {
    pub fn new() -> Self {
        Self {
            active_channel: None,
        }
    }

    pub fn connect(&mut self, channel: CommChannel) {
        self.active_channel = Some(channel);
    }

    pub fn disconnect(&mut self) {
        self.active_channel = None;
    }

    pub fn is_connected(&self) -> bool {
        self.active_channel.is_some()
    }
}

impl Default for CommsManager {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Tests ───────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── CommsManager ─────────────────────────────────────────────

    #[test]
    fn test_comms_manager_new() {
        let manager = CommsManager::new();
        assert!(!manager.is_connected());
    }

    #[test]
    fn test_comms_connect_disconnect() {
        let mut manager = CommsManager::new();
        manager.connect(CommChannel::Cellular);
        assert!(manager.is_connected());
        manager.disconnect();
        assert!(!manager.is_connected());
    }

    // ── ROS2 Bridge ──────────────────────────────────────────────

    #[test]
    fn test_ros2_bridge_new() {
        let bridge = Ros2Bridge::new("test_node");
        assert_eq!(bridge.node_name(), "test_node");
        assert!(bridge.latest_arm_status().is_none());
        assert!(bridge.latest_grasp_status().is_none());
        assert!(bridge.latest_odometry().is_none());
    }

    #[test]
    fn test_ros2_bridge_publish_arm_status() {
        let mut bridge = Ros2Bridge::new("test_node");
        let status = ArmStatus {
            header: Header::default(),
            positions: [0.1, 0.2, 0.3, 0.4, 0.5],
            currents: [0.0; 5],
            gripper_holding: true,
        };
        bridge.publish_arm_status(&status);
        let published = bridge.latest_arm_status().unwrap();
        assert_eq!(published.positions[0], 0.1);
        assert!(published.gripper_holding);
    }

    #[test]
    fn test_ros2_bridge_publish_grasp_status() {
        let mut bridge = Ros2Bridge::new("test_node");
        let status = GraspStatus {
            header: Header::default(),
            state: GraspState::Holding,
            confidence: 0.95,
            object_label: "plastic_bottle".into(),
        };
        bridge.publish_grasp_status(&status);
        let published = bridge.latest_grasp_status().unwrap();
        assert_eq!(published.state, GraspState::Holding);
        assert_eq!(published.confidence, 0.95);
    }

    #[test]
    fn test_ros2_bridge_publish_odometry() {
        let mut bridge = Ros2Bridge::new("test_node");
        let odom = Odometry {
            header: Header {
                stamp: Timestamp { sec: 1, nanosec: 0 },
                frame_id: "odom".into(),
            },
            child_frame_id: "base_link".into(),
            pose: Pose::default(),
            twist: Twist::default(),
        };
        bridge.publish_odometry(&odom);
        let published = bridge.latest_odometry().unwrap();
        assert_eq!(published.header.frame_id, "odom");
        assert_eq!(published.child_frame_id, "base_link");
    }

    #[test]
    fn test_ros2_bridge_set_callbacks() {
        let mut bridge = Ros2Bridge::new("test_node");
        bridge.set_arm_command_callback(|_cmd| {});
        bridge.set_coverage_path_callback(|_path| {});
        assert!(bridge.arm_command_cb.is_some());
        assert!(bridge.coverage_path_cb.is_some());
    }

    // ── Message Types ────────────────────────────────────────────

    #[test]
    fn test_arm_joint_command_default() {
        let cmd = ArmJointCommand::default();
        assert_eq!(cmd.positions, [0.0; 5]);
        assert_eq!(cmd.velocities, [0.0; 5]);
    }

    #[test]
    fn test_grasp_state_default() {
        let state = GraspState::default();
        assert_eq!(state, GraspState::Idle);
    }

    #[test]
    fn test_quaternion_default_is_identity() {
        let q = Quaternion::default();
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
    }

    #[test]
    fn test_coverage_path_default() {
        let path = CoveragePath::default();
        assert!(path.waypoints.is_empty());
        assert!(!path.loop_path);
    }

    #[test]
    fn test_spin_once_does_not_panic() {
        let bridge = Ros2Bridge::new("test_node");
        bridge.spin_once();
    }
}
