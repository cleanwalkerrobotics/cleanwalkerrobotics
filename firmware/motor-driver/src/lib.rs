// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

//! CleanWalker CW-1 Motor Driver Library
//!
//! Provides motor control abstractions for the 18-DOF CleanWalker robot:
//! - 12 leg joints (4 legs x 3 DOF each: hip_yaw, hip_pitch, knee_pitch)
//! - 5 arm joints (turret_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, gripper)
//! - 1 bag frame hinge
//!
//! # Implementations
//! - [`PwmServoDriver`] -- PWM servo control for development/prototype hardware
//! - [`CanBusDriver`] -- CAN bus motor control for production hardware
//!
//! All implementations enforce joint safety limits derived from the CW-1 URDF.

use core::fmt;

// ─── Joint Identification ────────────────────────────────────────

/// Identifies which leg of the robot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Leg {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

impl fmt::Display for Leg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Leg::FrontLeft => write!(f, "fl"),
            Leg::FrontRight => write!(f, "fr"),
            Leg::RearLeft => write!(f, "rl"),
            Leg::RearRight => write!(f, "rr"),
        }
    }
}

/// Identifies a specific actuated joint on the CW-1 robot (18 total).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum JointId {
    /// Hip yaw joint (Z-axis rotation, +/-28.6 deg).
    HipYaw(Leg),
    /// Hip pitch joint (Y-axis rotation, +/-90 deg).
    HipPitch(Leg),
    /// Knee pitch joint (Y-axis rotation, range depends on front/rear).
    KneePitch(Leg),
    /// Arm turret yaw (Z-axis, +/-180 deg).
    ArmTurretYaw,
    /// Arm shoulder pitch (Y-axis, -45 to 180 deg).
    ArmShoulderPitch,
    /// Arm elbow pitch (Y-axis, 0 to 150 deg).
    ArmElbowPitch,
    /// Arm wrist pitch (Y-axis, +/-90 deg).
    ArmWristPitch,
    /// Arm gripper (Y-axis, 0 to 60 deg).
    ArmGripper,
    /// Bag frame hinge (Y-axis, 0 to 135 deg).
    BagFrameHinge,
}

impl fmt::Display for JointId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JointId::HipYaw(leg) => write!(f, "{}_hip_yaw", leg),
            JointId::HipPitch(leg) => write!(f, "{}_hip_pitch", leg),
            JointId::KneePitch(leg) => write!(f, "{}_knee_pitch", leg),
            JointId::ArmTurretYaw => write!(f, "arm_turret_yaw"),
            JointId::ArmShoulderPitch => write!(f, "arm_shoulder_pitch"),
            JointId::ArmElbowPitch => write!(f, "arm_elbow_pitch"),
            JointId::ArmWristPitch => write!(f, "arm_wrist_pitch"),
            JointId::ArmGripper => write!(f, "arm_gripper"),
            JointId::BagFrameHinge => write!(f, "bag_frame_hinge"),
        }
    }
}

// ─── Joint Configuration ─────────────────────────────────────────

/// Configuration for a single joint, derived from the CW-1 URDF specification.
#[derive(Debug, Clone, Copy)]
pub struct JointConfig {
    /// Joint identifier.
    pub id: JointId,
    /// Minimum position in radians.
    pub position_min: f64,
    /// Maximum position in radians.
    pub position_max: f64,
    /// Maximum torque in N*m.
    pub max_effort: f64,
    /// Maximum angular velocity in rad/s.
    pub max_velocity: f64,
}

impl JointConfig {
    /// Returns `true` if the given position is within the joint's URDF limits.
    pub fn is_position_valid(&self, position: f64) -> bool {
        position >= self.position_min && position <= self.position_max
    }

    /// Clamps a position to the joint's URDF limits.
    pub fn clamp_position(&self, position: f64) -> f64 {
        position.clamp(self.position_min, self.position_max)
    }

    /// Clamps a velocity magnitude to the joint's max velocity.
    pub fn clamp_velocity(&self, velocity: f64) -> f64 {
        velocity.clamp(-self.max_velocity, self.max_velocity)
    }
}

// ─── URDF Joint Configurations ───────────────────────────────────

/// All 18 joint configurations matching the CW-1 URDF (v1.0).
///
/// Joint limits, effort, and velocity values are taken directly from
/// `hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.urdf`.
///
/// Note: Front and rear knee joints have different ranges due to the
/// mammalian stance (front knees forward, rear knees backward).
pub const JOINT_CONFIGS: [JointConfig; 18] = [
    // Front-Left leg
    JointConfig { id: JointId::HipYaw(Leg::FrontLeft),    position_min: -0.4993,  position_max: 0.4993,  max_effort: 30.0, max_velocity: 10.0 },
    JointConfig { id: JointId::HipPitch(Leg::FrontLeft),   position_min: -1.5708,  position_max: 1.5708,  max_effort: 30.0, max_velocity: 10.0 },
    JointConfig { id: JointId::KneePitch(Leg::FrontLeft),  position_min:  0.0,     position_max: 2.6005,  max_effort: 30.0, max_velocity: 10.0 },
    // Front-Right leg
    JointConfig { id: JointId::HipYaw(Leg::FrontRight),   position_min: -0.4993,  position_max: 0.4993,  max_effort: 30.0, max_velocity: 10.0 },
    JointConfig { id: JointId::HipPitch(Leg::FrontRight),  position_min: -1.5708,  position_max: 1.5708,  max_effort: 30.0, max_velocity: 10.0 },
    JointConfig { id: JointId::KneePitch(Leg::FrontRight), position_min:  0.0,     position_max: 2.6005,  max_effort: 30.0, max_velocity: 10.0 },
    // Rear-Left leg
    JointConfig { id: JointId::HipYaw(Leg::RearLeft),     position_min: -0.4993,  position_max: 0.4993,  max_effort: 30.0, max_velocity: 10.0 },
    JointConfig { id: JointId::HipPitch(Leg::RearLeft),    position_min: -1.5708,  position_max: 1.5708,  max_effort: 30.0, max_velocity: 10.0 },
    JointConfig { id: JointId::KneePitch(Leg::RearLeft),   position_min: -2.6005,  position_max: 0.0995,  max_effort: 30.0, max_velocity: 10.0 },
    // Rear-Right leg
    JointConfig { id: JointId::HipYaw(Leg::RearRight),    position_min: -0.4993,  position_max: 0.4993,  max_effort: 30.0, max_velocity: 10.0 },
    JointConfig { id: JointId::HipPitch(Leg::RearRight),   position_min: -1.5708,  position_max: 1.5708,  max_effort: 30.0, max_velocity: 10.0 },
    JointConfig { id: JointId::KneePitch(Leg::RearRight),  position_min: -2.6005,  position_max: 0.0995,  max_effort: 30.0, max_velocity: 10.0 },
    // Arm joints
    JointConfig { id: JointId::ArmTurretYaw,     position_min: -3.14159, position_max: 3.14159, max_effort: 15.0, max_velocity: 5.0 },
    JointConfig { id: JointId::ArmShoulderPitch,  position_min: -0.7854,  position_max: 3.14159, max_effort: 15.0, max_velocity: 5.0 },
    JointConfig { id: JointId::ArmElbowPitch,     position_min:  0.0,     position_max: 2.6180,  max_effort: 15.0, max_velocity: 5.0 },
    JointConfig { id: JointId::ArmWristPitch,     position_min: -1.5708,  position_max: 1.5708,  max_effort: 10.0, max_velocity: 5.0 },
    JointConfig { id: JointId::ArmGripper,        position_min:  0.0,     position_max: 1.0472,  max_effort:  5.0, max_velocity: 2.0 },
    // Bag system
    JointConfig { id: JointId::BagFrameHinge,     position_min:  0.0,     position_max: 2.3562,  max_effort: 10.0, max_velocity: 2.0 },
];

/// Look up the configuration for a specific joint by ID.
pub fn get_joint_config(id: JointId) -> Option<&'static JointConfig> {
    JOINT_CONFIGS.iter().find(|c| c.id == id)
}

// ─── Motor Error ─────────────────────────────────────────────────

/// Errors returned by motor driver operations.
#[derive(Debug, Clone)]
pub enum MotorError {
    /// Commanded position exceeds URDF joint limits.
    PositionOutOfRange {
        joint: JointId,
        commanded: f64,
        min: f64,
        max: f64,
    },
    /// Commanded velocity exceeds joint maximum.
    VelocityOutOfRange {
        joint: JointId,
        commanded: f64,
        max: f64,
    },
    /// Communication with the motor controller failed.
    CommunicationError(String),
    /// Motor reported a hardware fault.
    HardwareFault(String),
    /// Joint ID not recognized or not configured.
    JointNotFound(JointId),
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MotorError::PositionOutOfRange { joint, commanded, min, max } => {
                write!(f, "{}: position {:.4} rad out of range [{:.4}, {:.4}]", joint, commanded, min, max)
            }
            MotorError::VelocityOutOfRange { joint, commanded, max } => {
                write!(f, "{}: velocity {:.4} rad/s exceeds max {:.4}", joint, commanded, max)
            }
            MotorError::CommunicationError(msg) => write!(f, "communication error: {}", msg),
            MotorError::HardwareFault(msg) => write!(f, "hardware fault: {}", msg),
            MotorError::JointNotFound(joint) => write!(f, "joint not found: {}", joint),
        }
    }
}

// ─── Motor Driver Trait ──────────────────────────────────────────

/// Trait for motor control implementations.
///
/// All implementations **must** enforce joint safety limits from the URDF.
/// Attempting to command a position or velocity outside limits returns an error.
pub trait MotorDriver {
    /// Command a joint to a target position in radians.
    ///
    /// Returns [`MotorError::PositionOutOfRange`] if the position exceeds
    /// the joint's URDF limits.
    fn set_position(&mut self, joint: JointId, position: f64) -> Result<(), MotorError>;

    /// Command a joint to a target velocity in rad/s.
    ///
    /// Returns [`MotorError::VelocityOutOfRange`] if the velocity exceeds
    /// the joint's URDF maximum.
    fn set_velocity(&mut self, joint: JointId, velocity: f64) -> Result<(), MotorError>;

    /// Read the current position of a joint in radians.
    fn get_position(&self, joint: JointId) -> Result<f64, MotorError>;

    /// Read the current draw of a joint's motor in amps.
    fn get_current(&self, joint: JointId) -> Result<f64, MotorError>;

    /// Emergency stop: immediately halt all motors and reject further commands.
    fn emergency_stop(&mut self) -> Result<(), MotorError>;
}

// ─── PWM Servo Driver ────────────────────────────────────────────

/// PWM servo driver for development and prototype hardware.
///
/// Uses standard PWM signals (50 Hz, 500-2500 us pulse width) to control
/// hobby/industrial servo motors via a PCA9685 I2C PWM driver board.
///
/// Suitable for bench testing and early prototyping before transitioning
/// to CAN bus motors for production.
///
/// # Hardware Setup
/// - PCA9685 16-channel PWM driver (two boards daisy-chained for 18 channels)
/// - Each joint maps to a PWM channel (0-17)
/// - Servo power: 6-7.4 V, separate from logic supply
/// - I2C address: 0x40 (board 1), 0x41 (board 2)
pub struct PwmServoDriver {
    /// Current commanded position for each joint (radians).
    positions: [(JointId, f64); 18],
    /// Whether emergency stop is active.
    estop_active: bool,
}

impl PwmServoDriver {
    /// Create a new PWM servo driver with all joints at their zero position.
    pub fn new() -> Self {
        let mut positions = [(JointId::BagFrameHinge, 0.0); 18];
        for (i, config) in JOINT_CONFIGS.iter().enumerate() {
            positions[i] = (config.id, 0.0);
        }
        Self {
            positions,
            estop_active: false,
        }
    }

    fn find_joint_index(&self, joint: JointId) -> Option<usize> {
        self.positions.iter().position(|(id, _)| *id == joint)
    }
}

impl Default for PwmServoDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorDriver for PwmServoDriver {
    fn set_position(&mut self, joint: JointId, position: f64) -> Result<(), MotorError> {
        if self.estop_active {
            return Err(MotorError::HardwareFault("emergency stop active".into()));
        }
        let config = get_joint_config(joint).ok_or(MotorError::JointNotFound(joint))?;
        if !config.is_position_valid(position) {
            return Err(MotorError::PositionOutOfRange {
                joint,
                commanded: position,
                min: config.position_min,
                max: config.position_max,
            });
        }
        let idx = self.find_joint_index(joint).ok_or(MotorError::JointNotFound(joint))?;
        self.positions[idx].1 = position;
        // TODO: Convert radians to PWM pulse width and write via I2C to PCA9685
        Ok(())
    }

    fn set_velocity(&mut self, joint: JointId, velocity: f64) -> Result<(), MotorError> {
        if self.estop_active {
            return Err(MotorError::HardwareFault("emergency stop active".into()));
        }
        let config = get_joint_config(joint).ok_or(MotorError::JointNotFound(joint))?;
        if velocity.abs() > config.max_velocity {
            return Err(MotorError::VelocityOutOfRange {
                joint,
                commanded: velocity,
                max: config.max_velocity,
            });
        }
        // TODO: Convert velocity to PWM ramp rate on PCA9685
        Ok(())
    }

    fn get_position(&self, joint: JointId) -> Result<f64, MotorError> {
        let idx = self.find_joint_index(joint).ok_or(MotorError::JointNotFound(joint))?;
        Ok(self.positions[idx].1)
    }

    fn get_current(&self, joint: JointId) -> Result<f64, MotorError> {
        let _ = self.find_joint_index(joint).ok_or(MotorError::JointNotFound(joint))?;
        // TODO: Read current from ADC on servo power rail
        Ok(0.0)
    }

    fn emergency_stop(&mut self) -> Result<(), MotorError> {
        self.estop_active = true;
        for pos in &mut self.positions {
            pos.1 = 0.0;
        }
        // TODO: Disable PCA9685 output enable pin (active low)
        Ok(())
    }
}

// ─── CAN Bus Driver ──────────────────────────────────────────────

/// CAN bus motor driver for production hardware.
///
/// Communicates with brushless DC servo motors via CAN 2.0B at 1 Mbit/s.
/// Each motor has a unique CAN ID and supports position, velocity, and
/// torque control modes with real-time encoder and current feedback.
///
/// # Protocol
/// - Bus speed: 1 Mbit/s
/// - Frame format: Standard 11-bit ID, 8-byte data
/// - Motor CAN IDs: 0x01-0x12 (joints 1-18)
/// - Command frame: [mode, pos_hi, pos_lo, vel_hi, vel_lo, torque_hi, torque_lo, crc]
/// - Feedback rate: 1 kHz position/velocity, 100 Hz current
///
/// # Hardware
/// - CAN transceiver: MCP2551 or built-in STM32 CAN peripheral
/// - 120 ohm termination resistors at both ends of bus
/// - 48 V power bus for motors, separate CAN logic supply
pub struct CanBusDriver {
    /// Current position for each joint from encoder feedback (radians).
    positions: [(JointId, f64); 18],
    /// Whether emergency stop is active.
    estop_active: bool,
    /// CAN bus interface name (e.g., "can0").
    _bus_name: String,
}

impl CanBusDriver {
    /// Create a new CAN bus driver bound to the given interface.
    ///
    /// # Arguments
    /// * `bus_name` - CAN interface name (e.g., `"can0"`)
    pub fn new(bus_name: &str) -> Self {
        let mut positions = [(JointId::BagFrameHinge, 0.0); 18];
        for (i, config) in JOINT_CONFIGS.iter().enumerate() {
            positions[i] = (config.id, 0.0);
        }
        Self {
            positions,
            estop_active: false,
            _bus_name: bus_name.to_string(),
        }
    }

    fn find_joint_index(&self, joint: JointId) -> Option<usize> {
        self.positions.iter().position(|(id, _)| *id == joint)
    }
}

impl MotorDriver for CanBusDriver {
    fn set_position(&mut self, joint: JointId, position: f64) -> Result<(), MotorError> {
        if self.estop_active {
            return Err(MotorError::HardwareFault("emergency stop active".into()));
        }
        let config = get_joint_config(joint).ok_or(MotorError::JointNotFound(joint))?;
        if !config.is_position_valid(position) {
            return Err(MotorError::PositionOutOfRange {
                joint,
                commanded: position,
                min: config.position_min,
                max: config.position_max,
            });
        }
        let idx = self.find_joint_index(joint).ok_or(MotorError::JointNotFound(joint))?;
        self.positions[idx].1 = position;
        // TODO: Send CAN position command frame to motor controller
        Ok(())
    }

    fn set_velocity(&mut self, joint: JointId, velocity: f64) -> Result<(), MotorError> {
        if self.estop_active {
            return Err(MotorError::HardwareFault("emergency stop active".into()));
        }
        let config = get_joint_config(joint).ok_or(MotorError::JointNotFound(joint))?;
        if velocity.abs() > config.max_velocity {
            return Err(MotorError::VelocityOutOfRange {
                joint,
                commanded: velocity,
                max: config.max_velocity,
            });
        }
        // TODO: Send CAN velocity command frame to motor controller
        Ok(())
    }

    fn get_position(&self, joint: JointId) -> Result<f64, MotorError> {
        let idx = self.find_joint_index(joint).ok_or(MotorError::JointNotFound(joint))?;
        // TODO: Return latest position from CAN feedback ring buffer
        Ok(self.positions[idx].1)
    }

    fn get_current(&self, joint: JointId) -> Result<f64, MotorError> {
        let _ = self.find_joint_index(joint).ok_or(MotorError::JointNotFound(joint))?;
        // TODO: Return latest current from CAN feedback ring buffer
        Ok(0.0)
    }

    fn emergency_stop(&mut self) -> Result<(), MotorError> {
        self.estop_active = true;
        for pos in &mut self.positions {
            pos.1 = 0.0;
        }
        // TODO: Send broadcast CAN emergency stop frame (ID 0x000, highest priority)
        Ok(())
    }
}

// ─── Tests ───────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joint_config_count() {
        assert_eq!(JOINT_CONFIGS.len(), 18);
    }

    #[test]
    fn test_all_joints_have_configs() {
        for leg in [Leg::FrontLeft, Leg::FrontRight, Leg::RearLeft, Leg::RearRight] {
            assert!(get_joint_config(JointId::HipYaw(leg)).is_some());
            assert!(get_joint_config(JointId::HipPitch(leg)).is_some());
            assert!(get_joint_config(JointId::KneePitch(leg)).is_some());
        }
        assert!(get_joint_config(JointId::ArmTurretYaw).is_some());
        assert!(get_joint_config(JointId::ArmShoulderPitch).is_some());
        assert!(get_joint_config(JointId::ArmElbowPitch).is_some());
        assert!(get_joint_config(JointId::ArmWristPitch).is_some());
        assert!(get_joint_config(JointId::ArmGripper).is_some());
        assert!(get_joint_config(JointId::BagFrameHinge).is_some());
    }

    #[test]
    fn test_joint_config_lookup() {
        let config = get_joint_config(JointId::HipYaw(Leg::FrontLeft)).unwrap();
        assert_eq!(config.max_effort, 30.0);
        assert_eq!(config.max_velocity, 10.0);
    }

    #[test]
    fn test_position_clamping() {
        let config = get_joint_config(JointId::ArmGripper).unwrap();
        assert_eq!(config.clamp_position(-1.0), 0.0);
        assert_eq!(config.clamp_position(5.0), 1.0472);
        assert_eq!(config.clamp_position(0.5), 0.5);
    }

    #[test]
    fn test_position_validation() {
        let config = get_joint_config(JointId::ArmGripper).unwrap();
        assert!(config.is_position_valid(0.5));
        assert!(!config.is_position_valid(-0.1));
        assert!(!config.is_position_valid(2.0));
    }

    #[test]
    fn test_rear_knee_reversed_limits() {
        let rear = get_joint_config(JointId::KneePitch(Leg::RearLeft)).unwrap();
        assert!(rear.position_min < 0.0, "rear knee min should be negative");
        assert!(rear.position_max < 0.2, "rear knee max should be near zero");

        let front = get_joint_config(JointId::KneePitch(Leg::FrontLeft)).unwrap();
        assert_eq!(front.position_min, 0.0, "front knee min should be zero");
        assert!(front.position_max > 2.0, "front knee max should be >2 rad");
    }

    #[test]
    fn test_velocity_clamping() {
        let config = get_joint_config(JointId::ArmGripper).unwrap();
        assert_eq!(config.clamp_velocity(10.0), 2.0);
        assert_eq!(config.clamp_velocity(-10.0), -2.0);
        assert_eq!(config.clamp_velocity(1.0), 1.0);
    }

    // ── PWM Servo Driver ─────────────────────────────────────────

    #[test]
    fn test_pwm_driver_new() {
        let driver = PwmServoDriver::new();
        assert!(!driver.estop_active);
    }

    #[test]
    fn test_pwm_driver_set_get_position() {
        let mut driver = PwmServoDriver::new();
        let joint = JointId::ArmElbowPitch;
        driver.set_position(joint, 1.0).unwrap();
        assert_eq!(driver.get_position(joint).unwrap(), 1.0);
    }

    #[test]
    fn test_pwm_driver_position_out_of_range() {
        let mut driver = PwmServoDriver::new();
        let result = driver.set_position(JointId::ArmGripper, 2.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_pwm_driver_velocity_out_of_range() {
        let mut driver = PwmServoDriver::new();
        let result = driver.set_velocity(JointId::ArmGripper, 5.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_pwm_driver_estop_blocks_commands() {
        let mut driver = PwmServoDriver::new();
        driver.set_position(JointId::ArmElbowPitch, 1.0).unwrap();
        driver.emergency_stop().unwrap();
        assert!(driver.estop_active);
        assert!(driver.set_position(JointId::ArmElbowPitch, 0.5).is_err());
        assert!(driver.set_velocity(JointId::ArmElbowPitch, 0.5).is_err());
    }

    #[test]
    fn test_pwm_driver_get_current_stub() {
        let driver = PwmServoDriver::new();
        assert_eq!(driver.get_current(JointId::ArmGripper).unwrap(), 0.0);
    }

    // ── CAN Bus Driver ───────────────────────────────────────────

    #[test]
    fn test_can_driver_new() {
        let driver = CanBusDriver::new("can0");
        assert!(!driver.estop_active);
    }

    #[test]
    fn test_can_driver_set_get_position() {
        let mut driver = CanBusDriver::new("can0");
        let joint = JointId::HipPitch(Leg::FrontLeft);
        driver.set_position(joint, 0.5).unwrap();
        assert_eq!(driver.get_position(joint).unwrap(), 0.5);
    }

    #[test]
    fn test_can_driver_estop() {
        let mut driver = CanBusDriver::new("can0");
        driver.emergency_stop().unwrap();
        assert!(driver.set_position(JointId::HipYaw(Leg::FrontLeft), 0.1).is_err());
    }

    // ── Display ──────────────────────────────────────────────────

    #[test]
    fn test_joint_id_display() {
        assert_eq!(format!("{}", JointId::HipYaw(Leg::FrontLeft)), "fl_hip_yaw");
        assert_eq!(format!("{}", JointId::ArmGripper), "arm_gripper");
        assert_eq!(format!("{}", JointId::BagFrameHinge), "bag_frame_hinge");
    }

    #[test]
    fn test_motor_error_display() {
        let err = MotorError::PositionOutOfRange {
            joint: JointId::ArmGripper,
            commanded: 2.0,
            min: 0.0,
            max: 1.0472,
        };
        let msg = format!("{}", err);
        assert!(msg.contains("arm_gripper"));
        assert!(msg.contains("2.0000"));
    }
}
