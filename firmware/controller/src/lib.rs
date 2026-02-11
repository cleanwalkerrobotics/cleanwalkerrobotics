// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

//! CleanWalker CW-1 Robot Controller
//!
//! Coordinates locomotion, manipulation, and bag management for the
//! CleanWalker CW-1 autonomous litter collection robot.
//!
//! # Architecture
//! - [`GaitController`] -- Coordinates 12 leg joints for walking gaits
//! - [`ArmController`] -- Controls the 5-DOF manipulation arm
//! - [`RobotStateMachine`] -- Top-level state management
//! - [`EmergencyStop`] -- Safety system for immediate halt

use cw_motor_driver::{JointId, Leg, MotorDriver, MotorError};

// ─── Standing Pose ───────────────────────────────────────────────

/// Default standing joint positions (radians) derived from URDF neutral pose.
///
/// All hip yaw joints at 0 (straight ahead), hip pitch at slight forward
/// lean, knees bent to achieve approximately 35 cm ground clearance.
pub const STANDING_POSE: [(JointId, f64); 12] = [
    // Front-Left
    (JointId::HipYaw(Leg::FrontLeft), 0.0),
    (JointId::HipPitch(Leg::FrontLeft), 0.3),
    (JointId::KneePitch(Leg::FrontLeft), 0.8),
    // Front-Right
    (JointId::HipYaw(Leg::FrontRight), 0.0),
    (JointId::HipPitch(Leg::FrontRight), 0.3),
    (JointId::KneePitch(Leg::FrontRight), 0.8),
    // Rear-Left (reversed pitch/knee for mammalian stance)
    (JointId::HipYaw(Leg::RearLeft), 0.0),
    (JointId::HipPitch(Leg::RearLeft), -0.3),
    (JointId::KneePitch(Leg::RearLeft), -0.8),
    // Rear-Right
    (JointId::HipYaw(Leg::RearRight), 0.0),
    (JointId::HipPitch(Leg::RearRight), -0.3),
    (JointId::KneePitch(Leg::RearRight), -0.8),
];

/// Default arm stowed position (tucked close to body).
pub const ARM_STOWED: [(JointId, f64); 5] = [
    (JointId::ArmTurretYaw, 0.0),
    (JointId::ArmShoulderPitch, 0.0),
    (JointId::ArmElbowPitch, 0.0),
    (JointId::ArmWristPitch, 0.0),
    (JointId::ArmGripper, 0.0),
];

/// Arm reaching forward and slightly down (litter pickup pose).
pub const ARM_REACH_FORWARD: [(JointId, f64); 5] = [
    (JointId::ArmTurretYaw, 0.0),
    (JointId::ArmShoulderPitch, 1.2),   // forward
    (JointId::ArmElbowPitch, 0.8),      // partial bend
    (JointId::ArmWristPitch, -0.3),     // angled down
    (JointId::ArmGripper, 0.8),         // open
];

// ─── Robot State Machine ─────────────────────────────────────────

/// Top-level robot operating state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    /// Robot is powered on, standing still, awaiting commands.
    Idle,
    /// Robot is actively walking (executing a gait pattern).
    Walking,
    /// Robot is decelerating to a stop.
    Stopping,
    /// Arm is reaching toward a target (litter pickup).
    Reaching,
    /// Arm gripper is closing on a target.
    Grasping,
    /// Emergency stop activated -- all motors halted.
    EmergencyStopped,
}

/// State machine governing the robot's top-level behavior.
///
/// Valid transitions:
/// ```text
/// Idle -> Walking -> Stopping -> Idle
/// Idle -> Reaching -> Grasping -> Idle
/// Reaching -> Idle (abort)
/// (any) -> EmergencyStopped -> Idle (recovery)
/// ```
pub struct RobotStateMachine {
    state: RobotState,
}

impl RobotStateMachine {
    /// Create a new state machine in the [`RobotState::Idle`] state.
    pub fn new() -> Self {
        Self { state: RobotState::Idle }
    }

    /// Returns the current state.
    pub fn state(&self) -> RobotState {
        self.state
    }

    /// Attempt a state transition. Returns `Ok(())` if the transition is
    /// valid, or `Err` with a description if not.
    pub fn transition(&mut self, target: RobotState) -> Result<(), &'static str> {
        let valid = match (self.state, target) {
            // From Idle
            (RobotState::Idle, RobotState::Walking) => true,
            (RobotState::Idle, RobotState::Reaching) => true,
            // From Walking
            (RobotState::Walking, RobotState::Stopping) => true,
            // From Stopping
            (RobotState::Stopping, RobotState::Idle) => true,
            // From Reaching
            (RobotState::Reaching, RobotState::Grasping) => true,
            (RobotState::Reaching, RobotState::Idle) => true,
            // From Grasping
            (RobotState::Grasping, RobotState::Idle) => true,
            // Emergency stop from any state
            (_, RobotState::EmergencyStopped) => true,
            // Recovery from e-stop only to idle
            (RobotState::EmergencyStopped, RobotState::Idle) => true,
            _ => false,
        };
        if valid {
            self.state = target;
            Ok(())
        } else {
            Err("invalid state transition")
        }
    }
}

impl Default for RobotStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Gait Controller ─────────────────────────────────────────────

/// Phase of a single leg within a gait cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LegPhase {
    /// Leg is on the ground, pushing the body forward.
    Stance,
    /// Leg is in the air, swinging forward.
    Swing,
}

/// Gait pattern type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GaitType {
    /// Trot: diagonal leg pairs move together (FL+RR, FR+RL).
    Trot,
}

/// Coordinates 12 leg joints for walking gaits.
///
/// The CW-1 uses a trot gait where diagonal leg pairs move in sync:
/// - Phase A: Front-Left + Rear-Right in stance, Front-Right + Rear-Left in swing
/// - Phase B: Roles swap at the half-cycle mark
pub struct GaitController {
    /// Current gait type.
    gait_type: GaitType,
    /// Gait cycle phase (0.0 to 1.0).
    phase: f64,
    /// Gait cycle period in seconds.
    cycle_period: f64,
    /// Step height in radians (knee lift during swing).
    step_height: f64,
    /// Step length in radians (hip sweep during stride).
    step_length: f64,
    /// Whether the gait is currently active.
    active: bool,
}

impl GaitController {
    /// Create a new gait controller with default trot parameters.
    pub fn new() -> Self {
        Self {
            gait_type: GaitType::Trot,
            phase: 0.0,
            cycle_period: 0.5,
            step_height: 0.4,
            step_length: 0.3,
            active: false,
        }
    }

    /// Start the gait cycle.
    pub fn start(&mut self) {
        self.active = true;
        self.phase = 0.0;
    }

    /// Stop the gait cycle (legs return to standing pose).
    pub fn stop(&mut self) {
        self.active = false;
    }

    /// Whether the gait is currently running.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Current gait phase (0.0 to 1.0).
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Advance the gait by `dt` seconds and compute joint targets.
    ///
    /// Returns the 12 leg joint target positions for the current phase.
    /// When inactive, returns [`STANDING_POSE`].
    pub fn update(&mut self, dt: f64) -> [(JointId, f64); 12] {
        if !self.active {
            return STANDING_POSE;
        }
        self.phase = (self.phase + dt / self.cycle_period) % 1.0;

        match self.gait_type {
            GaitType::Trot => self.compute_trot(),
        }
    }

    /// Compute joint targets for trot gait at current phase.
    ///
    /// Phase 0.0-0.5: pair A (FL+RR) in stance, pair B (FR+RL) in swing.
    /// Phase 0.5-1.0: pair B in stance, pair A in swing.
    fn compute_trot(&self) -> [(JointId, f64); 12] {
        let (phase_a, phase_b) = if self.phase < 0.5 {
            (LegPhase::Stance, LegPhase::Swing)
        } else {
            (LegPhase::Swing, LegPhase::Stance)
        };

        let mut targets = STANDING_POSE;

        // Pair A: Front-Left + Rear-Right
        self.apply_leg_phase(&mut targets, Leg::FrontLeft, phase_a, true);
        self.apply_leg_phase(&mut targets, Leg::RearRight, phase_a, false);

        // Pair B: Front-Right + Rear-Left
        self.apply_leg_phase(&mut targets, Leg::FrontRight, phase_b, true);
        self.apply_leg_phase(&mut targets, Leg::RearLeft, phase_b, false);

        targets
    }

    /// Apply a leg phase (stance or swing) to the target positions array.
    fn apply_leg_phase(
        &self,
        targets: &mut [(JointId, f64); 12],
        leg: Leg,
        phase: LegPhase,
        is_front: bool,
    ) {
        // Sub-phase within the current half-cycle (0.0 to 1.0)
        let sub_phase = if self.phase < 0.5 {
            self.phase * 2.0
        } else {
            (self.phase - 0.5) * 2.0
        };

        // Index of the hip_pitch joint for this leg in the targets array
        let hip_pitch_idx = match leg {
            Leg::FrontLeft => 1,
            Leg::FrontRight => 4,
            Leg::RearLeft => 7,
            Leg::RearRight => 10,
        };
        let knee_idx = hip_pitch_idx + 1;

        // Sign flip for rear legs (reversed joint directions)
        let dir = if is_front { 1.0 } else { -1.0 };

        match phase {
            LegPhase::Stance => {
                // Push body forward: hip sweeps from front to back
                let hip_delta = self.step_length * (0.5 - sub_phase);
                targets[hip_pitch_idx].1 += hip_delta * dir;
            }
            LegPhase::Swing => {
                // Lift and swing forward: hip sweeps back to front, knee lifts
                let hip_delta = self.step_length * (sub_phase - 0.5);
                targets[hip_pitch_idx].1 += hip_delta * dir;
                // Parabolic knee lift during swing
                let lift = 4.0 * self.step_height * sub_phase * (1.0 - sub_phase);
                targets[knee_idx].1 += lift * dir;
            }
        }
    }

    /// Send computed joint targets to the motor driver.
    pub fn apply_to_driver(
        &self,
        targets: &[(JointId, f64); 12],
        driver: &mut dyn MotorDriver,
    ) -> Result<(), MotorError> {
        for &(joint, position) in targets.iter() {
            driver.set_position(joint, position)?;
        }
        Ok(())
    }
}

impl Default for GaitController {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Arm Controller ──────────────────────────────────────────────

/// Predefined arm poses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArmPose {
    /// Arm tucked close to body.
    Stowed,
    /// Arm extended forward for litter pickup.
    ReachForward,
    /// Arm reaching back to compress bag contents.
    CompressBag,
    /// Custom joint positions [turret, shoulder, elbow, wrist, gripper].
    Custom([f64; 5]),
}

/// Controls the 5-DOF manipulation arm.
///
/// The arm is used for:
/// - Picking up litter (reach forward, grasp, retract)
/// - Compressing bag contents (reach back, press down)
/// - Stowing during locomotion
pub struct ArmController {
    /// Current target pose.
    target_pose: ArmPose,
    /// Current interpolated joint positions.
    current_positions: [f64; 5],
    /// Whether the gripper is commanded closed.
    gripper_closed: bool,
    /// Interpolation speed (fraction per second converged).
    interp_speed: f64,
}

impl ArmController {
    /// Create a new arm controller in the stowed position.
    pub fn new() -> Self {
        Self {
            target_pose: ArmPose::Stowed,
            current_positions: [0.0; 5],
            gripper_closed: false,
            interp_speed: 2.0,
        }
    }

    /// Set the target arm pose.
    pub fn set_pose(&mut self, pose: ArmPose) {
        self.target_pose = pose;
    }

    /// Current target pose.
    pub fn target_pose(&self) -> ArmPose {
        self.target_pose
    }

    /// Close the gripper.
    pub fn close_gripper(&mut self) {
        self.gripper_closed = true;
    }

    /// Open the gripper.
    pub fn open_gripper(&mut self) {
        self.gripper_closed = false;
    }

    /// Whether the gripper is commanded closed.
    pub fn is_gripper_closed(&self) -> bool {
        self.gripper_closed
    }

    /// Get the raw target joint positions for the current pose.
    fn target_positions(&self) -> [f64; 5] {
        match self.target_pose {
            ArmPose::Stowed => [
                ARM_STOWED[0].1,
                ARM_STOWED[1].1,
                ARM_STOWED[2].1,
                ARM_STOWED[3].1,
                ARM_STOWED[4].1,
            ],
            ArmPose::ReachForward => [
                ARM_REACH_FORWARD[0].1,
                ARM_REACH_FORWARD[1].1,
                ARM_REACH_FORWARD[2].1,
                ARM_REACH_FORWARD[3].1,
                ARM_REACH_FORWARD[4].1,
            ],
            ArmPose::CompressBag => [
                std::f64::consts::PI, // turret rotated to rear
                1.5,                   // shoulder forward (reaches over body to bag)
                1.8,                   // elbow bent
                -0.5,                  // wrist angled down for compression
                0.0,                   // gripper closed flat for pressing
            ],
            ArmPose::Custom(positions) => positions,
        }
    }

    /// Update arm interpolation by `dt` seconds and return joint commands.
    ///
    /// Returns the 5 arm joint target positions. Joint positions are
    /// smoothly interpolated toward the target pose.
    pub fn update(&mut self, dt: f64) -> [(JointId, f64); 5] {
        let targets = self.target_positions();
        let alpha = (self.interp_speed * dt).min(1.0);

        for i in 0..5 {
            self.current_positions[i] +=
                (targets[i] - self.current_positions[i]) * alpha;
        }

        // Override gripper position based on grip command
        let gripper_target = if self.gripper_closed { 0.0 } else { 0.8 };
        self.current_positions[4] +=
            (gripper_target - self.current_positions[4]) * alpha;

        [
            (JointId::ArmTurretYaw, self.current_positions[0]),
            (JointId::ArmShoulderPitch, self.current_positions[1]),
            (JointId::ArmElbowPitch, self.current_positions[2]),
            (JointId::ArmWristPitch, self.current_positions[3]),
            (JointId::ArmGripper, self.current_positions[4]),
        ]
    }

    /// Send computed joint targets to the motor driver.
    pub fn apply_to_driver(
        &self,
        targets: &[(JointId, f64); 5],
        driver: &mut dyn MotorDriver,
    ) -> Result<(), MotorError> {
        for &(joint, position) in targets.iter() {
            driver.set_position(joint, position)?;
        }
        Ok(())
    }
}

impl Default for ArmController {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Emergency Stop ──────────────────────────────────────────────

/// Emergency stop handler.
///
/// When triggered, immediately commands all motors to halt via the
/// [`MotorDriver::emergency_stop`] method and records the reason.
pub struct EmergencyStop {
    /// Whether E-stop is currently active.
    active: bool,
    /// Reason for the most recent E-stop activation.
    reason: Option<String>,
}

impl EmergencyStop {
    /// Create a new emergency stop handler (inactive).
    pub fn new() -> Self {
        Self {
            active: false,
            reason: None,
        }
    }

    /// Trigger emergency stop on the given motor driver.
    pub fn trigger(
        &mut self,
        reason: &str,
        driver: &mut dyn MotorDriver,
    ) -> Result<(), MotorError> {
        self.active = true;
        self.reason = Some(reason.to_string());
        driver.emergency_stop()
    }

    /// Reset emergency stop (requires explicit operator action).
    pub fn reset(&mut self) {
        self.active = false;
        self.reason = None;
    }

    /// Whether E-stop is currently active.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Reason for the current E-stop, if active.
    pub fn reason(&self) -> Option<&str> {
        self.reason.as_deref()
    }
}

impl Default for EmergencyStop {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Tests ───────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use cw_motor_driver::PwmServoDriver;

    // ── State Machine ────────────────────────────────────────────

    #[test]
    fn test_initial_state_is_idle() {
        let sm = RobotStateMachine::new();
        assert_eq!(sm.state(), RobotState::Idle);
    }

    #[test]
    fn test_valid_walk_cycle() {
        let mut sm = RobotStateMachine::new();
        assert!(sm.transition(RobotState::Walking).is_ok());
        assert!(sm.transition(RobotState::Stopping).is_ok());
        assert!(sm.transition(RobotState::Idle).is_ok());
    }

    #[test]
    fn test_valid_grasp_cycle() {
        let mut sm = RobotStateMachine::new();
        assert!(sm.transition(RobotState::Reaching).is_ok());
        assert!(sm.transition(RobotState::Grasping).is_ok());
        assert!(sm.transition(RobotState::Idle).is_ok());
    }

    #[test]
    fn test_invalid_transition() {
        let mut sm = RobotStateMachine::new();
        assert!(sm.transition(RobotState::Grasping).is_err());
        assert!(sm.transition(RobotState::Stopping).is_err());
    }

    #[test]
    fn test_estop_from_any_state() {
        let mut sm = RobotStateMachine::new();
        sm.transition(RobotState::Walking).unwrap();
        assert!(sm.transition(RobotState::EmergencyStopped).is_ok());
        assert_eq!(sm.state(), RobotState::EmergencyStopped);
    }

    #[test]
    fn test_estop_recovery_to_idle() {
        let mut sm = RobotStateMachine::new();
        sm.transition(RobotState::EmergencyStopped).unwrap();
        assert!(sm.transition(RobotState::Idle).is_ok());
        assert_eq!(sm.state(), RobotState::Idle);
    }

    #[test]
    fn test_estop_blocks_non_idle_recovery() {
        let mut sm = RobotStateMachine::new();
        sm.transition(RobotState::EmergencyStopped).unwrap();
        assert!(sm.transition(RobotState::Walking).is_err());
    }

    #[test]
    fn test_reaching_abort_to_idle() {
        let mut sm = RobotStateMachine::new();
        sm.transition(RobotState::Reaching).unwrap();
        assert!(sm.transition(RobotState::Idle).is_ok());
    }

    // ── Gait Controller ──────────────────────────────────────────

    #[test]
    fn test_gait_starts_inactive() {
        let gc = GaitController::new();
        assert!(!gc.is_active());
    }

    #[test]
    fn test_gait_standing_when_inactive() {
        let mut gc = GaitController::new();
        let targets = gc.update(0.01);
        assert_eq!(targets, STANDING_POSE);
    }

    #[test]
    fn test_gait_active_after_start() {
        let mut gc = GaitController::new();
        gc.start();
        assert!(gc.is_active());
    }

    #[test]
    fn test_gait_phase_advances() {
        let mut gc = GaitController::new();
        gc.start();
        gc.update(0.1);
        assert!(gc.phase() > 0.0);
    }

    #[test]
    fn test_gait_phase_wraps() {
        let mut gc = GaitController::new();
        gc.start();
        gc.update(0.6); // > cycle_period (0.5s)
        assert!(gc.phase() < 1.0);
    }

    #[test]
    fn test_gait_stop() {
        let mut gc = GaitController::new();
        gc.start();
        gc.stop();
        assert!(!gc.is_active());
    }

    #[test]
    fn test_gait_trot_modifies_positions() {
        let mut gc = GaitController::new();
        gc.start();
        let targets = gc.update(0.1);
        // At least some positions should differ from standing
        let differs = targets
            .iter()
            .zip(STANDING_POSE.iter())
            .any(|(a, b)| (a.1 - b.1).abs() > 1e-6);
        assert!(differs, "trot gait should modify joint positions");
    }

    #[test]
    fn test_gait_apply_to_driver() {
        let gc = GaitController::new();
        let mut driver = PwmServoDriver::new();
        let result = gc.apply_to_driver(&STANDING_POSE, &mut driver);
        assert!(result.is_ok());
    }

    // ── Arm Controller ───────────────────────────────────────────

    #[test]
    fn test_arm_starts_stowed() {
        let ac = ArmController::new();
        assert_eq!(ac.target_pose(), ArmPose::Stowed);
    }

    #[test]
    fn test_arm_gripper_toggle() {
        let mut ac = ArmController::new();
        assert!(!ac.is_gripper_closed());
        ac.close_gripper();
        assert!(ac.is_gripper_closed());
        ac.open_gripper();
        assert!(!ac.is_gripper_closed());
    }

    #[test]
    fn test_arm_pose_change() {
        let mut ac = ArmController::new();
        ac.set_pose(ArmPose::ReachForward);
        assert_eq!(ac.target_pose(), ArmPose::ReachForward);
    }

    #[test]
    fn test_arm_update_returns_correct_joint_ids() {
        let mut ac = ArmController::new();
        let targets = ac.update(0.1);
        assert_eq!(targets[0].0, JointId::ArmTurretYaw);
        assert_eq!(targets[1].0, JointId::ArmShoulderPitch);
        assert_eq!(targets[2].0, JointId::ArmElbowPitch);
        assert_eq!(targets[3].0, JointId::ArmWristPitch);
        assert_eq!(targets[4].0, JointId::ArmGripper);
    }

    #[test]
    fn test_arm_interpolation_moves_toward_target() {
        let mut ac = ArmController::new();
        ac.set_pose(ArmPose::ReachForward);
        let targets = ac.update(0.1);
        // Shoulder should have moved toward 1.2 (from 0.0)
        assert!(targets[1].1 > 0.0, "shoulder should interpolate toward target");
    }

    #[test]
    fn test_arm_apply_to_driver() {
        let mut ac = ArmController::new();
        let targets = ac.update(0.1);
        let mut driver = PwmServoDriver::new();
        let result = ac.apply_to_driver(&targets, &mut driver);
        assert!(result.is_ok());
    }

    // ── Emergency Stop ───────────────────────────────────────────

    #[test]
    fn test_estop_initially_inactive() {
        let es = EmergencyStop::new();
        assert!(!es.is_active());
        assert!(es.reason().is_none());
    }

    #[test]
    fn test_estop_trigger_and_reset() {
        let mut es = EmergencyStop::new();
        let mut driver = PwmServoDriver::new();
        es.trigger("overcurrent detected", &mut driver).unwrap();
        assert!(es.is_active());
        assert_eq!(es.reason(), Some("overcurrent detected"));
        es.reset();
        assert!(!es.is_active());
        assert!(es.reason().is_none());
    }
}
