// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

//! CleanWalker CW-1 main controller binary.
//!
//! Coordinates locomotion, manipulation, and communication for the
//! CW-1 autonomous litter collection robot.

use cw_controller::{
    ArmController, EmergencyStop, GaitController, RobotState, RobotStateMachine,
};
use cw_motor_driver::PwmServoDriver;
use cw_comms::Ros2Bridge;

fn main() {
    println!("CleanWalker CW-1 Controller v0.1.0");
    println!("Initializing subsystems...");

    // Initialize motor driver (PWM for development, CAN for production)
    let mut driver = PwmServoDriver::new();
    println!("  Motor driver: PWM servo (dev mode)");

    // Initialize controllers
    let mut state_machine = RobotStateMachine::new();
    let mut gait = GaitController::new();
    let mut arm = ArmController::new();
    let mut estop = EmergencyStop::new();
    println!("  Gait controller: ready");
    println!("  Arm controller: ready (stowed)");
    println!("  State machine: {:?}", state_machine.state());

    // Initialize ROS2 bridge
    let mut bridge = Ros2Bridge::new("cw1_firmware");
    bridge.set_arm_command_callback(|cmd| {
        println!("  RX /arm/joint_commands: {:?}", cmd.positions);
    });
    bridge.set_coverage_path_callback(|path| {
        println!("  RX /coverage/path: {} waypoints", path.waypoints.len());
    });
    println!("  ROS2 bridge: {} (stub)", bridge.node_name());

    println!("All subsystems initialized.");

    // Single control loop tick (demonstration)
    let dt = 0.02; // 50 Hz control loop

    // Compute leg targets (standing pose when gait inactive)
    let leg_targets = gait.update(dt);
    if let Err(e) = gait.apply_to_driver(&leg_targets, &mut driver) {
        eprintln!("Gait error: {}", e);
        let _ = estop.trigger("gait driver error", &mut driver);
        let _ = state_machine.transition(RobotState::EmergencyStopped);
    }

    // Compute arm targets (stowed)
    let arm_targets = arm.update(dt);
    if let Err(e) = arm.apply_to_driver(&arm_targets, &mut driver) {
        eprintln!("Arm error: {}", e);
        let _ = estop.trigger("arm driver error", &mut driver);
        let _ = state_machine.transition(RobotState::EmergencyStopped);
    }

    // Process ROS2 messages
    bridge.spin_once();

    // Publish arm status
    let arm_status = cw_comms::ArmStatus {
        positions: [
            arm_targets[0].1,
            arm_targets[1].1,
            arm_targets[2].1,
            arm_targets[3].1,
            arm_targets[4].1,
        ],
        ..Default::default()
    };
    bridge.publish_arm_status(&arm_status);

    println!("Control loop tick complete. State: {:?}", state_machine.state());
    println!("Ready. (Stub -- real loop requires embedded runtime)");
}
