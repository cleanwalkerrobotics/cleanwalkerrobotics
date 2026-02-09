// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2026 CleanWalker Robotics

/// Direction of motor rotation.
pub enum Direction {
    Forward,
    Reverse,
    Stop,
}

/// Trait for motor control implementations.
pub trait Motor {
    fn set_speed(&mut self, speed: u16);
    fn set_direction(&mut self, direction: Direction);
    fn stop(&mut self);
}

/// Motor driver for dual H-bridge control.
pub struct MotorDriver {
    left_speed: u16,
    right_speed: u16,
}

impl MotorDriver {
    pub fn new() -> Self {
        Self {
            left_speed: 0,
            right_speed: 0,
        }
    }

    pub fn set_left_speed(&mut self, speed: u16) {
        self.left_speed = speed;
    }

    pub fn set_right_speed(&mut self, speed: u16) {
        self.right_speed = speed;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_driver_new() {
        let driver = MotorDriver::new();
        assert_eq!(driver.left_speed, 0);
        assert_eq!(driver.right_speed, 0);
    }
}
