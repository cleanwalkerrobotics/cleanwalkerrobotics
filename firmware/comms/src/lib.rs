// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

/// Communication channel type.
pub enum CommChannel {
    /// 4G LTE cellular connection
    Cellular,
    /// Mesh network for robot-to-robot communication
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_comms_manager_new() {
        let manager = CommsManager::new();
        assert!(!manager.is_connected());
    }
}
