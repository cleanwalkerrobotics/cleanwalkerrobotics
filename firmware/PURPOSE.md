# firmware — Purpose

Rust firmware for CW-1 motor control, gait generation, and ROS2 integration.

**Owner:** cw-hardware
**What belongs here:** Rust crates (controller, motor-driver, comms), ROS2 packages (bringup, description, navigation, perception, manipulation), embedded firmware.
**What does NOT belong here:** PCB design files (→ hardware/), ML models (→ ml/), web UI (→ apps/web/).
**Key constraint:** All crates must be registered in the workspace `Cargo.toml`.
