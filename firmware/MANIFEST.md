# firmware — Codebase Manifest

All Rust crates, ROS2 packages, controllers, and interfaces owned by the firmware team.
Every file added or removed MUST be reflected here. Build will fail on untracked files.

## Workspace

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `Cargo.toml` | Cargo workspace definition (controller, motor-driver, comms) | active | 2026-02-12 |

## Controller Crate

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `controller/Cargo.toml` | Controller crate dependencies and metadata | active | 2026-02-12 |
| `controller/src/main.rs` | Controller binary entry point | active | 2026-02-12 |
| `controller/src/lib.rs` | Controller library — gait controller, state machine | active | 2026-02-12 |

## Motor Driver Crate

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `motor-driver/Cargo.toml` | Motor driver crate dependencies | active | 2026-02-12 |
| `motor-driver/src/lib.rs` | Motor driver HAL — PWM, current sensing, CAN | active | 2026-02-12 |

## Comms Crate

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `comms/Cargo.toml` | Communications crate dependencies | active | 2026-02-12 |
| `comms/src/lib.rs` | CAN bus + telemetry communication layer | active | 2026-02-12 |

## ROS2 — Overview

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `ros2/README.md` | ROS2 workspace overview and setup guide | active | 2026-02-12 |

## ROS2 — Bringup Package

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `ros2/cleanwalker_bringup/package.xml` | ROS2 package manifest for bringup | active | 2026-02-12 |
| `ros2/cleanwalker_bringup/setup.py` | Python package setup for bringup | active | 2026-02-12 |
| `ros2/cleanwalker_bringup/setup.cfg` | Setup config for bringup | active | 2026-02-12 |
| `ros2/cleanwalker_bringup/cleanwalker_bringup/__init__.py` | Bringup package init | active | 2026-02-12 |
| `ros2/cleanwalker_bringup/launch/robot.launch.py` | Full robot launch file — starts all nodes | active | 2026-02-12 |
| `ros2/cleanwalker_bringup/config/robot_params.yaml` | Robot configuration parameters | active | 2026-02-12 |
| `ros2/cleanwalker_bringup/resource/cleanwalker_bringup` | Ament resource index marker | active | 2026-02-12 |

## ROS2 — Description Package

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `ros2/cleanwalker_description/package.xml` | ROS2 package manifest for description | active | 2026-02-12 |
| `ros2/cleanwalker_description/setup.py` | Python package setup for description | active | 2026-02-12 |
| `ros2/cleanwalker_description/setup.cfg` | Setup config for description | active | 2026-02-12 |
| `ros2/cleanwalker_description/cleanwalker_description/__init__.py` | Description package init | active | 2026-02-12 |
| `ros2/cleanwalker_description/launch/display.launch.py` | RViz URDF display launch file | active | 2026-02-12 |
| `ros2/cleanwalker_description/urdf/cleanwalker_cw1.urdf` | Symlink to hardware/urdf CW-1 URDF | active | 2026-02-12 |
| `ros2/cleanwalker_description/resource/cleanwalker_description` | Ament resource index marker | active | 2026-02-12 |

## ROS2 — Perception Package

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `ros2/cleanwalker_perception/package.xml` | ROS2 package manifest for perception | active | 2026-02-12 |
| `ros2/cleanwalker_perception/setup.py` | Python package setup for perception | active | 2026-02-12 |
| `ros2/cleanwalker_perception/setup.cfg` | Setup config for perception | active | 2026-02-12 |
| `ros2/cleanwalker_perception/cleanwalker_perception/__init__.py` | Perception package init | active | 2026-02-12 |
| `ros2/cleanwalker_perception/cleanwalker_perception/detection_node.py` | YOLO litter detection ROS2 node | active | 2026-02-12 |
| `ros2/cleanwalker_perception/cleanwalker_perception/depth_node.py` | Depth estimation ROS2 node | active | 2026-02-12 |
| `ros2/cleanwalker_perception/cleanwalker_perception/segmentation_node.py` | Semantic segmentation ROS2 node | active | 2026-02-12 |
| `ros2/cleanwalker_perception/nodes/detection_node.py` | Detection node entry point | active | 2026-02-12 |
| `ros2/cleanwalker_perception/nodes/depth_node.py` | Depth node entry point | active | 2026-02-12 |
| `ros2/cleanwalker_perception/nodes/segmentation_node.py` | Segmentation node entry point | active | 2026-02-12 |
| `ros2/cleanwalker_perception/msg/Detection.msg` | Detection message definition | active | 2026-02-12 |
| `ros2/cleanwalker_perception/msg/LitterItem.msg` | LitterItem message definition | active | 2026-02-12 |
| `ros2/cleanwalker_perception/resource/cleanwalker_perception` | Ament resource index marker | active | 2026-02-12 |

## ROS2 — Navigation Package

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `ros2/cleanwalker_navigation/package.xml` | ROS2 package manifest for navigation | active | 2026-02-12 |
| `ros2/cleanwalker_navigation/setup.py` | Python package setup for navigation | active | 2026-02-12 |
| `ros2/cleanwalker_navigation/setup.cfg` | Setup config for navigation | active | 2026-02-12 |
| `ros2/cleanwalker_navigation/cleanwalker_navigation/__init__.py` | Navigation package init | active | 2026-02-12 |
| `ros2/cleanwalker_navigation/cleanwalker_navigation/slam_node.py` | SLAM localization ROS2 node | active | 2026-02-12 |
| `ros2/cleanwalker_navigation/cleanwalker_navigation/path_planner_node.py` | Path planning ROS2 node | active | 2026-02-12 |
| `ros2/cleanwalker_navigation/nodes/slam_node.py` | SLAM node entry point | active | 2026-02-12 |
| `ros2/cleanwalker_navigation/nodes/path_planner_node.py` | Path planner node entry point | active | 2026-02-12 |
| `ros2/cleanwalker_navigation/resource/cleanwalker_navigation` | Ament resource index marker | active | 2026-02-12 |

## ROS2 — Manipulation Package

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `ros2/cleanwalker_manipulation/package.xml` | ROS2 package manifest for manipulation | active | 2026-02-12 |
| `ros2/cleanwalker_manipulation/setup.py` | Python package setup for manipulation | active | 2026-02-12 |
| `ros2/cleanwalker_manipulation/setup.cfg` | Setup config for manipulation | active | 2026-02-12 |
| `ros2/cleanwalker_manipulation/cleanwalker_manipulation/__init__.py` | Manipulation package init | active | 2026-02-12 |
| `ros2/cleanwalker_manipulation/cleanwalker_manipulation/arm_controller_node.py` | Gripper arm controller ROS2 node | active | 2026-02-12 |
| `ros2/cleanwalker_manipulation/cleanwalker_manipulation/grasp_planner_node.py` | Grasp planning ROS2 node | active | 2026-02-12 |
| `ros2/cleanwalker_manipulation/nodes/arm_controller_node.py` | Arm controller node entry point | active | 2026-02-12 |
| `ros2/cleanwalker_manipulation/nodes/grasp_planner_node.py` | Grasp planner node entry point | active | 2026-02-12 |
| `ros2/cleanwalker_manipulation/resource/cleanwalker_manipulation` | Ament resource index marker | active | 2026-02-12 |
