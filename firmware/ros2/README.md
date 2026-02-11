# CleanWalker CW-1 — ROS 2 Packages

ROS 2 Humble package structure for the CleanWalker CW-1 autonomous litter-picking quadruped.

**Status:** Foundational stubs — establishes codebase architecture for real development. Requires ROS 2 Humble + Jetson Orin Nano Super to run.

## Architecture

```
                    THREE CONCURRENT PIPELINES
═══════════════════════════════════════════════════════════════

PIPELINE 1: DETECTION → DEPTH → GRASP → ARM
──────────────────────────────────────────────────────────────
  OAK-D Pro RGB ──► YOLO26s (TRT INT8) ──► /detections
       30 Hz            5-10 ms              (bbox, class, conf)
                                                    │
  OAK-D Pro Stereo ──► Depth Filter ──► /depth/metric
       30 Hz                              (metric depth)
                                                    │
                                    ┌───────────────┘
                                    ▼
                           ROI Crop + Resize ──► GR-ConvNet v2
                                <2 ms              5-10 ms (TRT)
                                                      │
                                                      ▼
                                              /grasp_pose
                                                      │
                                                      ▼
                                              Arm IK + Execute
                                                (analytical)

PIPELINE 2: SEGMENTATION → NAVIGATION
──────────────────────────────────────────────────────────────
  OAK-D Pro RGB ──► SegFormer-B0 (TRT FP16) ──► /terrain/mask
       30 Hz             ~25 ms @ 5 Hz            (8-class map)
                                                       │
                                                       ▼
                                              /terrain/costmap
                                                       │
                                                       ▼
                                              Nav2 Path Planning

PIPELINE 3: SLAM → LOCALIZATION → MAP
──────────────────────────────────────────────────────────────
  OAK-D Pro Stereo ──┐
       30 Hz          │
                      ├──► cuVSLAM (GPU) ──► /visual_odom
  OAK-D Pro IMU ─────┘      ~9% GPU           (30 Hz VO)
       400 Hz                                      │
                                                   ▼
  Wheel/Leg Odom ──────────► robot_localization (EKF)
       50 Hz                        │
                                    ▼
                              /odometry/filtered
                                    │
  Livox Mid-360 ───────┐           │
       10 Hz            ├───► RTAB-Map (5 Hz)
  OAK-D Pro Depth ─────┘           │
                                    ▼
                              /map (OccupancyGrid)
```

## Packages

| Package | Purpose | Nodes |
|---------|---------|-------|
| `cleanwalker_bringup` | Launch files and configuration | — (launch only) |
| `cleanwalker_perception` | Litter detection, depth, terrain segmentation | `detection_node`, `depth_node`, `segmentation_node` |
| `cleanwalker_navigation` | SLAM (cuVSLAM wrapper) and coverage planning | `slam_node`, `path_planner_node` |
| `cleanwalker_manipulation` | Grasp planning and arm control | `grasp_planner_node`, `arm_controller_node` |
| `cleanwalker_description` | URDF robot model and RViz visualization | — (launch only) |

## Topics

| Topic | Message Type | Publisher | Rate |
|-------|-------------|-----------|------|
| `/oak/rgb/image` | `sensor_msgs/Image` | OAK-D driver | 30 Hz |
| `/oak/stereo/depth` | `sensor_msgs/Image` | OAK-D driver | 30 Hz |
| `/oak/stereo/left` | `sensor_msgs/Image` | OAK-D driver | 30 Hz |
| `/oak/stereo/right` | `sensor_msgs/Image` | OAK-D driver | 30 Hz |
| `/oak/imu` | `sensor_msgs/Imu` | OAK-D driver | 400 Hz |
| `/livox/pointcloud` | `sensor_msgs/PointCloud2` | Livox driver | 10 Hz |
| `/detections` | `vision_msgs/Detection3DArray` | `detection_node` | 30 Hz |
| `/depth/metric` | `sensor_msgs/Image` | `depth_node` | 30 Hz |
| `/terrain/mask` | `sensor_msgs/Image` | `segmentation_node` | 5 Hz |
| `/terrain/costmap` | `nav_msgs/OccupancyGrid` | `segmentation_node` | 5 Hz |
| `/visual_odom` | `nav_msgs/Odometry` | `slam_node` | 30 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | robot_localization | 50 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map | 1 Hz |
| `/coverage/path` | `nav_msgs/Path` | `path_planner_node` | Event |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `path_planner_node` | Event |
| `/grasp_pose` | `geometry_msgs/PoseStamped` | `grasp_planner_node` | Event |
| `/arm/joint_commands` | `trajectory_msgs/JointTrajectory` | `arm_controller_node` | Event |
| `/grasp/status` | `std_msgs/String` | `grasp_planner_node` | Event |
| `/arm/status` | `std_msgs/String` | `arm_controller_node` | Event |

## Custom Messages

| Message | Package | Fields |
|---------|---------|--------|
| `Detection.msg` | `cleanwalker_perception` | header, bbox, class_name, class_id, confidence, position, has_depth |
| `LitterItem.msg` | `cleanwalker_perception` | header, track_id, class/pose/size, recommended_grasp, tracking metadata |

## Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04 / JetPack 6.x)
- **Python 3.10+**
- **NVIDIA Jetson Orin Nano Super** (for TensorRT inference)
- **Isaac ROS** packages (cuVSLAM, ESS — optional Phase 2)

## Build

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace (if not using an existing one)
mkdir -p ~/cleanwalker_ws/src
cd ~/cleanwalker_ws/src

# Symlink packages from repo
ln -s /path/to/cleanwalkerrobotics/firmware/ros2/cleanwalker_bringup .
ln -s /path/to/cleanwalkerrobotics/firmware/ros2/cleanwalker_perception .
ln -s /path/to/cleanwalkerrobotics/firmware/ros2/cleanwalker_navigation .
ln -s /path/to/cleanwalkerrobotics/firmware/ros2/cleanwalker_manipulation .
ln -s /path/to/cleanwalkerrobotics/firmware/ros2/cleanwalker_description .

# Install dependencies
cd ~/cleanwalker_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Launch

```bash
# Full robot bringup
ros2 launch cleanwalker_bringup robot.launch.py

# Simulation mode
ros2 launch cleanwalker_bringup robot.launch.py use_sim:=true

# Disable segmentation (saves ~10% GPU)
ros2 launch cleanwalker_bringup robot.launch.py enable_segmentation:=false

# Visualize URDF in RViz
ros2 launch cleanwalker_description display.launch.py
```

## Configuration

All tunable parameters are in `cleanwalker_bringup/config/robot_params.yaml`:

- Detection confidence thresholds
- Depth filtering ranges
- Segmentation inference rate
- Coverage sweep width and overlap
- Arm trajectory velocity and timing
- Grasp planner strategy (rule-based vs learned)

## Development Roadmap

1. **Now:** Stub nodes with correct topic names and message types
2. **Phase 1:** Integrate OAK-D Pro driver, TensorRT YOLO26s, analytical arm IK
3. **Phase 2:** Add Isaac ROS ESS, GR-ConvNet v2, SegFormer-B0, Nav2
4. **Phase 3:** cuVSLAM + RTAB-Map hybrid SLAM, behavior tree, full integration

## References

- [Perception Pipeline Architecture](../../docs/technical/perception-pipeline-architecture.md)
- [Autonomy Stack Architecture](../../docs/technical/autonomy-stack-architecture.md)
- [URDF Model](../../hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.urdf)
