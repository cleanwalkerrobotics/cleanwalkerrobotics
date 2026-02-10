# CleanWalker URDF Robot Model

First-pass URDF model of the CleanWalker quadruped robot for Gazebo simulation.
Based on `docs/design/robot-design-spec.md`.

## Specifications

| Property | Value |
|----------|-------|
| Total mass | ~50 kg |
| Body dimensions | 0.60m (L) x 0.25m (W) x 0.12m (H) |
| Standing height (body center) | ~0.50m |
| Upper leg length | 0.25m |
| Lower leg length | 0.25m |
| Total DOF | 12 (3 per leg) |

## Joint Structure

```
base_link (body)
├── arm_mount (fixed) — gripper arm attachment, top-front
├── bag_frame_mount (fixed) — bag cassette frame attachment, top-rear
├── led_strip_left (fixed) — cosmetic LED
├── led_strip_right (fixed) — cosmetic LED
├── eye_left (fixed) — front camera window
├── eye_right (fixed) — front camera window
│
├── FL (front-left leg)
│   ├── fl_hip_yaw (revolute, Z-axis) — lateral swing
│   ├── fl_hip_pitch (revolute, Y-axis) — forward/backward
│   ├── fl_knee_pitch (revolute, Y-axis) — knee bend (forward)
│   └── fl_foot (fixed) — rubber pad
│
├── FR (front-right leg)
│   ├── fr_hip_yaw → fr_hip_pitch → fr_knee_pitch → fr_foot
│
├── RL (rear-left leg)
│   ├── rl_hip_yaw → rl_hip_pitch → rl_knee_pitch → rl_foot
│
└── RR (rear-right leg)
    └── rr_hip_yaw → rr_hip_pitch → rr_knee_pitch → rr_foot
```

## Joint Limits

| Joint | Axis | Lower (rad) | Upper (rad) | Notes |
|-------|------|-------------|-------------|-------|
| `*_hip_yaw` | Z | -0.5 | 0.5 | Lateral leg swing |
| `*_hip_pitch` | Y | -1.57 | 1.57 | Forward/backward swing |
| `fl/fr_knee_pitch` | Y | -0.1 | 2.6 | Front knees bend forward |
| `rl/rr_knee_pitch` | Y | -2.6 | 0.1 | Rear knees bend backward |

## Mammalian Stance

The robot uses a mammalian leg configuration:
- **Front legs:** knees bend forward (positive knee pitch)
- **Rear legs:** knees bend backward (negative knee pitch)

This is reflected in the asymmetric joint limits for front vs. rear knee joints.

## Colors

| Component | Color | Hex |
|-----------|-------|-----|
| Body | Dark olive-green | #3B4A3F |
| Hip joints | Orange-amber | accent |
| LED strips/eyes | Bright green | #22C55E |
| Feet | Rubber black | — |

## How to Validate

Parse-check the URDF XML:

```bash
python3 -c "import xml.etree.ElementTree as ET; ET.parse('hardware/urdf/cleanwalker.urdf'); print('URDF valid XML')"
```

## How to Visualize

### With ROS2 + RViz

```bash
# Install urdf_tutorial if needed
sudo apt install ros-${ROS_DISTRO}-urdf-tutorial

# View in RViz with joint sliders
ros2 launch urdf_tutorial display.launch.py model:=hardware/urdf/cleanwalker.urdf
```

### With Gazebo (standalone)

```bash
# Classic Gazebo
gazebo --verbose -s libgazebo_ros_factory.so

# Then spawn the model
ros2 run gazebo_ros spawn_entity.py -file hardware/urdf/cleanwalker.urdf -entity cleanwalker
```

### With check_urdf (ROS tool)

```bash
check_urdf hardware/urdf/cleanwalker.urdf
```

## What's Included

- 12-DOF quadruped kinematic chain
- Simplified box/cylinder collision geometries
- Mass and inertia properties for all links
- Gazebo material colors and friction properties
- Gripper arm attachment point (arm_mount)
- Bag cassette frame attachment point (bag_frame_mount)
- Cosmetic LED strips and eye indicators

## What's NOT Included (Future Work)

- Detailed STL/mesh visuals
- Sensor models (LiDAR, cameras)
- Articulated gripper arm joints
- Bag cassette scissor mechanism
- ROS2 control plugins / launch files
- Walking gait controller
