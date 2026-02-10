# Unitree Go2 URDF / Model Files

Source: [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) (master branch)

## Included Files

- `go2_description.urdf` — Full URDF (27KB, 761 lines, 12 revolute joints)
- `xacro/` — Xacro source files (robot, leg, const, gazebo, materials, transmission)
- `go2_mujoco.xml` — MuJoCo MJCF from [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)

## Mesh Files (NOT included — too large for repo)

The URDF references DAE mesh files (~24.7 MB total) at `package://go2_description/dae/`.

To download meshes:

```bash
# Option 1: Sparse clone just go2_description
git clone --depth 1 --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_ros.git /tmp/unitree_ros
cd /tmp/unitree_ros && git sparse-checkout set robots/go2_description
cp -r robots/go2_description/dae ./

# Option 2: MuJoCo OBJ meshes (~27.1 MB, 16 files)
git clone --depth 1 --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_mujoco.git /tmp/unitree_mujoco
cd /tmp/unitree_mujoco && git sparse-checkout set unitree_robots/go2
```

## Robot Specs (from URDF)

- Base mass: 6.921 kg
- 12 revolute joints (3 per leg: hip abduction, hip flexion, knee)
- 7 mesh parts: base, hip, thigh, thigh_mirror, calf, calf_mirror, foot
- Collision geometry: simplified boxes/cylinders
