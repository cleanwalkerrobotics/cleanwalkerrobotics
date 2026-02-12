# hardware — Codebase Manifest

All URDFs, PCB designs, BOM files, and mechanical specs owned by the hardware team.
Every file added or removed MUST be reflected here. Build will fail on untracked files.

## Bill of Materials

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `bom/README.md` | BOM management instructions | active | 2026-02-12 |
| `bom/bom-template.csv` | Base BOM template for component tracking | active | 2026-02-12 |

## Mainboard (Jetson Orin Carrier)

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `mainboard/README.md` | Mainboard PCB design overview and specs | active | 2026-02-12 |

## Motor Driver PCB

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `motor-driver-pcb/README.md` | Motor driver PCB design overview and specs | active | 2026-02-12 |

## URDF — CleanWalker CW-1

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `urdf/README.md` | URDF models overview, specs, and usage | active | 2026-02-12 |
| `urdf/cleanwalker.urdf` | CW-1 URDF v1 — 12-DOF quadruped model | active | 2026-02-12 |
| `urdf/validate.py` | URDF XML validation script | active | 2026-02-12 |
| `urdf/cleanwalker-cw1/cleanwalker_cw1.urdf` | CW-1 URDF v2 — refined model with sensors | active | 2026-02-12 |
| `urdf/cleanwalker-cw1/validate_urdf.py` | CW-1 v2 URDF validation with joint checks | active | 2026-02-12 |

## URDF — Unitree Go2 (Reference)

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `urdf/unitree-go2/README.md` | Go2 reference model source and mesh info | active | 2026-02-12 |
| `urdf/unitree-go2/go2_description.urdf` | Unitree Go2 full URDF (12 revolute joints) | active | 2026-02-12 |
| `urdf/unitree-go2/go2_mujoco.xml` | Go2 MuJoCo MJCF model for sim | active | 2026-02-12 |
| `urdf/unitree-go2/xacro/robot.xacro` | Go2 main robot xacro definition | active | 2026-02-12 |
| `urdf/unitree-go2/xacro/leg.xacro` | Go2 leg kinematic chain xacro | active | 2026-02-12 |
| `urdf/unitree-go2/xacro/const.xacro` | Go2 dimensional constants xacro | active | 2026-02-12 |
| `urdf/unitree-go2/xacro/gazebo.xacro` | Go2 Gazebo simulation properties | active | 2026-02-12 |
| `urdf/unitree-go2/xacro/materials.xacro` | Go2 material/color definitions | active | 2026-02-12 |
| `urdf/unitree-go2/xacro/transmission.xacro` | Go2 joint transmission definitions | active | 2026-02-12 |
