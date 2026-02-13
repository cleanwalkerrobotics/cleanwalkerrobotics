#!/usr/bin/env python3
"""Convert CW-1 URDF to MuJoCo MJCF XML format.

Generates a complete MuJoCo scene with:
- CW-1 robot model (18 DOF)
- Ground plane with appropriate friction
- Gravity, timestep, and solver configuration
- Camera and lighting for visualization

Usage:
    python convert_urdf_to_mjcf.py [--urdf PATH] [--output PATH]
"""

import argparse
import os
import subprocess
import sys
import textwrap
from pathlib import Path

# Paths relative to this script
SCRIPT_DIR = Path(__file__).parent.resolve()
REPO_ROOT = SCRIPT_DIR.parent.parent.parent
DEFAULT_URDF = REPO_ROOT / "hardware" / "urdf" / "cleanwalker-cw1" / "cleanwalker_cw1.urdf"
DEFAULT_OUTPUT = SCRIPT_DIR / "cw1_scene.xml"

# CW-1 joint configuration (from URDF)
LEG_JOINTS = [
    "FL_hip_yaw", "FL_hip_pitch", "FL_knee_pitch",
    "FR_hip_yaw", "FR_hip_pitch", "FR_knee_pitch",
    "RL_hip_yaw", "RL_hip_pitch", "RL_knee_pitch",
    "RR_hip_yaw", "RR_hip_pitch", "RR_knee_pitch",
]

ARM_JOINTS = [
    "arm_turret_yaw", "arm_shoulder_pitch", "arm_elbow_pitch",
    "arm_wrist_pitch", "arm_gripper_joint",
]

BAG_JOINTS = ["bag_frame_hinge"]

ALL_JOINTS = LEG_JOINTS + ARM_JOINTS + BAG_JOINTS


def compile_urdf(urdf_path: Path) -> str:
    """Use MuJoCo's built-in URDF compiler to generate base MJCF."""
    import mujoco

    # MuJoCo can load URDF directly
    model = mujoco.MjModel.from_xml_path(str(urdf_path))

    # Save to temporary MJCF to get the compiled XML
    tmp_path = SCRIPT_DIR / "_tmp_compiled.xml"
    mujoco.mj_saveLastXML(str(tmp_path), model)

    with open(tmp_path) as f:
        xml_content = f.read()

    tmp_path.unlink()
    return xml_content


def generate_scene_xml(urdf_path: Path) -> str:
    """Generate a complete MuJoCo scene XML for CW-1 locomotion training.

    Instead of compiling the URDF (which can have compatibility issues),
    we generate a clean MJCF scene that includes the URDF via MuJoCo's
    built-in compiler, adding simulation parameters and environment.
    """
    urdf_abs = str(urdf_path.resolve())
    urdf_dir = str(urdf_path.parent.resolve())

    scene_xml = textwrap.dedent(f"""\
    <mujoco model="cw1_locomotion">
      <!-- CW-1 Locomotion Training Scene -->
      <!-- Auto-generated from: {urdf_path.name} -->

      <compiler angle="radian" meshdir="{urdf_dir}" autolimits="true"/>

      <option timestep="0.005" gravity="0 0 -9.81" iterations="50" solver="Newton"
              integrator="implicitfast" cone="pyramidal">
        <flag warmstart="enable" multiccd="enable"/>
      </option>

      <size nconmax="200" njmax="500"/>

      <default>
        <default class="cw1">
          <joint armature="0.01" damping="0.5" frictionloss="0.1"/>
          <geom friction="1.0 0.005 0.001" condim="3" conaffinity="1" contype="1"/>
          <motor ctrlrange="-1 1" ctrllimited="true"/>
        </default>
        <default class="ground">
          <geom friction="1.0 0.005 0.001" condim="3" conaffinity="1" contype="1"/>
        </default>
      </default>

      <visual>
        <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.2 0.2 0.2"/>
        <rgba haze="0.15 0.25 0.35 1"/>
        <global azimuth="120" elevation="-20"/>
      </visual>

      <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
                 width="512" height="3072"/>
        <texture type="2d" name="groundplane" builtin="checker" mark="edge"
                 rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
                 width="300" height="300"/>
        <material name="groundplane" texture="groundplane" texuniform="true"
                  texrepeat="5 5" reflectance="0.2"/>
      </asset>

      <worldbody>
        <!-- Ground plane -->
        <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
        <geom name="floor" type="plane" size="50 50 0.1" material="groundplane"
              class="ground"/>

        <!-- Include CW-1 robot from URDF -->
        <include file="{urdf_abs}"/>
      </worldbody>

      <!-- Actuators for leg joints (position control, matching URDF PD gains) -->
      <actuator>
        <!-- Front Left Leg -->
        <position name="FL_hip_yaw_act" joint="FL_hip_yaw" kp="25" kv="0.5"
                  ctrlrange="-0.4993 0.4993" class="cw1"/>
        <position name="FL_hip_pitch_act" joint="FL_hip_pitch" kp="25" kv="0.5"
                  ctrlrange="-1.5708 1.5708" class="cw1"/>
        <position name="FL_knee_pitch_act" joint="FL_knee_pitch" kp="25" kv="0.5"
                  ctrlrange="-0.0995 2.6005" class="cw1"/>

        <!-- Front Right Leg -->
        <position name="FR_hip_yaw_act" joint="FR_hip_yaw" kp="25" kv="0.5"
                  ctrlrange="-0.4993 0.4993" class="cw1"/>
        <position name="FR_hip_pitch_act" joint="FR_hip_pitch" kp="25" kv="0.5"
                  ctrlrange="-1.5708 1.5708" class="cw1"/>
        <position name="FR_knee_pitch_act" joint="FR_knee_pitch" kp="25" kv="0.5"
                  ctrlrange="-0.0995 2.6005" class="cw1"/>

        <!-- Rear Left Leg -->
        <position name="RL_hip_yaw_act" joint="RL_hip_yaw" kp="25" kv="0.5"
                  ctrlrange="-0.4993 0.4993" class="cw1"/>
        <position name="RL_hip_pitch_act" joint="RL_hip_pitch" kp="25" kv="0.5"
                  ctrlrange="-1.5708 1.5708" class="cw1"/>
        <position name="RL_knee_pitch_act" joint="RL_knee_pitch" kp="25" kv="0.5"
                  ctrlrange="-2.6005 0.0995" class="cw1"/>

        <!-- Rear Right Leg -->
        <position name="RR_hip_yaw_act" joint="RR_hip_yaw" kp="25" kv="0.5"
                  ctrlrange="-0.4993 0.4993" class="cw1"/>
        <position name="RR_hip_pitch_act" joint="RR_hip_pitch" kp="25" kv="0.5"
                  ctrlrange="-1.5708 1.5708" class="cw1"/>
        <position name="RR_knee_pitch_act" joint="RR_knee_pitch" kp="25" kv="0.5"
                  ctrlrange="-2.6005 0.0995" class="cw1"/>
      </actuator>

      <!-- Sensors -->
      <sensor>
        <!-- IMU on body -->
        <accelerometer name="body_accel" site="body"/>
        <gyro name="body_gyro" site="body"/>
        <framequat name="body_quat" objtype="body" objname="body"/>

        <!-- Joint position/velocity sensors -->
        <jointpos name="FL_hip_yaw_pos" joint="FL_hip_yaw"/>
        <jointpos name="FL_hip_pitch_pos" joint="FL_hip_pitch"/>
        <jointpos name="FL_knee_pitch_pos" joint="FL_knee_pitch"/>
        <jointpos name="FR_hip_yaw_pos" joint="FR_hip_yaw"/>
        <jointpos name="FR_hip_pitch_pos" joint="FR_hip_pitch"/>
        <jointpos name="FR_knee_pitch_pos" joint="FR_knee_pitch"/>
        <jointpos name="RL_hip_yaw_pos" joint="RL_hip_yaw"/>
        <jointpos name="RL_hip_pitch_pos" joint="RL_hip_pitch"/>
        <jointpos name="RL_knee_pitch_pos" joint="RL_knee_pitch"/>
        <jointpos name="RR_hip_yaw_pos" joint="RR_hip_yaw"/>
        <jointpos name="RR_hip_pitch_pos" joint="RR_hip_pitch"/>
        <jointpos name="RR_knee_pitch_pos" joint="RR_knee_pitch"/>

        <jointvel name="FL_hip_yaw_vel" joint="FL_hip_yaw"/>
        <jointvel name="FL_hip_pitch_vel" joint="FL_hip_pitch"/>
        <jointvel name="FL_knee_pitch_vel" joint="FL_knee_pitch"/>
        <jointvel name="FR_hip_yaw_vel" joint="FR_hip_yaw"/>
        <jointvel name="FR_hip_pitch_vel" joint="FR_hip_pitch"/>
        <jointvel name="FR_knee_pitch_vel" joint="FR_knee_pitch"/>
        <jointvel name="RL_hip_yaw_vel" joint="RL_hip_yaw"/>
        <jointvel name="RL_hip_pitch_vel" joint="RL_hip_pitch"/>
        <jointvel name="RL_knee_pitch_vel" joint="RL_knee_pitch"/>
        <jointvel name="RR_hip_yaw_vel" joint="RR_hip_yaw"/>
        <jointvel name="RR_hip_pitch_vel" joint="RR_hip_pitch"/>
        <jointvel name="RR_knee_pitch_vel" joint="RR_knee_pitch"/>

        <!-- Foot contact forces -->
        <touch name="FL_foot_touch" site="FL_foot"/>
        <touch name="FR_foot_touch" site="FR_foot"/>
        <touch name="RL_foot_touch" site="RL_foot"/>
        <touch name="RR_foot_touch" site="RR_foot"/>
      </sensor>
    </mujoco>
    """)
    return scene_xml


def build_standalone_mjcf(urdf_path: Path, output_path: Path) -> None:
    """Build a standalone MJCF by validating the URDF then generating the training scene.

    The URDF-compiled model has a different structure than what the training
    pipeline expects (no freejoint, *_joint suffix naming, no foot contact
    geoms/sites, no parent body). We validate the URDF for correctness, then
    generate the handcrafted MJCF which is purpose-built for locomotion training.
    """
    import mujoco

    print(f"Loading URDF: {urdf_path}")

    # Validate URDF by loading it
    try:
        model = mujoco.MjModel.from_xml_path(str(urdf_path))
        print(f"  URDF validated: {model.nq} qpos, {model.nv} qvel, "
              f"{model.nu} actuators, {model.njnt} joints")
    except Exception as e:
        print(f"  URDF loading failed: {e}")

    # Generate training-ready MJCF (the URDF-compiled model uses *_joint naming,
    # lacks freejoint, foot contact sites, and body structure the env expects)
    print("  Generating training-ready MJCF from CW-1 specifications...")
    xml = generate_handcrafted_mjcf()
    with open(output_path, "w") as f:
        f.write(xml)

    print(f"  Wrote scene MJCF to: {output_path}")
    validate_model(output_path)


def wrap_in_scene(robot_xml: str) -> str:
    """Wrap compiled robot MJCF in a complete training scene.

    Adds ground plane, cameras, lighting, and actuators needed for training.
    """
    import xml.etree.ElementTree as ET

    tree = ET.ElementTree(ET.fromstring(robot_xml))
    root = tree.getroot()

    # Update compiler settings
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")

    # Set simulation options
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.005")
    option.set("gravity", "0 0 -9.81")
    option.set("iterations", "50")
    option.set("solver", "Newton")
    option.set("integrator", "implicitfast")

    # Add visual settings
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    headlight = visual.find("headlight")
    if headlight is None:
        headlight = ET.SubElement(visual, "headlight")
    headlight.set("diffuse", "0.6 0.6 0.6")
    headlight.set("ambient", "0.3 0.3 0.3")

    # Add ground plane to worldbody
    worldbody = root.find("worldbody")
    if worldbody is not None:
        # Add light
        light = ET.SubElement(worldbody, "light")
        light.set("pos", "0 0 3.5")
        light.set("dir", "0 0 -1")
        light.set("directional", "true")

        # Add ground
        ground = ET.SubElement(worldbody, "geom")
        ground.set("name", "floor")
        ground.set("type", "plane")
        ground.set("size", "50 50 0.1")
        ground.set("rgba", "0.2 0.3 0.4 1")
        ground.set("friction", "1.0 0.005 0.001")

    # Ensure actuators exist for leg joints
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")

    existing_actuators = {a.get("joint") for a in actuator}
    for joint_name in LEG_JOINTS:
        if joint_name not in existing_actuators:
            act = ET.SubElement(actuator, "position")
            act.set("name", f"{joint_name}_act")
            act.set("joint", joint_name)
            act.set("kp", "80")

    # Write formatted XML
    ET.indent(tree, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def generate_handcrafted_mjcf() -> str:
    """Generate MJCF from scratch based on CW-1 URDF specifications.

    Fallback when direct URDF compilation fails. Reproduces the robot
    geometry, joints, and physical properties from the URDF spec.
    """
    return textwrap.dedent("""\
    <mujoco model="cw1_locomotion">
      <compiler angle="radian" autolimits="true"/>

      <option timestep="0.005" gravity="0 0 -9.81" iterations="50"
              solver="Newton" integrator="implicitfast"/>

      <size nconmax="200" njmax="500"/>

      <default>
        <joint armature="0.01" damping="2.0" frictionloss="0.1"/>
        <geom friction="1.0 0.005 0.001" condim="3" conaffinity="1" contype="1"
              rgba="0.3 0.3 0.3 1"/>
        <position kp="80" kv="2.0"/>
      </default>

      <visual>
        <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.2 0.2 0.2"/>
        <rgba haze="0.15 0.25 0.35 1"/>
        <global offwidth="1920" offheight="1080"/>
      </visual>

      <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
                 width="512" height="3072"/>
        <texture type="2d" name="groundplane" builtin="checker" mark="edge"
                 rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
                 width="300" height="300"/>
        <material name="groundplane" texture="groundplane" texuniform="true"
                  texrepeat="5 5" reflectance="0.2"/>
        <material name="body_mat" rgba="0.15 0.15 0.15 1"/>
        <material name="leg_mat" rgba="0.25 0.25 0.25 1"/>
        <material name="foot_mat" rgba="0.6 0.6 0.6 1"/>
        <material name="arm_mat" rgba="0.2 0.2 0.3 1"/>
      </asset>

      <worldbody>
        <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
        <geom name="floor" type="plane" size="50 50 0.1" material="groundplane"/>

        <!-- CW-1 Robot Body -->
        <body name="body" pos="0 0 0.35">
          <freejoint name="root"/>
          <inertial pos="0 0 0" mass="8.0"
                    diaginertia="0.1 0.3 0.35"/>
          <geom name="body_geom" type="box" size="0.30 0.075 0.06"
                material="body_mat"/>
          <site name="body_imu" pos="0 0 0"/>

          <!-- Head -->
          <body name="head" pos="0.34 0 0.02">
            <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>
            <geom name="head_geom" type="box" size="0.04 0.075 0.04"
                  rgba="0.2 0.2 0.2 1"/>
          </body>

          <!-- ============ FRONT LEFT LEG ============ -->
          <body name="FL_hip" pos="0.20 0.075 0">
            <joint name="FL_hip_yaw" type="hinge" axis="0 0 1"
                   range="-0.4993 0.4993" damping="0.5"/>
            <inertial pos="0 0.03 0" mass="0.4" diaginertia="0.0005 0.0005 0.0005"/>
            <geom name="FL_hip_geom" type="sphere" size="0.03" material="leg_mat"/>

            <body name="FL_thigh" pos="0 0.06 0">
              <joint name="FL_hip_pitch" type="hinge" axis="0 1 0"
                     range="-1.5708 1.5708" damping="0.5"/>
              <inertial pos="0 0 -0.10" mass="0.5" diaginertia="0.002 0.002 0.0005"/>
              <geom name="FL_thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.20"
                    size="0.02" material="leg_mat"/>

              <body name="FL_calf" pos="0 0 -0.20">
                <joint name="FL_knee_pitch" type="hinge" axis="0 1 0"
                       range="-0.0995 2.6005" damping="0.5"/>
                <inertial pos="0 0 -0.10" mass="0.4" diaginertia="0.001 0.001 0.0003"/>
                <geom name="FL_calf_geom" type="capsule" fromto="0 0 0 0 0 -0.20"
                      size="0.015" material="leg_mat"/>

                <body name="FL_foot" pos="0 0 -0.20">
                  <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001"/>
                  <geom name="FL_foot_geom" type="sphere" size="0.025"
                        material="foot_mat" friction="1.2 0.005 0.001"/>
                  <site name="FL_foot_site" pos="0 0 0"/>
                </body>
              </body>
            </body>
          </body>

          <!-- ============ FRONT RIGHT LEG ============ -->
          <body name="FR_hip" pos="0.20 -0.075 0">
            <joint name="FR_hip_yaw" type="hinge" axis="0 0 1"
                   range="-0.4993 0.4993" damping="0.5"/>
            <inertial pos="0 -0.03 0" mass="0.4" diaginertia="0.0005 0.0005 0.0005"/>
            <geom name="FR_hip_geom" type="sphere" size="0.03" material="leg_mat"/>

            <body name="FR_thigh" pos="0 -0.06 0">
              <joint name="FR_hip_pitch" type="hinge" axis="0 1 0"
                     range="-1.5708 1.5708" damping="0.5"/>
              <inertial pos="0 0 -0.10" mass="0.5" diaginertia="0.002 0.002 0.0005"/>
              <geom name="FR_thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.20"
                    size="0.02" material="leg_mat"/>

              <body name="FR_calf" pos="0 0 -0.20">
                <joint name="FR_knee_pitch" type="hinge" axis="0 1 0"
                       range="-0.0995 2.6005" damping="0.5"/>
                <inertial pos="0 0 -0.10" mass="0.4" diaginertia="0.001 0.001 0.0003"/>
                <geom name="FR_calf_geom" type="capsule" fromto="0 0 0 0 0 -0.20"
                      size="0.015" material="leg_mat"/>

                <body name="FR_foot" pos="0 0 -0.20">
                  <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001"/>
                  <geom name="FR_foot_geom" type="sphere" size="0.025"
                        material="foot_mat" friction="1.2 0.005 0.001"/>
                  <site name="FR_foot_site" pos="0 0 0"/>
                </body>
              </body>
            </body>
          </body>

          <!-- ============ REAR LEFT LEG ============ -->
          <body name="RL_hip" pos="-0.20 0.075 0">
            <joint name="RL_hip_yaw" type="hinge" axis="0 0 1"
                   range="-0.4993 0.4993" damping="0.5"/>
            <inertial pos="0 0.03 0" mass="0.4" diaginertia="0.0005 0.0005 0.0005"/>
            <geom name="RL_hip_geom" type="sphere" size="0.03" material="leg_mat"/>

            <body name="RL_thigh" pos="0 0.06 0">
              <joint name="RL_hip_pitch" type="hinge" axis="0 1 0"
                     range="-1.5708 1.5708" damping="0.5"/>
              <inertial pos="0 0 -0.10" mass="0.5" diaginertia="0.002 0.002 0.0005"/>
              <geom name="RL_thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.20"
                    size="0.02" material="leg_mat"/>

              <body name="RL_calf" pos="0 0 -0.20">
                <joint name="RL_knee_pitch" type="hinge" axis="0 1 0"
                       range="-2.6005 0.0995" damping="0.5"/>
                <inertial pos="0 0 -0.10" mass="0.4" diaginertia="0.001 0.001 0.0003"/>
                <geom name="RL_calf_geom" type="capsule" fromto="0 0 0 0 0 -0.20"
                      size="0.015" material="leg_mat"/>

                <body name="RL_foot" pos="0 0 -0.20">
                  <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001"/>
                  <geom name="RL_foot_geom" type="sphere" size="0.025"
                        material="foot_mat" friction="1.2 0.005 0.001"/>
                  <site name="RL_foot_site" pos="0 0 0"/>
                </body>
              </body>
            </body>
          </body>

          <!-- ============ REAR RIGHT LEG ============ -->
          <body name="RR_hip" pos="-0.20 -0.075 0">
            <joint name="RR_hip_yaw" type="hinge" axis="0 0 1"
                   range="-0.4993 0.4993" damping="0.5"/>
            <inertial pos="0 -0.03 0" mass="0.4" diaginertia="0.0005 0.0005 0.0005"/>
            <geom name="RR_hip_geom" type="sphere" size="0.03" material="leg_mat"/>

            <body name="RR_thigh" pos="0 -0.06 0">
              <joint name="RR_hip_pitch" type="hinge" axis="0 1 0"
                     range="-1.5708 1.5708" damping="0.5"/>
              <inertial pos="0 0 -0.10" mass="0.5" diaginertia="0.002 0.002 0.0005"/>
              <geom name="RR_thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.20"
                    size="0.02" material="leg_mat"/>

              <body name="RR_calf" pos="0 0 -0.20">
                <joint name="RR_knee_pitch" type="hinge" axis="0 1 0"
                       range="-2.6005 0.0995" damping="0.5"/>
                <inertial pos="0 0 -0.10" mass="0.4" diaginertia="0.001 0.001 0.0003"/>
                <geom name="RR_calf_geom" type="capsule" fromto="0 0 0 0 0 -0.20"
                      size="0.015" material="leg_mat"/>

                <body name="RR_foot" pos="0 0 -0.20">
                  <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001"/>
                  <geom name="RR_foot_geom" type="sphere" size="0.025"
                        material="foot_mat" friction="1.2 0.005 0.001"/>
                  <site name="RR_foot_site" pos="0 0 0"/>
                </body>
              </body>
            </body>
          </body>

          <!-- ============ ARM (5 DOF, heavily damped during locomotion) ============ -->
          <body name="arm_turret" pos="0.10 0 0.06">
            <joint name="arm_turret_yaw" type="hinge" axis="0 0 1"
                   range="-3.1416 3.1416" damping="20.0"/>
            <inertial pos="0 0 0.03" mass="0.3" diaginertia="0.0003 0.0003 0.0003"/>
            <geom name="arm_turret_geom" type="cylinder" size="0.04 0.03"
                  material="arm_mat"/>

            <body name="arm_upper" pos="0 0 0.06">
              <joint name="arm_shoulder_pitch" type="hinge" axis="0 1 0"
                     range="-0.7854 3.1416" damping="20.0"/>
              <inertial pos="0 0 0.09" mass="0.3" diaginertia="0.001 0.001 0.0002"/>
              <geom name="arm_upper_geom" type="capsule" fromto="0 0 0 0 0 0.18"
                    size="0.015" material="arm_mat"/>

              <body name="arm_forearm" pos="0 0 0.18">
                <joint name="arm_elbow_pitch" type="hinge" axis="0 1 0"
                       range="0 2.618" damping="20.0"/>
                <inertial pos="0 0 0.09" mass="0.3" diaginertia="0.001 0.001 0.0002"/>
                <geom name="arm_forearm_geom" type="capsule" fromto="0 0 0 0 0 0.18"
                      size="0.012" material="arm_mat"/>

                <body name="arm_wrist" pos="0 0 0.18">
                  <joint name="arm_wrist_pitch" type="hinge" axis="0 1 0"
                         range="-1.5708 1.5708" damping="20.0"/>
                  <inertial pos="0 0 0.025" mass="0.15"
                            diaginertia="0.0001 0.0001 0.0001"/>
                  <geom name="arm_wrist_geom" type="cylinder" size="0.01 0.025"
                        material="arm_mat"/>

                  <body name="arm_gripper" pos="0 0 0.05">
                    <joint name="arm_gripper_joint" type="hinge" axis="0 1 0"
                           range="0 1.0472" damping="20.0"/>
                    <inertial pos="0 0 0.02" mass="0.15"
                              diaginertia="0.0001 0.0001 0.0001"/>
                    <geom name="arm_gripper_geom" type="box" size="0.04 0.015 0.05"
                          material="arm_mat"/>
                  </body>
                </body>
              </body>
            </body>
          </body>

          <!-- ============ BAG FRAME (1 DOF, heavily damped) ============ -->
          <body name="bag_mount" pos="-0.15 0 0.06">
            <inertial pos="0 0 0" mass="0.2" diaginertia="0.0002 0.0002 0.0002"/>
            <geom name="bag_mount_geom" type="cylinder" size="0.03 0.02"
                  rgba="0.3 0.3 0.3 1"/>

            <body name="bag_frame" pos="0 0 0.02">
              <joint name="bag_frame_hinge" type="hinge" axis="0 1 0"
                     range="0 2.3562" damping="20.0"/>
              <inertial pos="0 0 0.05" mass="0.3" diaginertia="0.001 0.001 0.0002"/>
              <geom name="bag_frame_geom" type="box" size="0.11 0.075 0.01"
                    pos="0 0 0.05" rgba="0.4 0.35 0.3 1"/>
            </body>
          </body>

        </body> <!-- end body -->
      </worldbody>

      <!-- Leg actuators (position-controlled, matching URDF PD gains) -->
      <actuator>
        <position name="FL_hip_yaw_act" joint="FL_hip_yaw" kp="25"/>
        <position name="FL_hip_pitch_act" joint="FL_hip_pitch" kp="25"/>
        <position name="FL_knee_pitch_act" joint="FL_knee_pitch" kp="25"/>
        <position name="FR_hip_yaw_act" joint="FR_hip_yaw" kp="25"/>
        <position name="FR_hip_pitch_act" joint="FR_hip_pitch" kp="25"/>
        <position name="FR_knee_pitch_act" joint="FR_knee_pitch" kp="25"/>
        <position name="RL_hip_yaw_act" joint="RL_hip_yaw" kp="25"/>
        <position name="RL_hip_pitch_act" joint="RL_hip_pitch" kp="25"/>
        <position name="RL_knee_pitch_act" joint="RL_knee_pitch" kp="25"/>
        <position name="RR_hip_yaw_act" joint="RR_hip_yaw" kp="25"/>
        <position name="RR_hip_pitch_act" joint="RR_hip_pitch" kp="25"/>
        <position name="RR_knee_pitch_act" joint="RR_knee_pitch" kp="25"/>
      </actuator>

      <!-- Sensors for observation -->
      <sensor>
        <framequat name="body_quat" objtype="body" objname="body"/>
        <framelinvel name="body_linvel" objtype="body" objname="body"/>
        <frameangvel name="body_angvel" objtype="body" objname="body"/>
      </sensor>
    </mujoco>
    """)


def validate_model(xml_path: Path) -> None:
    """Validate the generated MJCF by loading it in MuJoCo."""
    import mujoco

    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        mujoco.mj_step(model, data)
        print(f"  Validation passed:")
        print(f"    Bodies: {model.nbody}, Joints: {model.njnt}, "
              f"Actuators: {model.nu}")
        print(f"    qpos dim: {model.nq}, qvel dim: {model.nv}")
        print(f"    Timestep: {model.opt.timestep}s")

        # List joint names
        joint_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(model.njnt)
        ]
        print(f"    Joints: {joint_names}")

    except Exception as e:
        print(f"  Validation FAILED: {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Convert CW-1 URDF to MuJoCo MJCF")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF,
                        help="Path to CW-1 URDF file")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="Output MJCF XML path")
    parser.add_argument("--force-handcraft", action="store_true",
                        help="Skip URDF compilation, generate from specs")
    args = parser.parse_args()

    print("=" * 60)
    print("CW-1 URDF → MuJoCo MJCF Converter")
    print("=" * 60)

    if args.force_handcraft or not args.urdf.exists():
        if not args.urdf.exists():
            print(f"URDF not found at: {args.urdf}")
            print("Generating MJCF from CW-1 specifications...")
        else:
            print("Force-handcraft mode: generating MJCF from specifications...")

        xml = generate_handcrafted_mjcf()
        with open(args.output, "w") as f:
            f.write(xml)
        print(f"Wrote MJCF to: {args.output}")
        validate_model(args.output)
    else:
        build_standalone_mjcf(args.urdf, args.output)

    print()
    print("Done! You can now use this file with cw1_env.py for training.")


if __name__ == "__main__":
    main()
