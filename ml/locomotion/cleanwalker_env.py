# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""
CleanWalker CW-1 — IsaacLab DirectRLEnv for quadruped locomotion.

This environment trains a 12-joint locomotion policy using PPO. The robot
has 18 DOF total (12 leg + 5 arm + 1 bag hinge), but only the 12 leg
joints are actuated during locomotion training. Arm and bag joints are
held at their default positions.

Architecture follows the ANYmal-C direct environment pattern from IsaacLab.

Joint order (12 actions):
    FL_hip_yaw, FL_hip_pitch, FL_knee_pitch,
    FR_hip_yaw, FR_hip_pitch, FR_knee_pitch,
    RL_hip_yaw, RL_hip_pitch, RL_knee_pitch,
    RR_hip_yaw, RR_hip_pitch, RR_knee_pitch

Observation vector (48 dims):
    [0:3]   base linear velocity (body frame)
    [3:6]   base angular velocity (body frame)
    [6:9]   projected gravity (body frame)
    [9:12]  velocity commands (vx, vy, yaw_rate)
    [12:24] joint positions relative to default
    [24:36] joint velocities
    [36:48] previous actions
"""

from __future__ import annotations

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

from .terrain_config import ROUGH_TERRAIN_CFG
from . import rewards as litter_rewards

# ---------------------------------------------------------------------------
# URDF path (relative to repository root). Convert to USD before training:
#
#   ./isaaclab.sh -p scripts/tools/convert_urdf.py \
#       hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.urdf \
#       hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.usd \
#       --merge-joints \
#       --make-instanceable \
#       --joint-stiffness 0.0 \
#       --joint-damping 0.0 \
#       --joint-target-type none
#
# After conversion, update CLEANWALKER_URDF_PATH to point to the .usd file
# and switch spawn to sim_utils.UsdFileCfg.
# ---------------------------------------------------------------------------

CLEANWALKER_URDF_PATH = "hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.urdf"

# ---------------------------------------------------------------------------
# Leg joint names in SDK order (matches URDF joint definitions).
# Only these 12 joints are actuated during locomotion training.
# ---------------------------------------------------------------------------

LEG_JOINT_NAMES = [
    "FL_hip_yaw_joint", "FL_hip_pitch_joint", "FL_knee_pitch_joint",
    "FR_hip_yaw_joint", "FR_hip_pitch_joint", "FR_knee_pitch_joint",
    "RL_hip_yaw_joint", "RL_hip_pitch_joint", "RL_knee_pitch_joint",
    "RR_hip_yaw_joint", "RR_hip_pitch_joint", "RR_knee_pitch_joint",
]

# Bodies that should NOT contact the ground (penalty for undesired contacts)
PENALIZED_CONTACT_BODIES = [
    "body", "head",
    "FL_hip", "FL_thigh", "FR_hip", "FR_thigh",
    "RL_hip", "RL_thigh", "RR_hip", "RR_thigh",
    "FL_calf", "FR_calf", "RL_calf", "RR_calf",
    "arm_turret", "arm_upper", "arm_forearm", "arm_wrist", "arm_gripper",
    "bag_roll_mount", "bag_frame_hinge", "bag_frame",
]

# Foot links (desired ground contact)
FOOT_BODIES = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]

# Arm joint names (5 DOF — actuated during litter collection)
ARM_JOINT_NAMES = [
    "arm_turret_yaw_joint",
    "arm_shoulder_pitch_joint",
    "arm_elbow_pitch_joint",
    "arm_wrist_pitch_joint",
    "arm_gripper_joint",
]

# Default arm target for ground pickup (pre-computed IK):
# turret: forward (0), shoulder: extended (2.5), elbow: bent (1.5),
# wrist: down (-0.5), gripper: open (0.0) → closed (0.8)
ARM_PICKUP_TARGET = [0.0, 2.5, 1.5, -0.5, 0.0]
ARM_GRIPPER_CLOSED = 0.8


# ===========================================================================
# Robot articulation configuration
# ===========================================================================

CLEANWALKER_CW1_CFG = ArticulationCfg(
    prim_path="/World/envs/env_.*/Robot",
    spawn=sim_utils.UrdfFileCfg(
        asset_path=CLEANWALKER_URDF_PATH,
        fix_base=False,
        activate_contact_sensors=True,
        replace_cylinders_with_capsules=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # Body height: ~0.35m with bent-leg standing pose
        pos=(0.0, 0.0, 0.35),
        joint_pos={
            # Hip yaw: slight abduction (left positive, right negative)
            ".*L_hip_yaw_joint": 0.1,
            ".*R_hip_yaw_joint": -0.1,
            # Hip pitch: thighs angled for standing crouch
            ".*_hip_pitch_joint": 0.8,
            # Knee pitch: front bends positive, rear bends negative
            # (asymmetric limits: front [-0.1, 2.6], rear [-2.6, 0.1])
            "FL_knee_pitch_joint": 1.5,
            "FR_knee_pitch_joint": 1.5,
            "RL_knee_pitch_joint": -1.5,
            "RR_knee_pitch_joint": -1.5,
            # Arm and bag held at zero during locomotion
            "arm_turret_yaw_joint": 0.0,
            "arm_shoulder_pitch_joint": 0.0,
            "arm_elbow_pitch_joint": 0.0,
            "arm_wrist_pitch_joint": 0.0,
            "arm_gripper_joint": 0.0,
            "bag_frame_hinge_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        # Leg actuators: 12 joints with position control (PD)
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_pitch_joint",
                ".*_knee_pitch_joint",
            ],
            effort_limit=30.0,
            velocity_limit=10.0,
            stiffness=25.0,
            damping=0.5,
        ),
        # Arm actuators: held stiff at default during locomotion
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["arm_.*"],
            effort_limit=15.0,
            velocity_limit=5.0,
            stiffness=40.0,
            damping=2.0,
        ),
        # Bag hinge: held stiff at default during locomotion
        "bag": ImplicitActuatorCfg(
            joint_names_expr=["bag_frame_hinge_joint"],
            effort_limit=10.0,
            velocity_limit=2.0,
            stiffness=40.0,
            damping=2.0,
        ),
    },
)


# ===========================================================================
# Environment config — Flat terrain (Phase 1: curriculum start)
# ===========================================================================

@configclass
class CleanWalkerFlatEnvCfg(DirectRLEnvCfg):
    """Flat terrain locomotion for CleanWalker CW-1."""

    # --- Environment ---
    episode_length_s: float = 20.0
    decimation: int = 4              # policy runs at 50 Hz (sim 200 Hz / 4)
    action_scale: float = 0.25       # conservative for 15 kg robot
    action_space: int = 12           # 12 leg joints
    observation_space: int = 48      # see module docstring
    state_space: int = 0

    # --- Simulation ---
    sim: SimulationCfg = SimulationCfg(
        dt=1.0 / 200.0,
        render_interval=4,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    # --- Scene ---
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096,
        env_spacing=4.0,
        replicate_physics=True,
    )

    # --- Terrain (flat) ---
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # --- Robot ---
    robot: ArticulationCfg = CLEANWALKER_CW1_CFG

    # --- Contact sensor ---
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*",
        history_length=3,
        update_period=0.005,         # 200 Hz, matches sim dt
        track_air_time=True,
    )

    # --- Velocity command ranges ---
    cmd_vx_range: tuple[float, float] = (-1.0, 1.5)   # m/s forward
    cmd_vy_range: tuple[float, float] = (-0.5, 0.5)    # m/s lateral
    cmd_yaw_range: tuple[float, float] = (-1.5, 1.5)   # rad/s yaw rate

    # --- Reward scales ---
    # Tracking rewards (positive)
    rew_track_lin_vel_xy: float = 1.5
    rew_track_ang_vel_z: float = 0.75
    rew_feet_air_time: float = 0.5

    # Penalty rewards (negative)
    rew_lin_vel_z: float = -2.0
    rew_ang_vel_xy: float = -0.05
    rew_joint_torques: float = -2.5e-5
    rew_joint_accel: float = -2.5e-7
    rew_action_rate: float = -0.01
    rew_flat_orientation: float = -5.0
    rew_undesired_contacts: float = -1.0
    rew_joint_pos_limits: float = -10.0
    rew_stumble: float = -0.5

    # --- Domain randomization ---
    # Applied in _reset_idx via additive noise
    randomize_friction: bool = True
    friction_range: tuple[float, float] = (0.5, 1.5)
    randomize_base_mass: bool = True
    base_mass_range: tuple[float, float] = (-1.0, 3.0)   # additive kg
    randomize_motor_strength: bool = True
    motor_strength_range: tuple[float, float] = (0.8, 1.2)  # multiplier


@configclass
class CleanWalkerRoughEnvCfg(CleanWalkerFlatEnvCfg):
    """Rough terrain locomotion for CleanWalker CW-1 (Phase 2: curriculum)."""

    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=ROUGH_TERRAIN_CFG,
        max_init_terrain_level=9,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # Relax flat orientation penalty for rough terrain
    rew_flat_orientation: float = -1.0


# ===========================================================================
# Litter Collection Environment Config (Phase 3)
# ===========================================================================

@configclass
class CleanWalkerLitterEnvCfg(CleanWalkerFlatEnvCfg):
    """Litter collection environment — locomotion + arm control.

    Extends flat terrain locomotion with stop-and-reach behavior
    for picking up litter. The robot learns to:
      1. Walk toward a commanded position (velocity tracking)
      2. Stop smoothly when a pick command is issued
      3. Stabilize body for arm operation
      4. Extend arm toward ground target
      5. Close gripper to grasp litter

    Action space expands from 12 (legs only) to 17 (legs + arm):
      [0:12]  leg joints (same as locomotion)
      [12:17] arm joints (turret, shoulder, elbow, wrist, gripper)

    Observation space expands from 48 to 58:
      [0:48]  same as locomotion
      [48:53] arm joint positions (relative to default)
      [53:58] arm joint velocities
      Plus pick_command flag is embedded in commands[2] override.
    """

    # Expanded action/observation spaces
    action_space: int = 17           # 12 leg + 5 arm
    observation_space: int = 58      # 48 base + 5 arm pos + 5 arm vel

    # Episode is shorter — pickup should be fast
    episode_length_s: float = 15.0

    # --- Litter collection reward scales ---
    # Stop-and-reach rewards (active during pick command)
    rew_stop_and_stabilize: float = 2.0
    rew_arm_reach: float = 3.0
    rew_grasp: float = 5.0
    rew_smooth_deceleration: float = -2.0

    # Enhanced smoothness rewards
    rew_jerk: float = -0.005         # Penalize jerk (2nd derivative of action)
    rew_power: float = -1.0e-5       # Penalize mechanical power

    # Probability of pick command per episode reset
    pick_command_prob: float = 0.4   # 40% of episodes include a pickup phase

    # Pick command activates after this many seconds into the episode
    pick_command_delay_range: tuple[float, float] = (3.0, 8.0)


# ===========================================================================
# Environment implementation
# ===========================================================================

class CleanWalkerLocomotionEnv(DirectRLEnv):
    """CleanWalker CW-1 locomotion environment.

    Trains a velocity-tracking locomotion policy for a 15 kg quadruped
    with 12 actuated leg joints using PPO via rsl_rl.
    """

    cfg: CleanWalkerFlatEnvCfg

    def __init__(self, cfg: CleanWalkerFlatEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # Action buffers
        self._actions = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)
        self._previous_actions = torch.zeros_like(self._actions)

        # Velocity command buffer: [vx, vy, yaw_rate]
        self._commands = torch.zeros(self.num_envs, 3, device=self.device)

        # Resolve body indices for contact rewards/termination
        # Use contact_sensor.find_bodies() so indices match sensor data arrays
        self._base_id, _ = self._contact_sensor.find_bodies("body")
        self._feet_ids, _ = self._contact_sensor.find_bodies(".*_foot")
        self._undesired_contact_ids, _ = self._contact_sensor.find_bodies(
            "|".join(PENALIZED_CONTACT_BODIES)
        )

        # Resolve leg joint indices (only the 12 leg joints we control)
        self._leg_joint_ids, _ = self._robot.find_joints(
            "|".join(LEG_JOINT_NAMES)
        )

        # Motor strength randomization buffer
        self._motor_strength = torch.ones(
            self.num_envs, self.cfg.action_space, device=self.device
        )

    # ----- Scene setup -----

    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot)
        self.scene.articulations["robot"] = self._robot

        self._contact_sensor = ContactSensor(self.cfg.contact_sensor)
        self.scene.sensors["contact_sensor"] = self._contact_sensor

        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    # ----- Actions -----

    def _pre_physics_step(self, actions: torch.Tensor):
        self._actions = actions.clone().clamp(-1.0, 1.0)

        # Scale actions and add to default joint positions (only leg joints)
        default_leg_pos = self._robot.data.default_joint_pos[:, self._leg_joint_ids]
        scaled_actions = self.cfg.action_scale * self._actions * self._motor_strength
        self._processed_actions = scaled_actions + default_leg_pos

    def _apply_action(self):
        # Apply position targets only to leg joints
        self._robot.set_joint_position_target(
            self._processed_actions, joint_ids=self._leg_joint_ids
        )

    # ----- Observations -----

    def _get_observations(self) -> dict:
        self._previous_actions = self._actions.clone()

        obs = torch.cat(
            [
                self._robot.data.root_lin_vel_b,                                        # (N, 3)
                self._robot.data.root_ang_vel_b,                                        # (N, 3)
                self._robot.data.projected_gravity_b,                                   # (N, 3)
                self._commands,                                                          # (N, 3)
                self._robot.data.joint_pos[:, self._leg_joint_ids]
                - self._robot.data.default_joint_pos[:, self._leg_joint_ids],           # (N, 12)
                self._robot.data.joint_vel[:, self._leg_joint_ids],                     # (N, 12)
                self._actions,                                                           # (N, 12)
            ],
            dim=-1,
        )
        observations = {"policy": obs}
        return observations

    # ----- Rewards -----

    def _get_rewards(self) -> torch.Tensor:
        dt = self.step_dt

        # -- Tracking rewards (exponential kernel) --

        # Forward/lateral velocity tracking
        lin_vel_error = torch.sum(
            torch.square(
                self._commands[:, :2] - self._robot.data.root_lin_vel_b[:, :2]
            ),
            dim=1,
        )
        track_lin_vel = torch.exp(-lin_vel_error / 0.25) * self.cfg.rew_track_lin_vel_xy * dt

        # Yaw rate tracking
        yaw_rate_error = torch.square(
            self._commands[:, 2] - self._robot.data.root_ang_vel_b[:, 2]
        )
        track_ang_vel = torch.exp(-yaw_rate_error / 0.25) * self.cfg.rew_track_ang_vel_z * dt

        # -- Penalty terms --

        # Vertical velocity penalty
        lin_vel_z = torch.square(self._robot.data.root_lin_vel_b[:, 2])
        rew_lin_vel_z = lin_vel_z * self.cfg.rew_lin_vel_z * dt

        # Roll/pitch angular velocity penalty
        ang_vel_xy = torch.sum(
            torch.square(self._robot.data.root_ang_vel_b[:, :2]), dim=1
        )
        rew_ang_vel_xy = ang_vel_xy * self.cfg.rew_ang_vel_xy * dt

        # Energy penalty: joint torques
        joint_torques = torch.sum(
            torch.square(self._robot.data.applied_torque[:, self._leg_joint_ids]),
            dim=1,
        )
        rew_torques = joint_torques * self.cfg.rew_joint_torques * dt

        # Smoothness penalty: joint accelerations
        joint_accel = torch.sum(
            torch.square(self._robot.data.joint_acc[:, self._leg_joint_ids]),
            dim=1,
        )
        rew_accel = joint_accel * self.cfg.rew_joint_accel * dt

        # Action rate penalty: change between consecutive actions
        action_rate = torch.sum(
            torch.square(self._actions - self._previous_actions), dim=1
        )
        rew_action_rate = action_rate * self.cfg.rew_action_rate * dt

        # Flat orientation penalty: penalize body tilt
        flat_orient = torch.sum(
            torch.square(self._robot.data.projected_gravity_b[:, :2]), dim=1
        )
        rew_flat_orient = flat_orient * self.cfg.rew_flat_orientation * dt

        # Feet air time reward: encourage alternating gait
        first_contact = self._contact_sensor.compute_first_contact(self.step_dt)[
            :, self._feet_ids
        ]
        last_air_time = self._contact_sensor.data.last_air_time[:, self._feet_ids]
        air_time_reward = torch.sum(
            (last_air_time - 0.5) * first_contact, dim=1
        )
        # Only reward air time when moving
        moving = torch.norm(self._commands[:, :2], dim=1) > 0.1
        rew_air_time = air_time_reward * moving.float() * self.cfg.rew_feet_air_time * dt

        # Undesired body contacts penalty
        net_forces = self._contact_sensor.data.net_forces_w_history
        is_contact = (
            torch.max(
                torch.norm(net_forces[:, :, self._undesired_contact_ids], dim=-1),
                dim=1,
            )[0]
            > 1.0
        )
        undesired_contacts = torch.sum(is_contact.float(), dim=1)
        rew_undesired = undesired_contacts * self.cfg.rew_undesired_contacts * dt

        # Joint position limits penalty: penalize joints near limits
        joint_pos = self._robot.data.joint_pos[:, self._leg_joint_ids]
        joint_pos_lower = self._robot.data.joint_pos_limits[:, self._leg_joint_ids, 0]
        joint_pos_upper = self._robot.data.joint_pos_limits[:, self._leg_joint_ids, 1]
        joint_range = joint_pos_upper - joint_pos_lower
        # Fraction of range that is "near limit" (within 10%)
        near_lower = torch.clamp(
            (joint_pos_lower + 0.1 * joint_range - joint_pos) / (0.1 * joint_range),
            min=0.0,
            max=1.0,
        )
        near_upper = torch.clamp(
            (joint_pos - joint_pos_upper + 0.1 * joint_range) / (0.1 * joint_range),
            min=0.0,
            max=1.0,
        )
        rew_limits = (
            torch.sum(near_lower + near_upper, dim=1)
            * self.cfg.rew_joint_pos_limits
            * dt
        )

        # Stumble penalty: foot contact force exceeds threshold while moving fast
        foot_forces = torch.norm(
            self._contact_sensor.data.net_forces_w_history[:, 0, self._feet_ids],
            dim=-1,
        )
        foot_vel = torch.norm(
            self._robot.data.root_lin_vel_b[:, :2], dim=1, keepdim=True
        )
        stumble = torch.sum(
            (foot_forces > 5.0 * self._robot.data.root_link_pos_w[:, 2:3]).float()
            * (foot_vel > 0.5).float(),
            dim=1,
        )
        rew_stumble = stumble * self.cfg.rew_stumble * dt

        # -- Total reward --
        reward = (
            track_lin_vel
            + track_ang_vel
            + rew_lin_vel_z
            + rew_ang_vel_xy
            + rew_torques
            + rew_accel
            + rew_action_rate
            + rew_flat_orient
            + rew_air_time
            + rew_undesired
            + rew_limits
            + rew_stumble
        )

        return reward

    # ----- Termination -----

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1

        # Terminate if body contacts the ground
        net_forces = self._contact_sensor.data.net_forces_w_history
        body_contact = torch.any(
            torch.max(
                torch.norm(net_forces[:, :, self._base_id], dim=-1), dim=1
            )[0]
            > 1.0,
            dim=1,
        )

        # Terminate if body orientation is too extreme (> 60 degrees from upright)
        # projected_gravity_b z-component: 1.0 = upright, 0.0 = 90 degrees
        extreme_orient = self._robot.data.projected_gravity_b[:, 2] > -0.5

        terminated = body_contact | extreme_orient
        return terminated, time_out

    # ----- Reset -----

    def _reset_idx(self, env_ids: torch.Tensor | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES

        self._robot.reset(env_ids)
        super()._reset_idx(env_ids)

        # Stagger episode resets to avoid synchronization artifacts
        if len(env_ids) == self.num_envs:
            self.episode_length_buf[:] = torch.randint_like(
                self.episode_length_buf, high=int(self.max_episode_length)
            )

        # Reset action buffers
        self._actions[env_ids] = 0.0
        self._previous_actions[env_ids] = 0.0

        # Randomize velocity commands
        self._commands[env_ids, 0] = torch.empty(len(env_ids), device=self.device).uniform_(
            *self.cfg.cmd_vx_range
        )
        self._commands[env_ids, 1] = torch.empty(len(env_ids), device=self.device).uniform_(
            *self.cfg.cmd_vy_range
        )
        self._commands[env_ids, 2] = torch.empty(len(env_ids), device=self.device).uniform_(
            *self.cfg.cmd_yaw_range
        )

        # Reset robot state to default standing pose
        joint_pos = self._robot.data.default_joint_pos[env_ids]
        joint_vel = self._robot.data.default_joint_vel[env_ids]
        default_root_state = self._robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self._terrain.env_origins[env_ids]

        self._robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        # --- Domain Randomization ---

        if self.cfg.randomize_motor_strength:
            self._motor_strength[env_ids] = torch.empty(
                len(env_ids), self.cfg.action_space, device=self.device
            ).uniform_(*self.cfg.motor_strength_range)


class CleanWalkerLitterEnv(CleanWalkerLocomotionEnv):
    """Litter collection environment — extends locomotion with arm control.

    Adds stop-and-reach behavior: the robot learns to walk, stop when
    near litter, stabilize, extend its arm, and close the gripper.

    This is what differentiates CleanWalker from standard quadruped
    locomotion training.
    """

    cfg: CleanWalkerLitterEnvCfg

    def __init__(self, cfg: CleanWalkerLitterEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # Arm joint indices
        self._arm_joint_ids, _ = self._robot.find_joints(
            "|".join(ARM_JOINT_NAMES)
        )

        # Expanded action buffer (legs + arm)
        self._actions = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)
        self._previous_actions = torch.zeros_like(self._actions)
        self._preprevious_actions = torch.zeros_like(self._actions)

        # Pick command state
        self._pick_command = torch.zeros(self.num_envs, device=self.device)
        self._pick_command_step = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        # Arm pickup target (pre-computed IK for ground reach)
        self._arm_target = torch.tensor(
            ARM_PICKUP_TARGET, device=self.device
        ).unsqueeze(0).expand(self.num_envs, -1)

        # Previous velocity for deceleration penalty
        self._prev_root_lin_vel_b = torch.zeros(self.num_envs, 3, device=self.device)

        # Motor strength for expanded action space
        self._motor_strength = torch.ones(
            self.num_envs, self.cfg.action_space, device=self.device
        )

    def _pre_physics_step(self, actions: torch.Tensor):
        self._preprevious_actions = self._previous_actions.clone()
        self._actions = actions.clone().clamp(-1.0, 1.0)

        # Leg actions (indices 0:12)
        leg_actions = self._actions[:, :12]
        default_leg_pos = self._robot.data.default_joint_pos[:, self._leg_joint_ids]
        leg_strength = self._motor_strength[:, :12]
        self._processed_leg_actions = self.cfg.action_scale * leg_actions * leg_strength + default_leg_pos

        # Arm actions (indices 12:17) — use smaller action scale for precision
        arm_actions = self._actions[:, 12:17]
        default_arm_pos = self._robot.data.default_joint_pos[:, self._arm_joint_ids]
        arm_strength = self._motor_strength[:, 12:17]
        arm_action_scale = 0.1  # More conservative than legs
        self._processed_arm_actions = arm_action_scale * arm_actions * arm_strength + default_arm_pos

        # During non-pick phases, override arm to default (stay folded)
        no_pick = (self._pick_command == 0.0).unsqueeze(1).expand_as(self._processed_arm_actions)
        self._processed_arm_actions = torch.where(
            no_pick, default_arm_pos, self._processed_arm_actions
        )

        # Activate pick command based on episode step
        current_step = self.episode_length_buf
        should_pick = (current_step >= self._pick_command_step) & (self._pick_command_step > 0)
        self._pick_command = should_pick.float()

    def _apply_action(self):
        # Apply leg actions
        self._robot.set_joint_position_target(
            self._processed_leg_actions, joint_ids=self._leg_joint_ids
        )
        # Apply arm actions
        self._robot.set_joint_position_target(
            self._processed_arm_actions, joint_ids=self._arm_joint_ids
        )

    def _get_observations(self) -> dict:
        self._previous_actions = self._actions.clone()
        self._prev_root_lin_vel_b = self._robot.data.root_lin_vel_b.clone()

        # Base observations (48 dims — same as locomotion)
        base_obs = torch.cat(
            [
                self._robot.data.root_lin_vel_b,
                self._robot.data.root_ang_vel_b,
                self._robot.data.projected_gravity_b,
                self._commands,
                self._robot.data.joint_pos[:, self._leg_joint_ids]
                - self._robot.data.default_joint_pos[:, self._leg_joint_ids],
                self._robot.data.joint_vel[:, self._leg_joint_ids],
                self._actions[:, :12],
            ],
            dim=-1,
        )

        # Arm observations (10 dims — arm pos + arm vel)
        arm_obs = torch.cat(
            [
                self._robot.data.joint_pos[:, self._arm_joint_ids]
                - self._robot.data.default_joint_pos[:, self._arm_joint_ids],
                self._robot.data.joint_vel[:, self._arm_joint_ids],
            ],
            dim=-1,
        )

        obs = torch.cat([base_obs, arm_obs], dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        # Get base locomotion reward
        base_reward = super()._get_rewards()
        dt = self.step_dt

        # --- Litter collection rewards ---

        # Stop and stabilize (when pick command active)
        rew_stop = litter_rewards.reward_stop_and_stabilize(
            root_lin_vel_b=self._robot.data.root_lin_vel_b,
            root_ang_vel_b=self._robot.data.root_ang_vel_b,
            projected_gravity_b=self._robot.data.projected_gravity_b,
            pick_command=self._pick_command,
            dt=dt,
            stop_scale=self.cfg.rew_stop_and_stabilize,
        )

        # Arm reach toward pickup target
        arm_pos = self._robot.data.joint_pos[:, self._arm_joint_ids]
        rew_reach = litter_rewards.reward_arm_reach(
            arm_joint_pos=arm_pos,
            arm_target_pos=self._arm_target,
            pick_command=self._pick_command,
            dt=dt,
            reach_scale=self.cfg.rew_arm_reach,
        )

        # Grasp reward (gripper closing when arm is near target)
        arm_error = torch.sum(torch.square(arm_pos[:, :4] - self._arm_target[:, :4]), dim=1)
        arm_near = arm_error < 0.3  # threshold for "near enough"
        gripper_pos = arm_pos[:, 4]  # gripper joint
        rew_grasp = litter_rewards.reward_grasp(
            gripper_pos=gripper_pos,
            gripper_target=ARM_GRIPPER_CLOSED,
            arm_near_target=arm_near,
            pick_command=self._pick_command,
            dt=dt,
            grasp_scale=self.cfg.rew_grasp,
        )

        # Smooth deceleration penalty
        rew_decel = litter_rewards.reward_smooth_deceleration(
            root_lin_vel_b=self._robot.data.root_lin_vel_b,
            prev_root_lin_vel_b=self._prev_root_lin_vel_b,
            pick_command=self._pick_command,
            dt=dt,
            scale=self.cfg.rew_smooth_deceleration,
        )

        # Smooth motion (jerk penalty) for all phases
        rew_smooth = litter_rewards.reward_smooth_motion(
            actions=self._actions,
            previous_actions=self._previous_actions,
            preprevious_actions=self._preprevious_actions,
            joint_acc=self._robot.data.joint_acc[:, self._leg_joint_ids],
            dt=dt,
            jerk_scale=self.cfg.rew_jerk,
        )

        # Energy efficiency (power penalty)
        rew_energy = litter_rewards.reward_energy_efficiency(
            applied_torque=self._robot.data.applied_torque[:, self._leg_joint_ids],
            joint_vel=self._robot.data.joint_vel[:, self._leg_joint_ids],
            dt=dt,
            power_scale=self.cfg.rew_power,
        )

        return base_reward + rew_stop + rew_reach + rew_grasp + rew_decel + rew_smooth + rew_energy

    def _reset_idx(self, env_ids: torch.Tensor | None):
        super()._reset_idx(env_ids)

        if env_ids is None:
            return

        # Reset arm-related buffers
        self._preprevious_actions[env_ids] = 0.0
        self._prev_root_lin_vel_b[env_ids] = 0.0

        # Randomly assign pick commands to some episodes
        has_pick = torch.rand(len(env_ids), device=self.device) < self.cfg.pick_command_prob
        delay_s = torch.empty(len(env_ids), device=self.device).uniform_(
            *self.cfg.pick_command_delay_range
        )
        delay_steps = (delay_s / self.step_dt).long()

        self._pick_command[env_ids] = 0.0
        self._pick_command_step[env_ids] = torch.where(
            has_pick, delay_steps, torch.zeros_like(delay_steps)
        )
