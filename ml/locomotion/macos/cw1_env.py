"""CW-1 Locomotion Gymnasium Environment for MuJoCo (v5).

Custom Gymnasium environment with:
- 52 or 56-dim observation space (56 when use_foot_contacts=True)
- 12-dim action space (leg joints only)
- Mammalian knee configuration (all knees backward)
- Heightfield terrain support (flat, rough, obstacles, stairs, slopes)
- WTW-inspired shaped rewards: gait clock, air time, clearance, smoothness
- Configurable: action_scale, only_positive_rewards, foot contacts in obs
- 200Hz simulation, 50Hz control (decimation=4)
- Episode: 1000 steps (20s), terminates on fall

Usage:
    import gymnasium as gym
    from cw1_env import CW1LocomotionEnv

    env = CW1LocomotionEnv()
    obs, info = env.reset()
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
"""

import math
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

SCRIPT_DIR = Path(__file__).parent.resolve()
DEFAULT_MJCF = SCRIPT_DIR / "cw1_scene.xml"

# Joint ordering (must match MJCF actuator order)
LEG_JOINT_NAMES = [
    "FL_hip_yaw", "FL_hip_pitch", "FL_knee_pitch",
    "FR_hip_yaw", "FR_hip_pitch", "FR_knee_pitch",
    "RL_hip_yaw", "RL_hip_pitch", "RL_knee_pitch",
    "RR_hip_yaw", "RR_hip_pitch", "RR_knee_pitch",
]

# Default standing joint positions — mammalian configuration.
# All four legs identical: hip_pitch=-0.6 (thigh tilted forward),
# knee_pitch=1.2 (all knees bent backward ~69 degrees).
# hip_pitch = -knee/2 gives zero net foot forward displacement.
# Standing height: 2 * 0.2 * cos(0.6) + 0.025 = 0.355m ~ 0.35m
DEFAULT_JOINT_POS = np.array([
    0.0, -0.6, 1.2,    # FL: yaw, hip_pitch, knee_pitch
    0.0, -0.6, 1.2,    # FR
    0.0, -0.6, 1.2,    # RL (same as front — mammalian)
    0.0, -0.6, 1.2,    # RR
], dtype=np.float32)

# Joint limits — all knees now have same range (mammalian)
JOINT_LOWER = np.array([
    -0.4993, -1.5708, -0.0995,
    -0.4993, -1.5708, -0.0995,
    -0.4993, -1.5708, -0.0995,
    -0.4993, -1.5708, -0.0995,
], dtype=np.float32)

JOINT_UPPER = np.array([
    0.4993, 1.5708, 2.6005,
    0.4993, 1.5708, 2.6005,
    0.4993, 1.5708, 2.6005,
    0.4993, 1.5708, 2.6005,
], dtype=np.float32)


class CW1LocomotionEnv(gym.Env):
    """CW-1 quadruped locomotion environment (v5: configurable obs + rewards).

    Observation space (52 or 56 dims):
        [0:3]   Base linear velocity (body frame)
        [3:6]   Base angular velocity (body frame)
        [6:9]   Projected gravity (body frame)
        [9:12]  Velocity commands [vx, vy, yaw_rate]
        [12:24] Leg joint positions relative to default (12)
        [24:36] Leg joint velocities (12)
        [36:48] Previous leg actions (12)
        [48:52] Gait clock [sin(FL/RR), cos(FL/RR), sin(FR/RL), cos(FR/RL)]
        [52:56] Foot contacts [FL, FR, RL, RR] (optional, binary: 1=contact, 0=air)

    Action space (12 dims):
        Position targets for 12 leg joints, scaled by action_scale.
        Final target = default_pos + action * action_scale
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(
        self,
        mjcf_path: str | Path = DEFAULT_MJCF,
        render_mode: str | None = None,
        # Simulation
        sim_dt: float = 0.005,        # 200 Hz physics
        control_dt: float = 0.02,     # 50 Hz control (decimation=4)
        max_episode_steps: int = 1000,  # 20 seconds
        # Action
        action_scale: float = 0.330,
        # Velocity commands (always forward — no standing command)
        cmd_vx_range: tuple = (0.0, 0.7),
        cmd_vy_range: tuple = (-0.1, 0.1),
        cmd_yaw_range: tuple = (-0.3, 0.3),
        # Termination
        min_body_height: float = 0.15,
        max_body_tilt: float = 0.7,   # radians (~40 degrees)
        # Reward weights — HPO v3 Trial #35 (stable 0.66 score, no collapse)
        rew_track_lin_vel: float = 5.587,
        rew_track_ang_vel: float = 0.5,
        rew_alive: float = 0.366,
        rew_lin_vel_z: float = -2.0,
        rew_ang_vel_xy: float = -0.05,
        rew_torques: float = -1e-5,
        rew_joint_accel: float = -2.5e-7,
        rew_action_rate: float = -0.01,
        rew_orientation: float = -0.208,
        rew_joint_limits: float = -5.0,
        rew_feet_air_time: float = 1.0,
        rew_base_height: float = 0.5,
        rew_collision: float = -1.0,
        fall_penalty: float = 3.502,
        rew_gait_clock: float = 1.018,
        # Shaped gait rewards — HPO v3 Trial #35 values
        rew_gait_force: float = 2.835,
        rew_gait_vel: float = 1.004,
        rew_clearance_target: float = -24.765,
        rew_action_smooth_1: float = -0.454,
        rew_action_smooth_2: float = -0.083,
        rew_foot_slip: float = -0.096,
        rew_air_time_var: float = -0.836,
        rew_hip_vel: float = 0.0,
        # Reward clipping
        only_positive_rewards: bool = False,
        # Observation options
        use_foot_contacts: bool = True,
        # Tracking
        tracking_sigma: float = 0.271,
        # Gait clock (trot pattern)
        gait_period: float = 0.458,
        swing_height_target: float = 0.111,
        feet_air_time_threshold: float = 0.186,  # 0.813 * 0.458 / 2
        target_base_height: float = 0.35,
        # Domain randomization
        randomize_friction: bool = True,
        friction_range: tuple = (0.5, 1.5),
        randomize_mass: bool = True,
        mass_range: tuple = (-1.0, 3.0),
        # Terrain
        terrain_types: list[str] | None = None,
    ):
        super().__init__()

        self.mjcf_path = Path(mjcf_path)
        self.render_mode = render_mode

        # Timing
        self.sim_dt = sim_dt
        self.control_dt = control_dt
        self.decimation = int(control_dt / sim_dt)
        self.max_episode_steps = max_episode_steps

        # Action scaling
        self.action_scale = action_scale

        # Command ranges
        self.cmd_vx_range = cmd_vx_range
        self.cmd_vy_range = cmd_vy_range
        self.cmd_yaw_range = cmd_yaw_range

        # Termination thresholds
        self.min_body_height = min_body_height
        self.max_body_tilt = max_body_tilt

        # Tracking + penalties
        self.fall_penalty = fall_penalty
        self.only_positive_rewards = only_positive_rewards
        self.use_foot_contacts = use_foot_contacts
        self.tracking_sigma = tracking_sigma
        self.target_base_height = target_base_height
        self.swing_height_target = swing_height_target
        self.feet_air_time_threshold = feet_air_time_threshold

        # Reward weights
        self.reward_weights = {
            "track_lin_vel": rew_track_lin_vel,
            "track_ang_vel": rew_track_ang_vel,
            "alive": rew_alive,
            "lin_vel_z": rew_lin_vel_z,
            "ang_vel_xy": rew_ang_vel_xy,
            "torques": rew_torques,
            "joint_accel": rew_joint_accel,
            "action_rate": rew_action_rate,
            "orientation": rew_orientation,
            "joint_limits": rew_joint_limits,
            "feet_air_time": rew_feet_air_time,
            "base_height": rew_base_height,
            "collision": rew_collision,
            "gait_clock": rew_gait_clock,
            "gait_force": rew_gait_force,
            "gait_vel": rew_gait_vel,
            "clearance_target": rew_clearance_target,
            "action_smooth_1": rew_action_smooth_1,
            "action_smooth_2": rew_action_smooth_2,
            "foot_slip": rew_foot_slip,
            "air_time_var": rew_air_time_var,
            "hip_vel": rew_hip_vel,
        }

        # Gait clock params
        self.gait_period = gait_period
        # Trot gait phase offsets: FL=0, FR=π, RL=π, RR=0 (diagonal pairs)
        self._gait_phase_offsets = np.array([0.0, math.pi, math.pi, 0.0])

        # Domain randomization
        self.randomize_friction = randomize_friction
        self.friction_range = friction_range
        self.randomize_mass = randomize_mass
        self.mass_range = mass_range

        # Terrain
        self.terrain_types = terrain_types or ["flat"]
        self._terrain_gen = None
        self._terrain_height_at_robot = 0.0

        # Load MuJoCo model
        if not self.mjcf_path.exists():
            raise FileNotFoundError(
                f"MJCF not found: {self.mjcf_path}\n"
                "Run convert_urdf_to_mjcf.py first."
            )

        self.model = mujoco.MjModel.from_xml_path(str(self.mjcf_path))
        self.model.opt.timestep = self.sim_dt
        self.data = mujoco.MjData(self.model)

        # Cache joint/actuator indices
        self._setup_indices()

        # Initialize terrain generator
        self._init_terrain()

        # Observation and action spaces
        self.n_obs = 56 if use_foot_contacts else 52
        self.n_act = 12

        obs_high = np.full(self.n_obs, 100.0, dtype=np.float32)
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(self.n_act,), dtype=np.float32)

        # State tracking
        self._step_count = 0
        self._prev_action = np.zeros(self.n_act, dtype=np.float32)
        self._prev_prev_action = np.zeros(self.n_act, dtype=np.float32)  # for 2nd order smoothness
        self._prev_joint_vel = np.zeros(self.n_act, dtype=np.float32)
        self._velocity_cmd = np.zeros(3, dtype=np.float32)
        self._foot_contact_time = np.zeros(4, dtype=np.float32)
        self._foot_air_time = np.zeros(4, dtype=np.float32)
        self._last_contacts = np.zeros(4, dtype=bool)
        self._first_contact = np.zeros(4, dtype=bool)
        self._gait_phase = 0.0  # current gait clock phase [0, 2π)
        self._cached_contacts = np.zeros(4, dtype=bool)  # per-step cache

        # Original values for domain randomization
        self._default_body_mass = self.model.body_mass.copy()
        self._default_geom_friction = self.model.geom_friction.copy()

        # Renderer
        self._renderer = None
        if render_mode == "human":
            self._setup_viewer()

    def _setup_indices(self):
        """Cache MuJoCo joint, actuator, and foot body indices."""
        self.leg_joint_ids = []
        self.leg_qpos_ids = []
        self.leg_qvel_ids = []

        for name in LEG_JOINT_NAMES:
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jnt_id == -1:
                raise ValueError(f"Joint '{name}' not found in MJCF")
            self.leg_joint_ids.append(jnt_id)
            self.leg_qpos_ids.append(self.model.jnt_qposadr[jnt_id])
            self.leg_qvel_ids.append(self.model.jnt_dofadr[jnt_id])

        self.leg_qpos_ids = np.array(self.leg_qpos_ids)
        self.leg_qvel_ids = np.array(self.leg_qvel_ids)

        # Body index for the main body (freejoint)
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "body")

        # Actuator indices (should be 0-11 for leg actuators)
        self.leg_actuator_ids = []
        for name in LEG_JOINT_NAMES:
            act_name = f"{name}_act"
            act_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name
            )
            if act_id == -1:
                raise ValueError(f"Actuator '{act_name}' not found in MJCF")
            self.leg_actuator_ids.append(act_id)
        self.leg_actuator_ids = np.array(self.leg_actuator_ids)

        # Foot geom indices for contact detection
        self.foot_geom_ids = []
        for prefix in ["FL", "FR", "RL", "RR"]:
            geom_name = f"{prefix}_foot_geom"
            geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            self.foot_geom_ids.append(geom_id)
        self.foot_geom_ids = np.array(self.foot_geom_ids)

        # Foot body indices for clearance reward
        self.foot_body_ids = []
        for prefix in ["FL", "FR", "RL", "RR"]:
            body_name = f"{prefix}_foot"
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
            self.foot_body_ids.append(body_id)
        self.foot_body_ids = np.array(self.foot_body_ids)

        # Floor geom
        self.floor_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )

    def _init_terrain(self):
        """Initialize terrain generator and set initial flat terrain."""
        from terrain import TerrainGenerator
        self._terrain_gen = TerrainGenerator()
        # Start with flat terrain so the model loads cleanly
        hfield_data = self._terrain_gen.generate("flat")
        self.model.hfield_data[:] = (hfield_data * 65535).astype(np.uint16)

    def _setup_viewer(self):
        """Setup MuJoCo viewer for human rendering."""
        try:
            import mujoco.viewer
            self._viewer_available = True
        except ImportError:
            self._viewer_available = False
            print("Warning: mujoco.viewer not available. Install mujoco for viewer.")

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Generate terrain for this episode
        terrain_type = self.np_random.choice(self.terrain_types)
        rng = np.random.default_rng(self.np_random.integers(0, 2**31))
        self._terrain_gen.rng = rng
        hfield_data = self._terrain_gen.generate(terrain_type)
        self.model.hfield_data[:] = (hfield_data * 65535).astype(np.uint16)

        # Reset simulation
        mujoco.mj_resetData(self.model, self.data)

        # Recompute derived quantities after hfield change
        mujoco.mj_setConst(self.model, self.data)

        # Get terrain height at spawn point (center of hfield)
        self._terrain_height_at_robot = self._terrain_gen.get_height_at(0.0, 0.0)
        spawn_z = self._terrain_height_at_robot + self.target_base_height

        # Set initial pose: standing position
        self.data.qpos[0] = 0.0   # x
        self.data.qpos[1] = 0.0   # y
        self.data.qpos[2] = spawn_z  # z (terrain + standing height)
        self.data.qpos[3] = 1.0   # qw (identity quaternion)
        self.data.qpos[4] = 0.0   # qx
        self.data.qpos[5] = 0.0   # qy
        self.data.qpos[6] = 0.0   # qz

        # Set default leg joint positions
        for i, qpos_id in enumerate(self.leg_qpos_ids):
            self.data.qpos[qpos_id] = DEFAULT_JOINT_POS[i]

        # Randomize velocity command
        self._velocity_cmd = np.array([
            self.np_random.uniform(*self.cmd_vx_range),
            self.np_random.uniform(*self.cmd_vy_range),
            self.np_random.uniform(*self.cmd_yaw_range),
        ], dtype=np.float32)

        # Domain randomization (reset to defaults first)
        if self.randomize_friction:
            self.model.geom_friction[:] = self._default_geom_friction
            scale = self.np_random.uniform(*self.friction_range)
            self.model.geom_friction[:, 0] *= scale

        if self.randomize_mass:
            self.model.body_mass[:] = self._default_body_mass
            delta = self.np_random.uniform(*self.mass_range)
            self.model.body_mass[self.body_id] += delta

        # Step once to settle
        mujoco.mj_forward(self.model, self.data)

        # Reset state
        self._step_count = 0
        self._prev_action = np.zeros(self.n_act, dtype=np.float32)
        self._prev_prev_action = np.zeros(self.n_act, dtype=np.float32)
        self._prev_joint_vel = np.zeros(self.n_act, dtype=np.float32)
        # NOTE: _velocity_cmd already randomized above (lines 371-375) — do NOT zero it here
        self._foot_contact_time = np.zeros(4, dtype=np.float32)
        self._foot_air_time = np.zeros(4, dtype=np.float32)
        self._last_contacts = np.zeros(4, dtype=bool)
        self._first_contact = np.zeros(4, dtype=bool)
        self._gait_phase = 0.0

        obs = self._get_obs()
        info = {
            "velocity_cmd": self._velocity_cmd.copy(),
            "terrain_type": terrain_type,
        }

        return obs, info

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # Compute target joint positions: default + action * scale
        target_pos = DEFAULT_JOINT_POS + action * self.action_scale

        # Apply control for decimation steps (4 physics steps per control step)
        for _ in range(self.decimation):
            self.data.ctrl[self.leg_actuator_ids] = target_pos
            mujoco.mj_step(self.model, self.data)

        self._step_count += 1

        # NaN guard — terminate immediately if simulation went unstable
        if not np.isfinite(self.data.qpos).all() or \
           not np.isfinite(self.data.qvel).all():
            obs = np.zeros(self.n_obs, dtype=np.float32)
            self._prev_action = action.copy()
            return obs, 0.0, True, False, {"unstable": True}

        # Advance gait clock
        self._gait_phase += 2 * math.pi * self.control_dt / self.gait_period
        self._gait_phase %= (2 * math.pi)

        # Update terrain height at robot position
        robot_x = self.data.xpos[self.body_id][0]
        robot_y = self.data.xpos[self.body_id][1]
        self._terrain_height_at_robot = self._terrain_gen.get_height_at(robot_x, robot_y)

        # Detect foot contacts ONCE per step (reused by obs, reward, update)
        self._detect_foot_contacts()

        # Update foot contact tracking
        self._update_foot_contacts()

        # Compute observation
        obs = self._get_obs()
        np.clip(obs, -100.0, 100.0, out=obs)

        # Compute reward (clip to prevent value function explosion)
        reward, reward_info = self._compute_reward(action)
        if self.only_positive_rewards:
            reward = max(0.0, reward)
        reward = float(np.clip(reward, -10.0, 10.0))

        # Check termination
        terminated = self._check_termination()
        truncated = self._step_count >= self.max_episode_steps

        # Fall penalty: one-time cost for crashing (makes survival critical)
        if terminated:
            reward -= self.fall_penalty

        # Update history
        joint_vel = self.data.qvel[self.leg_qvel_ids].astype(np.float32)
        self._prev_joint_vel = joint_vel.copy()
        self._prev_prev_action = self._prev_action.copy()
        self._prev_action = action.copy()

        info = {
            "velocity_cmd": self._velocity_cmd.copy(),
            "step": self._step_count,
            **reward_info,
        }

        return obs, reward, terminated, truncated, info

    def _get_obs(self) -> np.ndarray:
        """Construct observation vector (52 or 56 dims depending on use_foot_contacts)."""
        # Body orientation quaternion [w, x, y, z]
        body_quat = self.data.xquat[self.body_id].copy()

        # Body velocities in body frame
        body_linvel_world = self.data.cvel[self.body_id][3:6]  # linear
        body_angvel_world = self.data.cvel[self.body_id][0:3]  # angular

        # Rotation matrix (world to body)
        body_rot = np.zeros(9)
        mujoco.mju_quat2Mat(body_rot, body_quat)
        body_rot = body_rot.reshape(3, 3)

        # Transform to body frame
        body_linvel = body_rot.T @ body_linvel_world
        body_angvel = body_rot.T @ body_angvel_world

        # Projected gravity in body frame
        gravity_world = np.array([0, 0, -1.0])
        proj_gravity = body_rot.T @ gravity_world

        # Joint states
        joint_pos = self.data.qpos[self.leg_qpos_ids].astype(np.float32)
        joint_vel = self.data.qvel[self.leg_qvel_ids].astype(np.float32)

        # Relative joint positions (offset from default)
        joint_pos_rel = joint_pos - DEFAULT_JOINT_POS

        # Gait clock signals: sin/cos for each diagonal pair
        # This tells the policy which feet should be in stance vs swing
        phase_fl_rr = self._gait_phase + self._gait_phase_offsets[0]  # FL/RR pair
        phase_fr_rl = self._gait_phase + self._gait_phase_offsets[1]  # FR/RL pair
        gait_clock = np.array([
            math.sin(phase_fl_rr), math.cos(phase_fl_rr),
            math.sin(phase_fr_rl), math.cos(phase_fr_rl),
        ], dtype=np.float32)

        parts = [
            body_linvel.astype(np.float32),       # [0:3]
            body_angvel.astype(np.float32),       # [3:6]
            proj_gravity.astype(np.float32),      # [6:9]
            self._velocity_cmd,                    # [9:12]
            joint_pos_rel,                         # [12:24]
            joint_vel,                             # [24:36]
            self._prev_action,                     # [36:48]
            gait_clock,                            # [48:52]
        ]

        if self.use_foot_contacts:
            parts.append(self._cached_contacts.astype(np.float32))  # [52:56]

        obs = np.concatenate(parts)

        return obs.astype(np.float32)

    def _compute_reward(self, action: np.ndarray) -> tuple[float, dict]:
        """Compute reward with WTW-style shaped gait rewards.

        Key changes from Run 7f:
        - Shaped gait force/velocity rewards replace binary gait clock
        - Phase-based foot clearance TARGET (penalizes deviation from swing height)
        - Action smoothness 1st + 2nd order (anti-stiff/jerky)
        - Foot slip penalty (anti-sliding in stance)
        - Air time variance penalty (even gait)
        - Hip yaw velocity penalty (anti-wobble)
        """
        w = self.reward_weights
        dt = self.control_dt
        info = {}

        # Body state
        body_quat = self.data.xquat[self.body_id]
        body_rot = np.zeros(9)
        mujoco.mju_quat2Mat(body_rot, body_quat)
        body_rot = body_rot.reshape(3, 3)

        body_linvel_world = self.data.cvel[self.body_id][3:6]
        body_angvel_world = self.data.cvel[self.body_id][0:3]
        body_linvel = body_rot.T @ body_linvel_world
        body_angvel = body_rot.T @ body_angvel_world

        # Joint state
        joint_vel = self.data.qvel[self.leg_qvel_ids]
        joint_pos = self.data.qpos[self.leg_qpos_ids]
        torques = self.data.actuator_force[self.leg_actuator_ids]

        # Foot contact (already detected, use cache)
        in_contact = self._cached_contacts

        cmd_speed = np.sqrt(self._velocity_cmd[0]**2 + self._velocity_cmd[1]**2)

        total_reward = 0.0

        # ===== TRACKING REWARDS =====

        # 1. Forward velocity tracking (exponential kernel)
        lin_vel_error = np.sum((body_linvel[:2] - self._velocity_cmd[:2]) ** 2)
        r_track_lin = np.exp(-lin_vel_error / self.tracking_sigma) * w["track_lin_vel"]
        total_reward += r_track_lin
        info["r_track_lin_vel"] = r_track_lin

        # 2. Angular velocity tracking
        ang_vel_error = (body_angvel[2] - self._velocity_cmd[2]) ** 2
        r_track_ang = np.exp(-ang_vel_error / self.tracking_sigma) * w["track_ang_vel"]
        total_reward += r_track_ang
        info["r_track_ang_vel"] = r_track_ang

        # 3. Alive bonus
        r_alive = w["alive"]
        total_reward += r_alive
        info["r_alive"] = r_alive

        # ===== STABILITY PENALTIES =====

        # 4. Vertical velocity penalty
        r_lin_vel_z = body_linvel[2] ** 2 * w["lin_vel_z"]
        total_reward += r_lin_vel_z
        info["r_lin_vel_z"] = r_lin_vel_z

        # 5. Roll/pitch angular velocity penalty
        r_ang_vel_xy = np.sum(body_angvel[:2] ** 2) * w["ang_vel_xy"]
        total_reward += r_ang_vel_xy
        info["r_ang_vel_xy"] = r_ang_vel_xy

        # 6. Orientation penalty (keep body level)
        proj_gravity = body_rot.T @ np.array([0, 0, -1.0])
        r_orientation = np.sum(proj_gravity[:2] ** 2) * w["orientation"]
        total_reward += r_orientation
        info["r_orientation"] = r_orientation

        # 7. Base height reward (terrain-relative)
        body_height = self.data.xpos[self.body_id][2]
        relative_height = body_height - self._terrain_height_at_robot
        height_error = (relative_height - self.target_base_height) ** 2
        r_base_height = np.exp(-height_error / 0.05) * w["base_height"]
        total_reward += r_base_height
        info["r_base_height"] = r_base_height

        # ===== ENERGY / SMOOTHNESS PENALTIES =====

        # 8. Torque penalty (10x stronger than Run 7f)
        r_torques = np.sum(torques ** 2) * w["torques"]
        total_reward += r_torques
        info["r_torques"] = r_torques

        # 9. Joint acceleration penalty
        joint_accel = (joint_vel - self._prev_joint_vel) / dt
        r_joint_accel = np.sum(joint_accel ** 2) * w["joint_accel"]
        total_reward += r_joint_accel
        info["r_joint_accel"] = r_joint_accel

        # 10. Action rate penalty (legacy)
        action_diff = action - self._prev_action
        r_action_rate = np.sum(action_diff ** 2) * w["action_rate"]
        total_reward += r_action_rate
        info["r_action_rate"] = r_action_rate

        # 11. Action smoothness 1st order (position target difference)
        r_smooth_1 = np.sum(np.abs(action - self._prev_action)) * w["action_smooth_1"]
        total_reward += r_smooth_1
        info["r_action_smooth_1"] = r_smooth_1

        # 12. Action smoothness 2nd order (acceleration of position targets)
        action_accel = action - 2 * self._prev_action + self._prev_prev_action
        r_smooth_2 = np.sum(np.abs(action_accel)) * w["action_smooth_2"]
        total_reward += r_smooth_2
        info["r_action_smooth_2"] = r_smooth_2

        # 13. Hip yaw velocity penalty (anti-wobble, only hip_yaw joints: indices 0,3,6,9)
        hip_yaw_vel = joint_vel[np.array([0, 3, 6, 9])]
        r_hip_vel = np.sum(hip_yaw_vel ** 2) * w["hip_vel"]
        total_reward += r_hip_vel
        info["r_hip_vel"] = r_hip_vel

        # 14. Joint limit penalty
        margin = 0.1  # radians from limit
        lower_violation = np.maximum(JOINT_LOWER + margin - joint_pos, 0)
        upper_violation = np.maximum(joint_pos - (JOINT_UPPER - margin), 0)
        r_joint_limits = np.sum(lower_violation + upper_violation) * w["joint_limits"]
        total_reward += r_joint_limits
        info["r_joint_limits"] = r_joint_limits

        # ===== GAIT SHAPING REWARDS (WTW-inspired) =====

        # 15. Feet air time reward (legged_gym style: reward on first contact)
        r_feet_air = 0.0
        if cmd_speed > 0.1:
            r_feet_air = np.sum(
                (self._foot_air_time - self.feet_air_time_threshold) * self._first_contact
            ) * w["feet_air_time"]
        total_reward += r_feet_air
        info["r_feet_air_time"] = r_feet_air

        # === VECTORIZED GAIT REWARDS (all 4 feet at once) ===
        # Precompute phases for all 4 feet
        foot_phases = self._gait_phase + self._gait_phase_offsets  # shape (4,)
        sin_phases = np.sin(foot_phases)  # >0 = stance, <0 = swing
        desired_stance = sin_phases > 0  # shape (4,) bool
        swing_signal = np.maximum(-sin_phases, 0.0)  # >0 during swing

        # Precompute foot positions and velocities (all 4 at once)
        foot_xpos = self.data.xpos[self.foot_body_ids]  # shape (4, 3)
        foot_cvel = self.data.cvel[self.foot_body_ids]  # shape (4, 6)
        foot_linvel = foot_cvel[:, 3:6]  # shape (4, 3)

        # 16. Shaped gait force reward — stance feet should have ground reaction force
        r_gait_force = 0.0
        if cmd_speed > 0.1 and w["gait_force"] != 0.0:
            # Use contact force: for stance feet that are in contact
            stance_contact = desired_stance & in_contact
            foot_forces = np.zeros(4)
            ncon = self.data.ncon
            if ncon > 0 and np.any(stance_contact):
                g1 = self.data.contact.geom1[:ncon]
                g2 = self.data.contact.geom2[:ncon]
                c_force = np.zeros(6)
                for j in np.where(stance_contact)[0]:
                    fid = self.foot_geom_ids[j]
                    mask = ((g1 == fid) & (g2 == self.floor_geom_id)) | \
                           ((g2 == fid) & (g1 == self.floor_geom_id))
                    ci = np.argmax(mask)
                    if mask[ci]:
                        mujoco.mj_contactForce(self.model, self.data, int(ci), c_force)
                        foot_forces[j] = np.sqrt(c_force[0]**2 + c_force[1]**2 + c_force[2]**2)
            r_gait_force = np.sum(1.0 - np.exp(-foot_forces[desired_stance] / 100.0))
            r_gait_force *= w["gait_force"] / 4.0
        total_reward += r_gait_force
        info["r_gait_force"] = r_gait_force

        # 17. Shaped gait velocity reward — swing feet should be moving
        r_gait_vel = 0.0
        if cmd_speed > 0.1 and w["gait_vel"] != 0.0:
            desired_swing = ~desired_stance
            foot_speeds = np.sqrt(np.sum(foot_linvel[desired_swing] ** 2, axis=1))
            r_gait_vel = np.sum(1.0 - np.exp(-foot_speeds / 10.0))
            r_gait_vel *= w["gait_vel"] / 4.0
        total_reward += r_gait_vel
        info["r_gait_vel"] = r_gait_vel

        # 18. Binary gait clock reward
        r_gait_clock = 0.0
        if cmd_speed > 0.1:
            matches = (desired_stance == in_contact)
            r_gait_clock = np.sum(matches) * 0.25 * w["gait_clock"]
        total_reward += r_gait_clock
        info["r_gait_clock"] = r_gait_clock

        # 19. Phase-based foot clearance TARGET
        r_clearance = 0.0
        if cmd_speed > 0.1 and w["clearance_target"] != 0.0:
            swing_mask = swing_signal > 0
            if np.any(swing_mask):
                swing_feet_xy = foot_xpos[swing_mask, :2]
                swing_feet_z = foot_xpos[swing_mask, 2]
                terrain_z = self._terrain_gen.get_heights_at(swing_feet_xy)
                clearance = swing_feet_z - terrain_z
                targets = self.swing_height_target * swing_signal[swing_mask]
                clearance_errors = np.abs(clearance - targets)
                r_clearance = np.sum(clearance_errors * swing_signal[swing_mask])
                r_clearance *= w["clearance_target"] / 4.0
        total_reward += r_clearance
        info["r_clearance_target"] = r_clearance

        # 20. Foot slip penalty — penalize foot XY velocity during stance contact
        r_slip = 0.0
        if cmd_speed > 0.1 and w["foot_slip"] != 0.0:
            contact_mask = in_contact
            if np.any(contact_mask):
                xy_speed_sq = np.sum(foot_linvel[contact_mask, :2] ** 2, axis=1)
                r_slip = np.sum(xy_speed_sq) * w["foot_slip"]
        total_reward += r_slip
        info["r_foot_slip"] = r_slip

        # 20. Air time variance penalty — all feet should have similar air time
        r_air_var = 0.0
        if cmd_speed > 0.1 and np.any(self._first_contact):
            air_times = self._foot_air_time.copy()
            # Only consider feet that have data (air_time > 0 or just landed)
            if np.sum(air_times > 0) >= 2:
                r_air_var = np.var(air_times[air_times > 0]) * w["air_time_var"]
        total_reward += r_air_var
        info["r_air_time_var"] = r_air_var

        # 21. Collision penalty (vectorized: count non-foot ground contacts, cap at 4)
        collision_count = 0
        ncon = self.data.ncon
        if ncon > 0:
            g1 = self.data.contact.geom1[:ncon]
            g2 = self.data.contact.geom2[:ncon]
            floor_id = self.floor_geom_id
            has_floor = (g1 == floor_id) | (g2 == floor_id)
            other = np.where(g1 == floor_id, g2, g1)
            is_foot = np.isin(other, self.foot_geom_ids)
            collision_count = min(int(np.sum(has_floor & ~is_foot)), 4)
        r_collision = collision_count * w["collision"]
        total_reward += r_collision
        info["r_collision"] = r_collision

        # Scale by dt
        total_reward *= dt

        info["total_reward"] = total_reward
        return float(total_reward), info

    def _detect_foot_contacts(self) -> np.ndarray:
        """Vectorized foot contact detection. Returns bool array of shape (4,)."""
        ncon = self.data.ncon
        if ncon == 0:
            self._cached_contacts[:] = False
            return self._cached_contacts

        g1 = self.data.contact.geom1[:ncon]
        g2 = self.data.contact.geom2[:ncon]
        floor_id = self.floor_geom_id

        # Broadcasting: (4, ncon) matches for each foot
        foot_ids = self.foot_geom_ids  # shape (4,)
        match = ((g1[None, :] == foot_ids[:, None]) & (g2[None, :] == floor_id)) | \
                ((g2[None, :] == foot_ids[:, None]) & (g1[None, :] == floor_id))
        self._cached_contacts = np.any(match, axis=1)
        return self._cached_contacts

    def _update_foot_contacts(self):
        """Track foot contact and air time (legged_gym style)."""
        in_contact = self._cached_contacts  # already computed in step()

        # Filtered contact (current or last step)
        contact_filt = in_contact | self._last_contacts
        self._last_contacts = in_contact.copy()

        # First contact: foot was airborne and now touching
        self._first_contact = (self._foot_air_time > 0.0) & contact_filt

        # Accumulate air time, reset on contact
        self._foot_air_time += self.control_dt
        self._foot_air_time *= ~contact_filt  # reset to 0 on contact

        # Track contact time (vectorized)
        self._foot_contact_time = np.where(
            in_contact, self._foot_contact_time + self.control_dt, 0.0
        )

    def _check_termination(self) -> bool:
        """Check if episode should terminate (terrain-relative)."""
        # Body height check (relative to terrain)
        body_height = self.data.xpos[self.body_id][2]
        relative_height = body_height - self._terrain_height_at_robot
        if relative_height < self.min_body_height:
            return True

        # Body tilt check (angle from upright)
        body_quat = self.data.xquat[self.body_id]
        body_rot = np.zeros(9)
        mujoco.mju_quat2Mat(body_rot, body_quat)
        body_rot = body_rot.reshape(3, 3)
        up_in_body = body_rot.T @ np.array([0, 0, 1.0])
        tilt_angle = np.arccos(np.clip(up_in_body[2], -1, 1))
        if tilt_angle > self.max_body_tilt:
            return True

        return False

    def render(self):
        """Render a frame."""
        if self.render_mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, 480, 640)
            self._renderer.update_scene(self.data)
            return self._renderer.render()
        return None

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


# Register with Gymnasium
gym.register(
    id="CW1-Locomotion-v0",
    entry_point="cw1_env:CW1LocomotionEnv",
    max_episode_steps=1000,
)
