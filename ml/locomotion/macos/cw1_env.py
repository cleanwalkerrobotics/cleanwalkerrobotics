"""CW-1 Locomotion Gymnasium Environment for MuJoCo.

Custom Gymnasium environment matching our IsaacLab configuration:
- 48-dim observation space (body state + joint state + commands + prev actions)
- 12-dim action space (leg joints only, arm fixed)
- Reward function from ml/locomotion/rewards.py
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

# Default standing joint positions — splayed stance for stability.
# Front legs pitched forward (negative hip_pitch = thigh forward in world X),
# rear legs pitched backward. This spreads the support polygon so the
# forward-heavy body (head + arm) doesn't tip over.
# Verified: robot stands at 0.34m with <1° tilt for 5+ seconds.
DEFAULT_JOINT_POS = np.array([
    0.0, -0.6, 1.24,    # FL: yaw, hip_pitch (forward), knee_pitch
    0.0, -0.6, 1.24,    # FR
    0.0,  0.6, -1.24,   # RL: opposite signs for rear legs
    0.0,  0.6, -1.24,   # RR
], dtype=np.float32)

# Joint limits from URDF (for penalty computation)
JOINT_LOWER = np.array([
    -0.4993, -1.5708, -0.0995,
    -0.4993, -1.5708, -0.0995,
    -0.4993, -1.5708, -2.6005,
    -0.4993, -1.5708, -2.6005,
], dtype=np.float32)

JOINT_UPPER = np.array([
    0.4993, 1.5708, 2.6005,
    0.4993, 1.5708, 2.6005,
    0.4993, 1.5708, 0.0995,
    0.4993, 1.5708, 0.0995,
], dtype=np.float32)


class CW1LocomotionEnv(gym.Env):
    """CW-1 quadruped locomotion environment.

    Observation space (48 dims):
        [0:3]   Base linear velocity (body frame)
        [3:6]   Base angular velocity (body frame)
        [6:9]   Projected gravity (body frame)
        [9:12]  Velocity commands [vx, vy, yaw_rate]
        [12:24] Leg joint positions relative to default (12)
        [24:36] Leg joint velocities (12)
        [36:48] Previous leg actions (12)

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
        action_scale: float = 0.5,
        # Velocity commands (always some forward component)
        cmd_vx_range: tuple = (0.2, 0.8),
        cmd_vy_range: tuple = (-0.2, 0.2),
        cmd_yaw_range: tuple = (-0.5, 0.5),
        # Termination
        min_body_height: float = 0.15,
        max_body_tilt: float = 0.7,   # radians (~40 degrees)
        # Reward weights (velocity-focused: robot can balance, now learn to walk)
        rew_track_lin_vel: float = 2.0,
        rew_track_ang_vel: float = 0.5,
        rew_alive: float = 0.1,
        rew_lin_vel_z: float = -2.0,
        rew_ang_vel_xy: float = -0.05,
        rew_torques: float = -1e-5,
        rew_joint_accel: float = -2.5e-7,
        rew_action_rate: float = -0.01,
        rew_orientation: float = -0.3,
        rew_joint_limits: float = -5.0,
        rew_feet_air_time: float = 1.0,
        rew_base_height: float = 0.5,
        rew_collision: float = -1.0,
        # Tracking
        tracking_sigma: float = 0.25,
        target_base_height: float = 0.35,
        # Domain randomization
        randomize_friction: bool = True,
        friction_range: tuple = (0.5, 1.5),
        randomize_mass: bool = True,
        mass_range: tuple = (-1.0, 3.0),
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

        # Tracking
        self.tracking_sigma = tracking_sigma
        self.target_base_height = target_base_height

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
        }

        # Domain randomization
        self.randomize_friction = randomize_friction
        self.friction_range = friction_range
        self.randomize_mass = randomize_mass
        self.mass_range = mass_range

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

        # Observation and action spaces
        self.n_obs = 48
        self.n_act = 12

        obs_high = np.full(self.n_obs, 100.0, dtype=np.float32)
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(self.n_act,), dtype=np.float32)

        # State tracking
        self._step_count = 0
        self._prev_action = np.zeros(self.n_act, dtype=np.float32)
        self._prev_joint_vel = np.zeros(self.n_act, dtype=np.float32)
        self._velocity_cmd = np.zeros(3, dtype=np.float32)
        self._foot_contact_time = np.zeros(4, dtype=np.float32)
        self._foot_air_time = np.zeros(4, dtype=np.float32)
        self._last_contacts = np.zeros(4, dtype=bool)
        self._first_contact = np.zeros(4, dtype=bool)

        # Original values for domain randomization
        self._default_body_mass = self.model.body_mass.copy()
        self._default_geom_friction = self.model.geom_friction.copy()

        # Renderer
        self._renderer = None
        if render_mode == "human":
            self._setup_viewer()

    def _setup_indices(self):
        """Cache MuJoCo joint and actuator indices."""
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

        # Floor geom
        self.floor_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )

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

        # Reset simulation
        mujoco.mj_resetData(self.model, self.data)

        # Set initial pose: standing position
        # Freejoint qpos: [x, y, z, qw, qx, qy, qz]
        self.data.qpos[0] = 0.0   # x
        self.data.qpos[1] = 0.0   # y
        self.data.qpos[2] = 0.35  # z (standing height)
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
        self._prev_joint_vel = np.zeros(self.n_act, dtype=np.float32)
        self._foot_contact_time = np.zeros(4, dtype=np.float32)
        self._foot_air_time = np.zeros(4, dtype=np.float32)
        self._last_contacts = np.zeros(4, dtype=bool)
        self._first_contact = np.zeros(4, dtype=bool)

        obs = self._get_obs()
        info = {"velocity_cmd": self._velocity_cmd.copy()}

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

        # Update foot contact tracking
        self._update_foot_contacts()

        # Compute observation (clip to prevent extreme values)
        obs = self._get_obs()
        obs = np.clip(obs, -100.0, 100.0)

        # Compute reward (clip to prevent value function explosion)
        reward, reward_info = self._compute_reward(action)
        reward = float(np.clip(reward, -10.0, 10.0))

        # Check termination
        terminated = self._check_termination()
        truncated = self._step_count >= self.max_episode_steps

        # Update history
        joint_vel = self.data.qvel[self.leg_qvel_ids].astype(np.float32)
        self._prev_joint_vel = joint_vel.copy()
        self._prev_action = action.copy()

        info = {
            "velocity_cmd": self._velocity_cmd.copy(),
            "step": self._step_count,
            **reward_info,
        }

        return obs, reward, terminated, truncated, info

    def _get_obs(self) -> np.ndarray:
        """Construct 48-dim observation vector."""
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

        obs = np.concatenate([
            body_linvel.astype(np.float32),       # [0:3]
            body_angvel.astype(np.float32),       # [3:6]
            proj_gravity.astype(np.float32),      # [6:9]
            self._velocity_cmd,                    # [9:12]
            joint_pos_rel,                         # [12:24]
            joint_vel,                             # [24:36]
            self._prev_action,                     # [36:48]
        ])

        return obs.astype(np.float32)

    def _compute_reward(self, action: np.ndarray) -> tuple[float, dict]:
        """Compute reward matching IsaacLab configuration."""
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

        total_reward = 0.0

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

        # 4. Vertical velocity penalty
        r_lin_vel_z = body_linvel[2] ** 2 * w["lin_vel_z"]
        total_reward += r_lin_vel_z
        info["r_lin_vel_z"] = r_lin_vel_z

        # 5. Roll/pitch angular velocity penalty
        r_ang_vel_xy = np.sum(body_angvel[:2] ** 2) * w["ang_vel_xy"]
        total_reward += r_ang_vel_xy
        info["r_ang_vel_xy"] = r_ang_vel_xy

        # 6. Torque penalty (energy efficiency)
        r_torques = np.sum(torques ** 2) * w["torques"]
        total_reward += r_torques
        info["r_torques"] = r_torques

        # 7. Joint acceleration penalty (smoothness)
        joint_accel = (joint_vel - self._prev_joint_vel) / dt
        r_joint_accel = np.sum(joint_accel ** 2) * w["joint_accel"]
        total_reward += r_joint_accel
        info["r_joint_accel"] = r_joint_accel

        # 8. Action rate penalty (smooth actions)
        action_diff = action - self._prev_action
        r_action_rate = np.sum(action_diff ** 2) * w["action_rate"]
        total_reward += r_action_rate
        info["r_action_rate"] = r_action_rate

        # 9. Orientation penalty (keep body level)
        proj_gravity = body_rot.T @ np.array([0, 0, -1.0])
        r_orientation = np.sum(proj_gravity[:2] ** 2) * w["orientation"]
        total_reward += r_orientation
        info["r_orientation"] = r_orientation

        # 10. Joint limit penalty
        margin = 0.1  # radians from limit
        lower_violation = np.maximum(JOINT_LOWER + margin - joint_pos, 0)
        upper_violation = np.maximum(joint_pos - (JOINT_UPPER - margin), 0)
        r_joint_limits = np.sum(lower_violation + upper_violation) * w["joint_limits"]
        total_reward += r_joint_limits
        info["r_joint_limits"] = r_joint_limits

        # 11. Feet air time reward (legged_gym style: reward on first contact)
        # Reward feet for landing after being airborne > threshold.
        # Threshold 0.2s matches the ~0.4-0.8s step cycle at current speed.
        # Only active when robot is commanded to move.
        cmd_speed = np.linalg.norm(self._velocity_cmd[:2])
        r_feet_air = 0.0
        if cmd_speed > 0.1:
            r_feet_air = np.sum(
                (self._foot_air_time - 0.2) * self._first_contact
            ) * w["feet_air_time"]
        total_reward += r_feet_air
        info["r_feet_air_time"] = r_feet_air

        # 12. Base height reward (keep body at target standing height)
        body_height = self.data.xpos[self.body_id][2]
        height_error = (body_height - self.target_base_height) ** 2
        r_base_height = np.exp(-height_error / 0.05) * w["base_height"]
        total_reward += r_base_height
        info["r_base_height"] = r_base_height

        # 13. Collision penalty (non-foot body parts touching ground)
        r_collision = 0.0
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1, geom2 = contact.geom1, contact.geom2
            # Check if floor is involved
            if geom1 != self.floor_geom_id and geom2 != self.floor_geom_id:
                continue
            # Check if it's NOT a foot contact (feet touching floor is OK)
            other_geom = geom2 if geom1 == self.floor_geom_id else geom1
            if other_geom not in self.foot_geom_ids:
                r_collision = w["collision"]
                break
        total_reward += r_collision
        info["r_collision"] = r_collision

        # Scale by dt
        total_reward *= dt

        info["total_reward"] = total_reward
        return float(total_reward), info

    def _update_foot_contacts(self):
        """Track foot contact and air time (legged_gym style).

        Computes first_contact: True when foot touches ground after being airborne.
        This is used for the feet_air_time reward (reward on landing).
        """
        in_contact = np.zeros(4, dtype=bool)

        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1, geom2 = contact.geom1, contact.geom2
            for j, foot_id in enumerate(self.foot_geom_ids):
                if (geom1 == foot_id and geom2 == self.floor_geom_id) or \
                   (geom2 == foot_id and geom1 == self.floor_geom_id):
                    in_contact[j] = True

        # Filtered contact (current or last step)
        contact_filt = np.logical_or(in_contact, self._last_contacts)
        self._last_contacts = in_contact.copy()

        # First contact: foot was airborne and now touching
        self._first_contact = (self._foot_air_time > 0.0) & contact_filt

        # Accumulate air time, reset on contact
        self._foot_air_time += self.control_dt
        self._foot_air_time *= ~contact_filt  # reset to 0 on contact

        # Track contact time for other uses
        for j in range(4):
            if in_contact[j]:
                self._foot_contact_time[j] += self.control_dt
            else:
                self._foot_contact_time[j] = 0.0

    def _check_termination(self) -> bool:
        """Check if episode should terminate (robot fell over)."""
        # Body height check
        body_height = self.data.xpos[self.body_id][2]
        if body_height < self.min_body_height:
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
