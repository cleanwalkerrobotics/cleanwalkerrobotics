# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""
CleanWalker CW-1 — Litter collection reward functions.

Extends the base locomotion reward with behaviors specific to
autonomous litter collection:

  1. Walking stability (minimize body roll/pitch)
  2. Forward velocity tracking
  3. Energy consumption penalty
  4. Smooth joint movements (minimize jerk)
  5. Stop-and-reach behavior (stop, extend arm, pick up litter)

The stop-and-reach reward is what differentiates CleanWalker from
standard quadruped locomotion — the robot must learn to:
  - Walk toward litter (velocity tracking)
  - Stop smoothly when near litter (deceleration reward)
  - Stabilize body for arm operation (ultra-low velocity + flat orientation)
  - Extend arm toward ground target (arm joint coordination)
  - Close gripper at correct position (grasp reward)

Training curriculum:
  Phase 1: Flat terrain locomotion (base rewards only)
  Phase 2: Rough terrain locomotion (base rewards, harder terrain)
  Phase 3: Litter collection (this module — locomotion + arm control)

Usage:
    The rewards are integrated into CleanWalkerLitterEnv and
    CleanWalkerLitterEnvCfg in cleanwalker_env.py.
"""

from __future__ import annotations

import torch


def reward_body_stability(
    projected_gravity_b: torch.Tensor,
    root_ang_vel_b: torch.Tensor,
    dt: float,
    scale: float = -5.0,
) -> torch.Tensor:
    """Penalize body roll and pitch for stable litter pickup.

    During the stop-and-reach phase, the body must be very stable
    to allow precise arm movements.

    Args:
        projected_gravity_b: (N, 3) gravity vector in body frame.
        root_ang_vel_b: (N, 3) angular velocity in body frame.
        dt: Physics timestep.
        scale: Reward scale (negative = penalty).

    Returns:
        (N,) reward tensor.
    """
    # Gravity deviation from vertical (projected_gravity_b[:, :2] = 0 when upright)
    tilt = torch.sum(torch.square(projected_gravity_b[:, :2]), dim=1)
    # Angular velocity around roll/pitch axes
    ang_vel = torch.sum(torch.square(root_ang_vel_b[:, :2]), dim=1)

    return (tilt + 0.1 * ang_vel) * scale * dt


def reward_velocity_tracking(
    commands: torch.Tensor,
    root_lin_vel_b: torch.Tensor,
    root_ang_vel_b: torch.Tensor,
    dt: float,
    lin_scale: float = 1.5,
    ang_scale: float = 0.75,
    sigma: float = 0.25,
) -> torch.Tensor:
    """Track commanded velocity with exponential kernel.

    Args:
        commands: (N, 3) velocity commands [vx, vy, yaw_rate].
        root_lin_vel_b: (N, 3) body-frame linear velocity.
        root_ang_vel_b: (N, 3) body-frame angular velocity.
        dt: Physics timestep.
        lin_scale: Linear velocity tracking weight.
        ang_scale: Angular velocity tracking weight.
        sigma: Exponential kernel width.

    Returns:
        (N,) reward tensor.
    """
    lin_vel_error = torch.sum(
        torch.square(commands[:, :2] - root_lin_vel_b[:, :2]), dim=1
    )
    track_lin = torch.exp(-lin_vel_error / sigma) * lin_scale * dt

    yaw_error = torch.square(commands[:, 2] - root_ang_vel_b[:, 2])
    track_ang = torch.exp(-yaw_error / sigma) * ang_scale * dt

    return track_lin + track_ang


def reward_energy_efficiency(
    applied_torque: torch.Tensor,
    joint_vel: torch.Tensor,
    dt: float,
    torque_scale: float = -2.5e-5,
    power_scale: float = -1.0e-5,
) -> torch.Tensor:
    """Penalize energy consumption (torque^2 + mechanical power).

    Args:
        applied_torque: (N, J) applied joint torques.
        joint_vel: (N, J) joint velocities.
        dt: Physics timestep.
        torque_scale: Torque penalty weight.
        power_scale: Mechanical power penalty weight.

    Returns:
        (N,) reward tensor.
    """
    torque_penalty = torch.sum(torch.square(applied_torque), dim=1) * torque_scale * dt
    # Mechanical power = |torque * velocity|
    power_penalty = torch.sum(
        torch.abs(applied_torque * joint_vel), dim=1
    ) * power_scale * dt

    return torque_penalty + power_penalty


def reward_smooth_motion(
    actions: torch.Tensor,
    previous_actions: torch.Tensor,
    preprevious_actions: torch.Tensor,
    joint_acc: torch.Tensor,
    dt: float,
    action_rate_scale: float = -0.01,
    jerk_scale: float = -0.005,
    accel_scale: float = -2.5e-7,
) -> torch.Tensor:
    """Penalize jerky motion (action rate + jerk + acceleration).

    Jerk is the derivative of acceleration — minimizing it produces
    smooth, natural-looking movements critical for arm precision.

    Args:
        actions: (N, J) current actions.
        previous_actions: (N, J) actions from previous step.
        preprevious_actions: (N, J) actions from 2 steps ago.
        joint_acc: (N, J) joint accelerations.
        dt: Physics timestep.
        action_rate_scale: First-derivative penalty weight.
        jerk_scale: Second-derivative (jerk) penalty weight.
        accel_scale: Acceleration penalty weight.

    Returns:
        (N,) reward tensor.
    """
    # Action rate (first derivative)
    action_rate = torch.sum(
        torch.square(actions - previous_actions), dim=1
    ) * action_rate_scale * dt

    # Action jerk (second derivative) — change in action rate
    jerk = torch.sum(
        torch.square(
            (actions - previous_actions) - (previous_actions - preprevious_actions)
        ),
        dim=1,
    ) * jerk_scale * dt

    # Joint acceleration
    accel = torch.sum(torch.square(joint_acc), dim=1) * accel_scale * dt

    return action_rate + jerk + accel


def reward_stop_and_stabilize(
    root_lin_vel_b: torch.Tensor,
    root_ang_vel_b: torch.Tensor,
    projected_gravity_b: torch.Tensor,
    pick_command: torch.Tensor,
    dt: float,
    stop_scale: float = 2.0,
    stability_scale: float = 3.0,
) -> torch.Tensor:
    """Reward the robot for stopping and stabilizing when pick command is active.

    When the pick_command flag is set, the robot should:
    1. Decelerate smoothly to zero velocity
    2. Maintain an extremely stable body orientation
    3. Keep all angular velocities near zero

    Args:
        root_lin_vel_b: (N, 3) body-frame linear velocity.
        root_ang_vel_b: (N, 3) body-frame angular velocity.
        projected_gravity_b: (N, 3) gravity vector in body frame.
        pick_command: (N,) binary flag — 1.0 when robot should stop for pickup.
        dt: Physics timestep.
        stop_scale: Reward for low velocity during pickup.
        stability_scale: Reward for stable orientation during pickup.

    Returns:
        (N,) reward tensor.
    """
    # Velocity near zero: exponential reward peaks at v=0
    vel_magnitude = torch.norm(root_lin_vel_b[:, :2], dim=1)
    stop_reward = torch.exp(-vel_magnitude / 0.05) * stop_scale * dt

    # Ultra-stable orientation: tighter than walking
    tilt = torch.sum(torch.square(projected_gravity_b[:, :2]), dim=1)
    ang_vel_mag = torch.sum(torch.square(root_ang_vel_b), dim=1)
    stability_reward = torch.exp(-(tilt + ang_vel_mag) / 0.01) * stability_scale * dt

    # Only active when pick command is set
    return (stop_reward + stability_reward) * pick_command


def reward_arm_reach(
    arm_joint_pos: torch.Tensor,
    arm_target_pos: torch.Tensor,
    pick_command: torch.Tensor,
    dt: float,
    reach_scale: float = 3.0,
    sigma: float = 0.1,
) -> torch.Tensor:
    """Reward arm joints reaching toward the litter target position.

    During the stop-and-reach phase, the arm should extend toward a
    ground-level target. This reward tracks how close the arm joint
    configuration is to the target configuration.

    The target configuration for ground pickup is pre-computed:
    - turret_yaw: pointing toward litter (variable)
    - shoulder_pitch: extended forward (~2.5 rad)
    - elbow_pitch: bent for ground reach (~1.5 rad)
    - wrist_pitch: angled down (~-0.5 rad)
    - gripper: open (0.0) or closed (0.8)

    Args:
        arm_joint_pos: (N, 5) current arm joint positions
            [turret_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, gripper].
        arm_target_pos: (N, 5) target arm joint positions.
        pick_command: (N,) binary flag — 1.0 when pickup is active.
        dt: Physics timestep.
        reach_scale: Reward weight.
        sigma: Exponential kernel width.

    Returns:
        (N,) reward tensor.
    """
    # Joint-space distance to target
    joint_error = torch.sum(
        torch.square(arm_joint_pos - arm_target_pos), dim=1
    )
    reach_reward = torch.exp(-joint_error / sigma) * reach_scale * dt

    return reach_reward * pick_command


def reward_grasp(
    gripper_pos: torch.Tensor,
    gripper_target: float,
    arm_near_target: torch.Tensor,
    pick_command: torch.Tensor,
    dt: float,
    grasp_scale: float = 5.0,
    threshold: float = 0.15,
) -> torch.Tensor:
    """Reward successful grasp (gripper closes when arm is at target).

    This is a staged reward: the gripper closing is only rewarded
    when the arm is already near the target position. This prevents
    the policy from learning to close the gripper prematurely.

    Args:
        gripper_pos: (N,) current gripper joint position.
        gripper_target: Target gripper position for closed grasp (~0.8 rad).
        arm_near_target: (N,) boolean — True when arm joints are near target.
        pick_command: (N,) binary flag — 1.0 during pickup.
        dt: Physics timestep.
        grasp_scale: Reward weight.
        threshold: Gripper position threshold for "grasped" state.

    Returns:
        (N,) reward tensor.
    """
    # Gripper closing toward target
    gripper_error = torch.abs(gripper_pos - gripper_target)
    gripper_reward = torch.exp(-gripper_error / 0.1) * grasp_scale * dt

    # Only reward when arm is already near target AND pick command active
    return gripper_reward * arm_near_target.float() * pick_command


def reward_smooth_deceleration(
    root_lin_vel_b: torch.Tensor,
    prev_root_lin_vel_b: torch.Tensor,
    pick_command: torch.Tensor,
    dt: float,
    scale: float = -2.0,
    max_decel: float = 2.0,
) -> torch.Tensor:
    """Penalize abrupt deceleration when transitioning to pickup.

    The robot should decelerate smoothly, not slam to a stop.
    Excessive deceleration destabilizes the body and makes arm
    control harder.

    Args:
        root_lin_vel_b: (N, 3) current body-frame linear velocity.
        prev_root_lin_vel_b: (N, 3) previous body-frame linear velocity.
        pick_command: (N,) binary flag — 1.0 during pickup.
        dt: Physics timestep.
        scale: Penalty weight (negative).
        max_decel: Maximum acceptable deceleration (m/s^2).

    Returns:
        (N,) reward tensor.
    """
    decel = (prev_root_lin_vel_b[:, :2] - root_lin_vel_b[:, :2]) / dt
    decel_magnitude = torch.norm(decel, dim=1)
    excess_decel = torch.clamp(decel_magnitude - max_decel, min=0.0)

    return torch.square(excess_decel) * scale * dt * pick_command
