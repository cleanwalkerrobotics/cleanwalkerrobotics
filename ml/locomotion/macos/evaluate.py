#!/usr/bin/env python3
"""Quick evaluation: render demo, extract frames, print metrics.

Usage:
    python evaluate.py                    # Evaluate latest checkpoint
    python evaluate.py --steps 500        # Longer evaluation
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

SCRIPT_DIR = Path(__file__).parent.resolve()


def find_best_model() -> Path | None:
    candidates = [SCRIPT_DIR / "cw1_policy_final.zip"]
    checkpoint_dir = SCRIPT_DIR / "checkpoints"
    if checkpoint_dir.exists():
        checkpoints = sorted(
            checkpoint_dir.glob("cw1_ppo_*_steps.zip"),
            key=lambda p: int(p.stem.split("_")[-2]),
        )
        if checkpoints:
            candidates.insert(0, checkpoints[-1])
    for path in candidates:
        if path.exists():
            return path
    return None


def main():
    parser = argparse.ArgumentParser(description="Evaluate CW-1 policy")
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--steps", type=int, default=250, help="Steps per episode (250=5s)")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--cmd-vx", type=float, default=0.4)
    parser.add_argument("--render-frames", action="store_true", help="Save frames")
    args = parser.parse_args()

    model_path = args.checkpoint or find_best_model()
    if model_path is None:
        print("No model found. Run train.py first.")
        sys.exit(1)

    import mujoco
    from stable_baselines3 import PPO

    sys.path.insert(0, str(SCRIPT_DIR))
    from cw1_env import CW1LocomotionEnv

    print(f"Loading: {model_path}")
    model = PPO.load(str(model_path), device="cpu")

    env = CW1LocomotionEnv(randomize_friction=False, randomize_mass=False)

    frame_dir = SCRIPT_DIR / "renders" / "frames"
    if args.render_frames:
        frame_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(env.model, 480, 640)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance = 2.0
        cam.azimuth = 90
        cam.elevation = -20

    print(f"\nEvaluating {args.episodes} episodes, {args.steps} steps each")
    print(f"Velocity command: {args.cmd_vx} m/s")
    print("-" * 60)

    all_results = []

    for ep in range(args.episodes):
        obs, info = env.reset()
        env._velocity_cmd = np.array([args.cmd_vx, 0.0, 0.0], dtype=np.float32)

        start_x = env.data.xpos[env.body_id][0]
        total_reward = 0
        heights = []
        tilts = []
        survived_steps = 0

        for step in range(args.steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)

            h = env.data.xpos[env.body_id][2]
            body_quat = env.data.xquat[env.body_id]
            body_rot = np.zeros(9)
            mujoco.mju_quat2Mat(body_rot, body_quat)
            body_rot = body_rot.reshape(3, 3)
            up = body_rot.T @ np.array([0, 0, 1.0])
            tilt = np.degrees(np.arccos(np.clip(up[2], -1, 1)))

            heights.append(h)
            tilts.append(tilt)
            total_reward += reward
            survived_steps = step + 1

            # Save frame every 1s
            if args.render_frames and ep == 0 and step % 50 == 0:
                cam.lookat[:] = env.data.xpos[env.body_id]
                renderer.update_scene(env.data, cam)
                frame = renderer.render()
                import cv2
                cv2.imwrite(str(frame_dir / f"eval_{step:04d}.png"), frame[:, :, ::-1])

            if terminated or truncated:
                break

        end_x = env.data.xpos[env.body_id][0]
        distance = end_x - start_x
        duration = survived_steps * 0.02
        avg_speed = distance / duration if duration > 0 else 0

        result = {
            "ep": ep,
            "reward": total_reward,
            "distance": distance,
            "avg_speed": avg_speed,
            "avg_height": np.mean(heights),
            "avg_tilt": np.mean(tilts),
            "max_tilt": np.max(tilts),
            "survived": survived_steps * 0.02,
            "survived_pct": survived_steps / args.steps * 100,
        }
        all_results.append(result)

        print(f"  Ep {ep}: reward={total_reward:.1f}  dist={distance:.2f}m  "
              f"speed={avg_speed:.3f}m/s  height={np.mean(heights):.3f}  "
              f"tilt={np.mean(tilts):.1f}°  survived={duration:.1f}s")

    print("-" * 60)
    avg = lambda k: np.mean([r[k] for r in all_results])
    print(f"AVERAGE: reward={avg('reward'):.1f}  dist={avg('distance'):.2f}m  "
          f"speed={avg('avg_speed'):.3f}m/s  height={avg('avg_height'):.3f}  "
          f"tilt={avg('avg_tilt'):.1f}°  survived={avg('survived'):.1f}s "
          f"({avg('survived_pct'):.0f}%)")

    # Walking assessment
    speed = avg("avg_speed")
    surv = avg("survived_pct")
    tilt = avg("avg_tilt")

    print("\n=== WALKING ASSESSMENT ===")
    if surv < 50:
        print("FAILING: Robot falls over quickly (<50% survival)")
    elif speed < 0.02:
        if tilt < 20:
            print("STANDING: Robot stays upright but doesn't move")
        else:
            print("LEANING: Robot tilts but doesn't walk")
    elif speed < 0.1:
        print("SHUFFLING: Some forward motion but not walking")
    elif speed < 0.3:
        print("CRAWLING: Slow forward movement, may be dragging")
    else:
        print("WALKING: Meaningful forward locomotion detected!")

    env.close()
    if args.render_frames:
        renderer.close()


if __name__ == "__main__":
    main()
