#!/usr/bin/env python3
"""Render a polished CW-1 walking demo video.

Produces a 1080p 30fps video with:
- Multiple camera angles (side, front, bird's eye)
- Clean background
- CleanWalker branding overlay
- Professional lighting

Usage:
    python render_demo.py                          # Full demo
    python render_demo.py --duration 10            # 10 second clip
    python render_demo.py --output my_demo.mp4     # Custom output
    python render_demo.py --no-overlay             # Without text
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

SCRIPT_DIR = Path(__file__).parent.resolve()

# Video settings
WIDTH = 1920
HEIGHT = 1080
FPS = 30

# Camera configurations
CAMERAS = {
    "side": {
        "lookat": [0, 0, 0.3],
        "distance": 1.8,
        "azimuth": 90,
        "elevation": -15,
    },
    "front": {
        "lookat": [0, 0, 0.3],
        "distance": 1.5,
        "azimuth": 180,
        "elevation": -10,
    },
    "bird": {
        "lookat": [0, 0, 0.2],
        "distance": 2.5,
        "azimuth": 120,
        "elevation": -45,
    },
    "tracking_side": {
        "lookat": None,  # Follows robot
        "distance": 2.0,
        "azimuth": 90,
        "elevation": -20,
    },
}


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


def add_text_overlay(frame: np.ndarray, text: str, position: str = "bottom",
                     font_scale: float = 1.0) -> np.ndarray:
    """Add text overlay to frame using matplotlib (no OpenCV dependency)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.figure import Figure
        from io import BytesIO

        h, w = frame.shape[:2]
        fig = Figure(figsize=(w / 100, h / 100), dpi=100)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.imshow(frame)
        ax.axis("off")

        if position == "bottom":
            y = 0.05
            va = "bottom"
        elif position == "top":
            y = 0.95
            va = "top"
        else:
            y = 0.5
            va = "center"

        ax.text(
            0.5, y, text,
            transform=ax.transAxes,
            fontsize=14 * font_scale,
            color="white",
            ha="center", va=va,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="black", alpha=0.6),
        )

        buf = BytesIO()
        fig.savefig(buf, format="raw", dpi=100)
        buf.seek(0)
        result = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        result = result.reshape(h, w, 4)[:, :, :3]  # RGBA -> RGB
        plt.close(fig)
        return result

    except Exception:
        return frame


def main():
    parser = argparse.ArgumentParser(description="Render CW-1 demo video")
    parser.add_argument(
        "--checkpoint", type=Path, default=None,
        help="Path to trained model"
    )
    parser.add_argument(
        "--output", type=Path, default=SCRIPT_DIR / "renders" / "cw1-walking-demo.mp4",
        help="Output video path"
    )
    parser.add_argument(
        "--duration", type=float, default=20.0,
        help="Video duration in seconds"
    )
    parser.add_argument(
        "--cmd-vx", type=float, default=0.8,
        help="Forward velocity command"
    )
    parser.add_argument(
        "--no-overlay", action="store_true",
        help="Skip text overlay"
    )
    parser.add_argument(
        "--camera", type=str, default="tracking_side",
        choices=list(CAMERAS.keys()) + ["multi"],
        help="Camera angle (multi = cycle through all)"
    )
    args = parser.parse_args()

    model_path = args.checkpoint or find_best_model()
    if model_path is None or not model_path.exists():
        print("No trained model found. Run train.py first.")
        sys.exit(1)

    import mujoco
    import imageio
    from stable_baselines3 import PPO

    sys.path.insert(0, str(SCRIPT_DIR))
    from cw1_env import CW1LocomotionEnv

    print(f"Loading model: {model_path}")
    model = PPO.load(str(model_path), device="cpu")

    print("Setting up environment...")
    env = CW1LocomotionEnv()

    # Setup renderer
    renderer = mujoco.Renderer(env.model, HEIGHT, WIDTH)

    # Camera setup
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    total_frames = int(args.duration * FPS)
    sim_steps_per_frame = int(1.0 / (FPS * env.control_dt))
    frames = []

    print(f"Rendering {total_frames} frames ({args.duration}s at {FPS}fps)...")
    print(f"Camera: {args.camera}, velocity cmd: {args.cmd_vx} m/s")

    obs, info = env.reset()
    env._velocity_cmd = np.array([args.cmd_vx, 0.0, 0.0], dtype=np.float32)

    # Determine camera schedule for multi-camera mode
    if args.camera == "multi":
        cam_names = ["side", "front", "bird", "tracking_side"]
        frames_per_cam = total_frames // len(cam_names)
    else:
        cam_names = [args.camera]
        frames_per_cam = total_frames

    frame_idx = 0
    sim_step = 0

    for cam_idx, cam_name in enumerate(cam_names):
        cam_config = CAMERAS[cam_name]
        n_frames = frames_per_cam if cam_idx < len(cam_names) - 1 else (
            total_frames - frame_idx
        )

        for local_frame in range(n_frames):
            # Step simulation
            for _ in range(max(1, sim_steps_per_frame)):
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                sim_step += 1

                if terminated or truncated:
                    obs, info = env.reset()
                    env._velocity_cmd = np.array(
                        [args.cmd_vx, 0.0, 0.0], dtype=np.float32
                    )

            # Set camera
            if cam_config["lookat"] is not None:
                cam.lookat[:] = cam_config["lookat"]
            else:
                # Track robot
                body_pos = env.data.xpos[env.body_id]
                cam.lookat[:] = body_pos

            cam.distance = cam_config["distance"]
            cam.azimuth = cam_config["azimuth"]
            cam.elevation = cam_config["elevation"]

            # Render
            renderer.update_scene(env.data, cam)
            frame = renderer.render()

            # Add overlay
            if not args.no_overlay:
                t = frame_idx / FPS
                overlay_text = f"CleanWalker CW-1  |  {t:.1f}s  |  {args.cmd_vx:.1f} m/s"
                frame = add_text_overlay(frame, overlay_text, position="bottom")

            frames.append(frame)
            frame_idx += 1

            if frame_idx % 100 == 0:
                print(f"  Frame {frame_idx}/{total_frames}")

    env.close()
    renderer.close()

    # Save video
    args.output.parent.mkdir(parents=True, exist_ok=True)
    print(f"\nSaving video to: {args.output}")
    imageio.mimwrite(str(args.output), frames, fps=FPS, quality=9)

    file_size = args.output.stat().st_size / (1024 * 1024)
    print(f"Video saved: {args.output} ({file_size:.1f} MB)")
    print(f"  Resolution: {WIDTH}x{HEIGHT}")
    print(f"  Duration: {len(frames) / FPS:.1f}s")
    print(f"  Frames: {len(frames)}")


if __name__ == "__main__":
    main()
