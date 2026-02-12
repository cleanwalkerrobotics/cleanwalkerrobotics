#!/usr/bin/env python3
"""Package and upload trained CW-1 policy for distribution.

Creates a versioned release package containing:
- Policy weights (.zip)
- ONNX export (.onnx)
- metadata.json (training params, hardware, metrics, git hash)
- Validation results (if available)

Usage:
    python upload_model.py --local               # Create local zip archive
    python upload_model.py --github              # Create GitHub release (requires gh CLI)
    python upload_model.py --local --github      # Both

    python upload_model.py --local --tag v0.2.0  # Custom version tag
"""

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

SCRIPT_DIR = Path(__file__).parent.resolve()
REPO_ROOT = SCRIPT_DIR.parents[2]  # ml/locomotion/macos -> repo root


def get_git_hash() -> str:
    """Get current git commit hash."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True, text=True, cwd=str(REPO_ROOT),
        )
        return result.stdout.strip()[:12] if result.returncode == 0 else "unknown"
    except FileNotFoundError:
        return "unknown"


def get_git_branch() -> str:
    """Get current git branch."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            capture_output=True, text=True, cwd=str(REPO_ROOT),
        )
        return result.stdout.strip() if result.returncode == 0 else "unknown"
    except FileNotFoundError:
        return "unknown"


def get_git_dirty() -> bool:
    """Check if working tree has uncommitted changes."""
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            capture_output=True, text=True, cwd=str(REPO_ROOT),
        )
        return len(result.stdout.strip()) > 0
    except FileNotFoundError:
        return False


def find_latest_model() -> Path | None:
    """Find the best available model."""
    final = SCRIPT_DIR / "cw1_policy_final.zip"
    if final.exists():
        return final
    checkpoint_dir = SCRIPT_DIR / "checkpoints"
    if checkpoint_dir.exists():
        checkpoints = sorted(
            checkpoint_dir.glob("cw1_ppo_*_steps.zip"),
            key=lambda p: int(p.stem.split("_")[-2]),
        )
        if checkpoints:
            return checkpoints[-1]
    return None


def generate_onnx(model_path: Path, output_path: Path) -> bool:
    """Export model to ONNX using export_policy.py."""
    print(f"  Exporting ONNX to {output_path.name}...")
    try:
        result = subprocess.run(
            [sys.executable, str(SCRIPT_DIR / "export_policy.py"),
             "--checkpoint", str(model_path),
             "--output", str(output_path)],
            capture_output=True, text=True, cwd=str(SCRIPT_DIR),
        )
        if result.returncode == 0:
            return True
        print(f"  Warning: ONNX export failed:\n{result.stderr}")
        return False
    except Exception as e:
        print(f"  Warning: ONNX export error: {e}")
        return False


def build_metadata(model_path: Path, tag: str) -> dict:
    """Build metadata.json with training params, hardware, and metrics."""
    import torch

    # Try loading model to get training params
    training_params = {}
    try:
        from stable_baselines3 import PPO
        model = PPO.load(str(model_path), device="cpu")
        training_params = {
            "total_timesteps": model.num_timesteps,
            "learning_rate": float(model.learning_rate)
                if not callable(model.learning_rate)
                else "scheduled",
            "batch_size": model.batch_size,
            "n_steps": model.n_steps,
            "n_epochs": model.n_epochs,
            "gamma": model.gamma,
            "gae_lambda": model.gae_lambda,
            "clip_range": float(model.clip_range(1))
                if callable(model.clip_range)
                else float(model.clip_range),
            "ent_coef": model.ent_coef,
            "vf_coef": model.vf_coef,
            "max_grad_norm": model.max_grad_norm,
            "policy_architecture": "MLP [512, 256, 128]",
            "activation": "ELU",
            "observation_dim": 48,
            "action_dim": 12,
        }
    except Exception as e:
        print(f"  Warning: Could not extract training params: {e}")

    # Hardware info
    hardware = {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "python_version": platform.python_version(),
        "torch_version": torch.__version__,
        "mps_available": torch.backends.mps.is_available()
            if hasattr(torch.backends, "mps") else False,
    }

    # Try reading macOS-specific hardware info
    try:
        result = subprocess.run(
            ["sysctl", "-n", "machdep.cpu.brand_string"],
            capture_output=True, text=True,
        )
        if result.returncode == 0:
            hardware["cpu"] = result.stdout.strip()

        result = subprocess.run(
            ["sysctl", "-n", "hw.memsize"],
            capture_output=True, text=True,
        )
        if result.returncode == 0:
            hardware["ram_gb"] = int(result.stdout.strip()) // (1024**3)
    except FileNotFoundError:
        pass  # Not on macOS

    # Validation results
    validation = None
    val_path = SCRIPT_DIR / "validation_results.json"
    if val_path.exists():
        try:
            with open(val_path) as f:
                validation = json.load(f)
        except Exception:
            pass

    # Git info
    git_info = {
        "hash": get_git_hash(),
        "branch": get_git_branch(),
        "dirty": get_git_dirty(),
    }

    metadata = {
        "model_name": "cw1-locomotion-ppo",
        "version": tag,
        "description": "CW-1 quadruped locomotion policy trained with PPO on macOS Apple Silicon",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "model_file": model_path.name,
        "training": training_params,
        "hardware": hardware,
        "git": git_info,
        "environment": {
            "name": "CW1-Locomotion-v0",
            "sim_dt": 0.005,
            "control_dt": 0.02,
            "decimation": 4,
            "max_episode_steps": 1000,
            "observation_dim": 48,
            "action_dim": 12,
        },
    }

    if validation:
        metadata["validation"] = {
            "trained": validation.get("trained", {}),
            "random_baseline": validation.get("random_baseline", {}),
            "episodes": validation.get("episodes_per_policy", 100),
        }

    return metadata


def auto_version_tag() -> str:
    """Generate an auto-incremented version tag."""
    git_hash = get_git_hash()
    date_str = time.strftime("%Y%m%d")
    return f"v0.1.0-{date_str}-{git_hash[:7]}"


def package_local(model_path: Path, tag: str, output_dir: Path) -> Path:
    """Create a local zip archive with all artifacts."""
    print("\n📦 Packaging local archive...")

    archive_name = f"cw1-locomotion-{tag}"
    output_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmpdir:
        pkg_dir = Path(tmpdir) / archive_name
        pkg_dir.mkdir()

        # Copy model weights
        dst = pkg_dir / model_path.name
        shutil.copy2(model_path, dst)
        print(f"  ✓ {model_path.name}")

        # Export ONNX
        onnx_path = pkg_dir / "cw1_policy.onnx"
        if generate_onnx(model_path, onnx_path):
            print(f"  ✓ cw1_policy.onnx")
        else:
            # Try copying existing ONNX
            existing_onnx = SCRIPT_DIR / "cw1_policy.onnx"
            if existing_onnx.exists():
                shutil.copy2(existing_onnx, onnx_path)
                print(f"  ✓ cw1_policy.onnx (existing)")

        # Generate metadata
        metadata = build_metadata(model_path, tag)
        meta_path = pkg_dir / "metadata.json"
        with open(meta_path, "w") as f:
            json.dump(metadata, f, indent=2)
        print(f"  ✓ metadata.json")

        # Copy validation results
        val_path = SCRIPT_DIR / "validation_results.json"
        if val_path.exists():
            shutil.copy2(val_path, pkg_dir / "validation_results.json")
            print(f"  ✓ validation_results.json")

        # Create zip
        archive_path = output_dir / archive_name
        shutil.make_archive(str(archive_path), "zip", tmpdir, archive_name)

    final_path = output_dir / f"{archive_name}.zip"
    size_mb = final_path.stat().st_size / (1024 * 1024)
    print(f"\n✅ Archive created: {final_path} ({size_mb:.1f} MB)")
    return final_path


def release_github(model_path: Path, tag: str):
    """Create a GitHub release using the gh CLI."""
    print("\n🚀 Creating GitHub release...")

    # Check gh CLI
    if not shutil.which("gh"):
        print("❌ GitHub CLI (gh) not found. Install: brew install gh")
        sys.exit(1)

    # Check auth
    result = subprocess.run(
        ["gh", "auth", "status"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print("❌ Not authenticated with GitHub. Run: gh auth login")
        sys.exit(1)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)
        assets = []

        # Copy model weights
        dst = tmpdir / model_path.name
        shutil.copy2(model_path, dst)
        assets.append(str(dst))

        # Export ONNX
        onnx_path = tmpdir / "cw1_policy.onnx"
        if generate_onnx(model_path, onnx_path):
            assets.append(str(onnx_path))
        elif (SCRIPT_DIR / "cw1_policy.onnx").exists():
            dst = tmpdir / "cw1_policy.onnx"
            shutil.copy2(SCRIPT_DIR / "cw1_policy.onnx", dst)
            assets.append(str(dst))

        # Generate metadata
        metadata = build_metadata(model_path, tag)
        meta_path = tmpdir / "metadata.json"
        with open(meta_path, "w") as f:
            json.dump(metadata, f, indent=2)
        assets.append(str(meta_path))

        # Validation results
        val_path = SCRIPT_DIR / "validation_results.json"
        if val_path.exists():
            dst = tmpdir / "validation_results.json"
            shutil.copy2(val_path, dst)
            assets.append(str(dst))

        # Build release notes
        notes_parts = [
            f"## CW-1 Locomotion Policy {tag}\n",
            f"Trained with PPO on macOS Apple Silicon.\n",
            f"- **Timesteps:** {metadata['training'].get('total_timesteps', 'N/A'):,}",
            f"- **Architecture:** MLP [512, 256, 128] + ELU",
            f"- **Git:** `{metadata['git']['hash']}`",
        ]

        if "validation" in metadata:
            v = metadata["validation"]
            trained = v.get("trained", {})
            notes_parts.extend([
                f"\n### Validation ({v.get('episodes', 100)} episodes)",
                f"- Success rate: {trained.get('success_rate', 0):.1%}",
                f"- Avg reward: {trained.get('avg_reward', 0):.2f}",
                f"- Avg distance: {trained.get('avg_distance', 0):.2f} m",
            ])

        notes = "\n".join(notes_parts)

        # Create release
        cmd = [
            "gh", "release", "create", tag,
            "--title", f"CW-1 Locomotion {tag}",
            "--notes", notes,
            "--repo", "cleanwalkerrobotics/cleanwalkerrobotics",
        ] + assets

        print(f"  Creating release {tag}...")
        result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(REPO_ROOT))

        if result.returncode == 0:
            print(f"✅ GitHub release created: {tag}")
            if result.stdout.strip():
                print(f"   {result.stdout.strip()}")
        else:
            print(f"❌ Release failed: {result.stderr}")
            sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Package and upload CW-1 trained policy"
    )
    parser.add_argument(
        "--checkpoint", type=Path, default=None,
        help="Path to trained model (.zip)"
    )
    parser.add_argument(
        "--tag", type=str, default=None,
        help="Version tag (auto-generated if omitted)"
    )
    parser.add_argument(
        "--local", action="store_true",
        help="Create local zip archive in releases/"
    )
    parser.add_argument(
        "--github", action="store_true",
        help="Create GitHub release (requires gh CLI)"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=SCRIPT_DIR / "releases",
        help="Output directory for local archives"
    )
    args = parser.parse_args()

    if not args.local and not args.github:
        print("Specify at least one of --local or --github")
        parser.print_help()
        sys.exit(1)

    # Find model
    model_path = args.checkpoint or find_latest_model()
    if model_path is None or not model_path.exists():
        print("❌ No trained model found. Run train.py first.")
        sys.exit(1)

    # Version tag
    tag = args.tag or auto_version_tag()

    print("=" * 56)
    print("CW-1 Model Upload")
    print("=" * 56)
    print(f"  Model: {model_path}")
    print(f"  Tag:   {tag}")
    print(f"  Mode:  {'local' if args.local else ''}"
          f"{'+ ' if args.local and args.github else ''}"
          f"{'github' if args.github else ''}")
    print()

    if args.local:
        archive = package_local(model_path, tag, args.output_dir)

    if args.github:
        release_github(model_path, tag)

    print("\n🎉 Done!")


if __name__ == "__main__":
    main()
