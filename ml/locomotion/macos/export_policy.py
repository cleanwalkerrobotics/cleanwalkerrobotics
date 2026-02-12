#!/usr/bin/env python3
"""Export trained CW-1 policy to ONNX for deployment.

Exports the policy network to ONNX format suitable for:
- Jetson Orin Nano (with TensorRT)
- Any ONNX runtime

Usage:
    python export_policy.py                                    # Auto-find model
    python export_policy.py --checkpoint cw1_policy_final.zip  # Specific model
    python export_policy.py --output cw1_policy.onnx           # Custom output
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import torch

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

SCRIPT_DIR = Path(__file__).parent.resolve()

# Must match cw1_env.py
OBS_DIM = 48
ACT_DIM = 12


def find_best_model() -> Path | None:
    """Find the best available model file."""
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
    parser = argparse.ArgumentParser(description="Export CW-1 policy to ONNX")
    parser.add_argument(
        "--checkpoint", type=Path, default=None,
        help="Path to trained model (.zip)"
    )
    parser.add_argument(
        "--output", type=Path, default=SCRIPT_DIR / "cw1_policy.onnx",
        help="Output ONNX file path"
    )
    parser.add_argument(
        "--opset", type=int, default=17,
        help="ONNX opset version"
    )
    parser.add_argument(
        "--verify", action="store_true", default=True,
        help="Verify exported model (default: True)"
    )
    args = parser.parse_args()

    model_path = args.checkpoint or find_best_model()
    if model_path is None or not model_path.exists():
        print("No trained model found. Run train.py first.")
        sys.exit(1)

    print(f"Loading model: {model_path}")

    from stable_baselines3 import PPO

    model = PPO.load(str(model_path), device="cpu")
    policy = model.policy

    print(f"Policy parameters: {sum(p.numel() for p in policy.parameters()):,}")

    # Extract the MLP actor network
    # SB3 MlpPolicy: policy.mlp_extractor → policy.action_net
    policy.eval()

    # Create wrapper that takes observation and returns action (deterministic)
    class PolicyWrapper(torch.nn.Module):
        def __init__(self, sb3_policy):
            super().__init__()
            self.features_extractor = sb3_policy.features_extractor
            self.mlp_extractor = sb3_policy.mlp_extractor
            self.action_net = sb3_policy.action_net

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            features = self.features_extractor(obs)
            latent_pi, _ = self.mlp_extractor(features)
            actions = self.action_net(latent_pi)
            return actions

    wrapper = PolicyWrapper(policy)
    wrapper.eval()

    # Export to ONNX
    dummy_obs = torch.randn(1, OBS_DIM, dtype=torch.float32)

    print(f"\nExporting to ONNX (opset {args.opset})...")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    torch.onnx.export(
        wrapper,
        dummy_obs,
        str(args.output),
        opset_version=args.opset,
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes={
            "observation": {0: "batch_size"},
            "action": {0: "batch_size"},
        },
    )

    file_size = args.output.stat().st_size
    print(f"Exported: {args.output} ({file_size / 1024:.1f} KB)")

    # Verify
    if args.verify:
        print("\nVerifying ONNX model...")
        import onnx
        import onnxruntime as ort

        # Check model validity
        onnx_model = onnx.load(str(args.output))
        onnx.checker.check_model(onnx_model)
        print("  ONNX model check: PASSED")

        # Run inference comparison
        sess = ort.InferenceSession(str(args.output))
        test_obs = np.random.randn(1, OBS_DIM).astype(np.float32)

        # PyTorch output
        with torch.no_grad():
            torch_out = wrapper(torch.from_numpy(test_obs)).numpy()

        # ONNX output
        onnx_out = sess.run(None, {"observation": test_obs})[0]

        # Compare
        max_diff = np.max(np.abs(torch_out - onnx_out))
        print(f"  Max difference (PyTorch vs ONNX): {max_diff:.2e}")
        if max_diff < 1e-5:
            print("  Verification: PASSED")
        else:
            print(f"  Warning: difference > 1e-5 (may be float precision)")

        print(f"\n  Input shape:  {test_obs.shape} (observation)")
        print(f"  Output shape: {onnx_out.shape} (action)")
        print(f"  Output range: [{onnx_out.min():.3f}, {onnx_out.max():.3f}]")

    print(f"\nDone! Deploy {args.output.name} to Jetson with TensorRT or onnxruntime.")


if __name__ == "__main__":
    main()
