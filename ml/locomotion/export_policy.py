# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""
CleanWalker CW-1 — Export trained policy to ONNX for Jetson deployment.

Converts a PyTorch PPO policy checkpoint (.pt) to ONNX format (.onnx)
that can be run on NVIDIA Jetson Orin Nano Super via TensorRT.

The exported model takes the 48-dim observation vector as input and
outputs 12-dim leg joint position targets.

Usage:
    python ml/locomotion/export_policy.py \\
        --checkpoint logs/rsl_rl/cleanwalker_cw1_flat/<run>/model_1500.pt \\
        --output ml/locomotion/assets/cleanwalker_cw1_policy.onnx

    # With TensorRT optimization hints
    python ml/locomotion/export_policy.py \\
        --checkpoint logs/rsl_rl/cleanwalker_cw1_rough/<run>/model_3000.pt \\
        --output ml/locomotion/assets/cleanwalker_cw1_policy.onnx \\
        --fp16

Jetson Deployment:
    # On Jetson Orin Nano Super, convert ONNX to TensorRT engine:
    /usr/src/tensorrt/bin/trtexec \\
        --onnx=cleanwalker_cw1_policy.onnx \\
        --saveEngine=cleanwalker_cw1_policy.engine \\
        --fp16 \\
        --workspace=256

    # Inference at 50 Hz (policy frequency):
    #   Input:  float32[1, 48] — observation vector
    #   Output: float32[1, 12] — joint position targets (scaled)
    # Latency: <1ms on Jetson Orin Nano Super with FP16
"""

from __future__ import annotations

import argparse
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ML_DIR = os.path.dirname(_SCRIPT_DIR)
if _ML_DIR not in sys.path:
    sys.path.insert(0, _ML_DIR)


def export_onnx(
    checkpoint_path: str,
    output_path: str,
    obs_size: int = 48,
    act_size: int = 12,
    fp16: bool = False,
    opset_version: int = 17,
) -> str:
    """Export a trained rsl_rl actor network to ONNX.

    Args:
        checkpoint_path: Path to the .pt checkpoint from rsl_rl training.
        output_path: Path to write the .onnx file.
        obs_size: Observation space dimension (48 for locomotion).
        act_size: Action space dimension (12 for leg joints).
        fp16: Whether to use float16 for weights (smaller model, faster on Jetson).
        opset_version: ONNX opset version.

    Returns:
        Absolute path to the exported .onnx file.
    """
    import torch
    import torch.nn as nn

    # Load checkpoint
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)

    # rsl_rl saves the actor-critic model state dict under 'model_state_dict'
    model_state = checkpoint.get("model_state_dict", checkpoint)

    # Extract actor network weights from the ActorCritic model
    # rsl_rl ActorCritic uses: actor = nn.Sequential(...)
    # Keys look like: actor.0.weight, actor.0.bias, actor.2.weight, ...
    actor_keys = [k for k in model_state.keys() if k.startswith("actor.")]

    if not actor_keys:
        print("ERROR: No 'actor.*' keys found in checkpoint.")
        print(f"Available keys: {list(model_state.keys())[:20]}")
        sys.exit(1)

    # Reconstruct the actor MLP from checkpoint weights
    # rsl_rl uses: Linear → ELU → Linear → ELU → ... → Linear (no final activation)
    layers = []
    layer_indices = sorted(set(int(k.split(".")[1]) for k in actor_keys))

    for idx in layer_indices:
        weight_key = f"actor.{idx}.weight"
        bias_key = f"actor.{idx}.bias"

        if weight_key in model_state:
            weight = model_state[weight_key]
            in_features = weight.shape[1]
            out_features = weight.shape[0]

            linear = nn.Linear(in_features, out_features)
            linear.weight.data = weight
            if bias_key in model_state:
                linear.bias.data = model_state[bias_key]

            layers.append(linear)
        elif f"actor.{idx}" in str(actor_keys):
            # This is an activation layer (ELU) — inferred from position
            layers.append(nn.ELU())

    # If we only got linear layers, interleave ELU activations (except last)
    if all(isinstance(l, nn.Linear) for l in layers):
        interleaved = []
        for i, layer in enumerate(layers):
            interleaved.append(layer)
            if i < len(layers) - 1:
                interleaved.append(nn.ELU())
        layers = interleaved

    actor = nn.Sequential(*layers)
    actor.eval()

    if fp16:
        actor = actor.half()

    print(f"[export] Actor network architecture:")
    print(f"  {actor}")
    total_params = sum(p.numel() for p in actor.parameters())
    print(f"  Total parameters: {total_params:,}")

    # Create dummy input
    dtype = torch.float16 if fp16 else torch.float32
    dummy_obs = torch.randn(1, obs_size, dtype=dtype)

    # Test forward pass
    with torch.no_grad():
        dummy_out = actor(dummy_obs)
    print(f"  Input shape:  {dummy_obs.shape}")
    print(f"  Output shape: {dummy_out.shape}")
    assert dummy_out.shape[-1] == act_size, (
        f"Expected output dim {act_size}, got {dummy_out.shape[-1]}"
    )

    # Export to ONNX
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    torch.onnx.export(
        actor,
        dummy_obs,
        output_path,
        opset_version=opset_version,
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes={
            "observation": {0: "batch_size"},
            "action": {0: "batch_size"},
        },
    )

    file_size = os.path.getsize(output_path)
    print(f"\n[export] ONNX model saved: {output_path}")
    print(f"[export] File size: {file_size / 1024:.1f} KB")
    print(f"[export] Precision: {'FP16' if fp16 else 'FP32'}")
    print(f"[export] Opset version: {opset_version}")

    # Validate with onnxruntime if available
    try:
        import onnxruntime as ort

        session = ort.InferenceSession(output_path)
        input_name = session.get_inputs()[0].name
        ort_out = session.run(None, {input_name: dummy_obs.numpy()})

        # Compare outputs
        diff = abs(dummy_out.numpy() - ort_out[0]).max()
        print(f"[export] ONNX validation: max diff = {diff:.6f} (should be ~0)")
        if diff < 1e-4:
            print("[export] ONNX model validated successfully.")
        else:
            print("[export] WARNING: Large difference between PyTorch and ONNX outputs.")
    except ImportError:
        print("[export] Install onnxruntime to validate: pip install onnxruntime")

    return os.path.abspath(output_path)


def main():
    parser = argparse.ArgumentParser(
        description="Export CleanWalker policy to ONNX for Jetson deployment"
    )
    parser.add_argument(
        "--checkpoint",
        type=str,
        required=True,
        help="Path to rsl_rl .pt checkpoint",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output .onnx path (default: assets/cleanwalker_cw1_policy.onnx)",
    )
    parser.add_argument(
        "--obs_size",
        type=int,
        default=48,
        help="Observation dimension (default: 48)",
    )
    parser.add_argument(
        "--act_size",
        type=int,
        default=12,
        help="Action dimension (default: 12)",
    )
    parser.add_argument(
        "--fp16",
        action="store_true",
        help="Export with FP16 weights (smaller, faster on Jetson)",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=17,
        help="ONNX opset version (default: 17)",
    )
    args = parser.parse_args()

    output = args.output or os.path.join(_SCRIPT_DIR, "assets", "cleanwalker_cw1_policy.onnx")

    export_onnx(
        checkpoint_path=args.checkpoint,
        output_path=output,
        obs_size=args.obs_size,
        act_size=args.act_size,
        fp16=args.fp16,
        opset_version=args.opset,
    )

    print("\n--- Jetson Deployment ---")
    print("Copy the .onnx file to Jetson and convert to TensorRT:")
    print(f"  scp {output} jetson:/home/cleanwalker/models/")
    print("  # On Jetson:")
    print("  /usr/src/tensorrt/bin/trtexec \\")
    print(f"    --onnx=cleanwalker_cw1_policy.onnx \\")
    print("    --saveEngine=cleanwalker_cw1_policy.engine \\")
    print("    --fp16 --workspace=256")
    print("\nExpected inference latency: <1ms at 50 Hz on Jetson Orin Nano Super")


if __name__ == "__main__":
    main()
