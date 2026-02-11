#!/usr/bin/env bash
# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0
#
# Download trained CW-1 model from a cloud GPU instance.
#
# Usage:
#   ./ml/locomotion/scripts/download-model.sh <HOST> [PORT] [USER]
#
# Examples:
#   # Vast.ai (custom port, root user)
#   ./ml/locomotion/scripts/download-model.sh 123.45.67.89 12345
#
#   # RunPod (custom port + user)
#   ./ml/locomotion/scripts/download-model.sh 123.45.67.89 22222 runpod
#
#   # Lambda Labs (default port 22, ubuntu user)
#   ./ml/locomotion/scripts/download-model.sh 123.45.67.89 22 ubuntu

set -euo pipefail

HOST="${1:?Usage: $0 <HOST> [PORT] [USER]}"
PORT="${2:-22}"
USER="${3:-root}"
REMOTE_REPO="cleanwalkerrobotics"

LOCAL_DIR="$(cd "$(dirname "$0")/../../.." && pwd)"
ASSETS_DIR="${LOCAL_DIR}/ml/locomotion/assets"
LOGS_DIR="${LOCAL_DIR}/logs"

echo "=== CW-1 Model Download ==="
echo "  Host:  ${USER}@${HOST}:${PORT}"
echo "  Local: ${ASSETS_DIR}"
echo ""

# --- Download ONNX model ---
echo "[1/3] Downloading ONNX model..."
mkdir -p "${ASSETS_DIR}"
scp -P "${PORT}" \
    "${USER}@${HOST}:~/${REMOTE_REPO}/ml/locomotion/assets/cleanwalker_cw1_policy.onnx" \
    "${ASSETS_DIR}/cleanwalker_cw1_policy.onnx"

if [ -f "${ASSETS_DIR}/cleanwalker_cw1_policy.onnx" ]; then
    SIZE=$(du -h "${ASSETS_DIR}/cleanwalker_cw1_policy.onnx" | cut -f1)
    echo "  OK: cleanwalker_cw1_policy.onnx (${SIZE})"
else
    echo "  FAIL: ONNX model not found on remote. Did export-policy run?"
    exit 1
fi

# --- Download checkpoints ---
echo ""
echo "[2/3] Downloading training checkpoints..."
mkdir -p "${LOGS_DIR}"
scp -P "${PORT}" -r \
    "${USER}@${HOST}:~/${REMOTE_REPO}/logs/rsl_rl/" \
    "${LOGS_DIR}/rsl_rl/" \
    2>/dev/null || echo "  SKIP: No training logs found (optional)."

if [ -d "${LOGS_DIR}/rsl_rl" ]; then
    COUNT=$(find "${LOGS_DIR}/rsl_rl" -name "model_*.pt" 2>/dev/null | wc -l)
    echo "  OK: Downloaded ${COUNT} checkpoint(s)"
fi

# --- Verify ---
echo ""
echo "[3/3] Verifying downloads..."
echo ""
echo "  ONNX model:  ${ASSETS_DIR}/cleanwalker_cw1_policy.onnx"

if [ -d "${LOGS_DIR}/rsl_rl" ]; then
    echo "  Training logs: ${LOGS_DIR}/rsl_rl/"
fi

echo ""
echo "=== Done ==="
echo ""
echo "Next steps:"
echo "  1. Copy to Jetson:  scp ${ASSETS_DIR}/cleanwalker_cw1_policy.onnx jetson:/home/cleanwalker/models/"
echo "  2. Convert to TensorRT on Jetson:"
echo "     /usr/src/tensorrt/bin/trtexec \\"
echo "       --onnx=cleanwalker_cw1_policy.onnx \\"
echo "       --saveEngine=cleanwalker_cw1_policy.engine \\"
echo "       --fp16 --workspace=256"
