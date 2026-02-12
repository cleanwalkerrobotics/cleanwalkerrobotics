#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────
# CW-1 Single-Command Training Pipeline — macOS Apple Silicon
#
# Usage:
#   ./run.sh                     # Full pipeline, 2M steps
#   ./run.sh --steps 500000      # Shorter run
#   ./run.sh --skip-video        # Skip video render + open
#   ./run.sh --cpu-only          # Force CPU (no MPS)
# ─────────────────────────────────────────────────────────────
set -euo pipefail

# ── ANSI colors ──────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
RESET='\033[0m'

ok()   { echo -e "${GREEN}✅ $*${RESET}"; }
fail() { echo -e "${RED}❌ $*${RESET}"; }
info() { echo -e "${BLUE}ℹ️  $*${RESET}"; }
warn() { echo -e "${YELLOW}⚠️  $*${RESET}"; }
step() { echo -e "\n${BOLD}${CYAN}━━━ Step $1: $2 ━━━${RESET}"; }

# ── Parse arguments ──────────────────────────────────────────
STEPS=2000000
SKIP_VIDEO=false
CPU_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --steps)
      STEPS="$2"
      shift 2
      ;;
    --skip-video)
      SKIP_VIDEO=true
      shift
      ;;
    --cpu-only)
      CPU_ONLY=true
      shift
      ;;
    -h|--help)
      echo "Usage: ./run.sh [--steps N] [--skip-video] [--cpu-only]"
      echo ""
      echo "Options:"
      echo "  --steps N       Training timesteps (default: 2000000)"
      echo "  --skip-video    Skip video rendering and playback"
      echo "  --cpu-only      Force CPU training (ignore MPS)"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TRAIN_START="$(date +%s)"

echo -e "${BOLD}"
echo "╔══════════════════════════════════════════════════════════╗"
echo "║      CleanWalker CW-1 — Training Pipeline (macOS)      ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo -e "║  Steps: $(printf '%-10s' "$(printf '%\047d' "$STEPS")")                                   ║"
echo -e "║  Video: $(printf '%-10s' "$( $SKIP_VIDEO && echo 'skip' || echo 'yes' )")                                   ║"
echo -e "║  Device: $(printf '%-9s' "$( $CPU_ONLY && echo 'cpu' || echo 'auto' )")                                   ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo -e "${RESET}"

# ── Estimate total time ─────────────────────────────────────
# Rough estimate: ~60k steps/s on M4 Max MPS, ~20k on CPU
if $CPU_ONLY; then
  EST_TRAIN_MIN=$(( STEPS / 20000 / 60 ))
else
  EST_TRAIN_MIN=$(( STEPS / 60000 / 60 ))
fi
EST_EXTRA_MIN=3  # convert + validate + render
EST_TOTAL_MIN=$(( EST_TRAIN_MIN + EST_EXTRA_MIN ))
info "Estimated total time: ~${EST_TOTAL_MIN} minutes (${EST_TRAIN_MIN}m train + ${EST_EXTRA_MIN}m extras)"

# ── Check Python version ────────────────────────────────────
info "Checking Python..."

PYTHON=""
for candidate in python3.12 python3.11 python3.10 python3; do
  if command -v "$candidate" &>/dev/null; then
    PY_VER="$("$candidate" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    PY_MAJOR="$("$candidate" -c 'import sys; print(sys.version_info.major)')"
    PY_MINOR="$("$candidate" -c 'import sys; print(sys.version_info.minor)')"
    if [[ "$PY_MAJOR" -ge 3 ]] && [[ "$PY_MINOR" -ge 10 ]]; then
      PYTHON="$candidate"
      break
    fi
  fi
done

if [[ -z "$PYTHON" ]]; then
  fail "Python 3.10+ required. Install with: brew install python@3.11"
  exit 1
fi
ok "Found $PYTHON ($PY_VER)"

# ── Create/activate venv ────────────────────────────────────
VENV_DIR="$SCRIPT_DIR/.venv"
if [[ ! -d "$VENV_DIR" ]]; then
  info "Creating virtual environment..."
  "$PYTHON" -m venv "$VENV_DIR"
  ok "Virtual environment created at .venv/"
else
  ok "Virtual environment exists at .venv/"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# ── Install dependencies ────────────────────────────────────
info "Installing dependencies from requirements-macos.txt..."
pip install --quiet --upgrade pip
pip install --quiet -r requirements-macos.txt
ok "Dependencies installed"

# ── Check MPS availability ──────────────────────────────────
DEVICE_ARG=""
if $CPU_ONLY; then
  DEVICE_ARG="--device cpu"
  warn "CPU-only mode (--cpu-only flag set)"
else
  MPS_AVAIL="$(python -c 'import torch; print(torch.backends.mps.is_available())' 2>/dev/null || echo 'False')"
  if [[ "$MPS_AVAIL" == "True" ]]; then
    ok "Apple MPS (Metal) GPU available"
  else
    warn "MPS not available — falling back to CPU"
    warn "For GPU: requires macOS 12.3+ and PyTorch 2.0+"
    DEVICE_ARG="--device cpu"
  fi
fi

# ── Step 1: Convert URDF → MJCF ─────────────────────────────
step "1/5" "Convert URDF → MJCF"

if [[ -f "$SCRIPT_DIR/cw1_scene.xml" ]]; then
  info "cw1_scene.xml already exists, skipping conversion"
  ok "MJCF scene ready"
else
  python convert_urdf_to_mjcf.py
  if [[ -f "$SCRIPT_DIR/cw1_scene.xml" ]]; then
    ok "URDF → MJCF conversion complete"
  else
    fail "MJCF conversion failed — cw1_scene.xml not found"
    exit 1
  fi
fi

# ── Step 2: Train PPO ───────────────────────────────────────
step "2/5" "Train PPO (${STEPS} steps)"

TRAIN_CMD="python train.py --timesteps $STEPS"
if [[ -n "$DEVICE_ARG" ]]; then
  TRAIN_CMD="$TRAIN_CMD $DEVICE_ARG"
fi

echo -e "${DIM}$ $TRAIN_CMD${RESET}"
eval "$TRAIN_CMD"

if [[ -f "$SCRIPT_DIR/cw1_policy_final.zip" ]]; then
  ok "Training complete — cw1_policy_final.zip saved"
else
  fail "Training failed — no final model found"
  exit 1
fi

# ── Step 3: Validate ────────────────────────────────────────
step "3/5" "Validate trained policy"

python validate.py
if [[ -f "$SCRIPT_DIR/validation_results.json" ]]; then
  ok "Validation complete — results saved to validation_results.json"
else
  warn "Validation ran but results file not found"
fi

# ── Step 4: Render demo video ───────────────────────────────
VIDEO_PATH="$SCRIPT_DIR/renders/cw1-walking-demo.mp4"

if $SKIP_VIDEO; then
  step "4/5" "Render demo video (SKIPPED)"
  info "Skipped (--skip-video)"
else
  step "4/5" "Render demo video"
  python render_demo.py
  if [[ -f "$VIDEO_PATH" ]]; then
    ok "Demo video rendered"
  else
    warn "Video rendering may have failed"
  fi
fi

# ── Step 5: Open video ──────────────────────────────────────
if $SKIP_VIDEO; then
  step "5/5" "Open demo video (SKIPPED)"
  info "Skipped (--skip-video)"
else
  step "5/5" "Open demo video"
  if [[ -f "$VIDEO_PATH" ]]; then
    open "$VIDEO_PATH" 2>/dev/null && ok "Opened video in default player" \
      || warn "Could not open video automatically — view at: $VIDEO_PATH"
  else
    warn "No video file to open"
  fi
fi

# ── Final summary ────────────────────────────────────────────
TRAIN_END="$(date +%s)"
ELAPSED=$(( TRAIN_END - TRAIN_START ))
ELAPSED_MIN=$(( ELAPSED / 60 ))
ELAPSED_SEC=$(( ELAPSED % 60 ))

echo ""
echo -e "${BOLD}${GREEN}"
echo "╔══════════════════════════════════════════════════════════╗"
echo "║                 🎉 Pipeline Complete!                   ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo -e "${RESET}"
echo -e "${BOLD}Total time:${RESET}  ${ELAPSED_MIN}m ${ELAPSED_SEC}s"
echo ""
echo -e "${BOLD}Artifacts:${RESET}"
echo -e "  Model:       ${CYAN}$SCRIPT_DIR/cw1_policy_final.zip${RESET}"

if [[ -d "$SCRIPT_DIR/checkpoints" ]]; then
  N_CKPT=$(find "$SCRIPT_DIR/checkpoints" -name '*.zip' 2>/dev/null | wc -l | tr -d ' ')
  echo -e "  Checkpoints: ${CYAN}$SCRIPT_DIR/checkpoints/${RESET} (${N_CKPT} files)"
fi

if [[ -d "$SCRIPT_DIR/logs" ]]; then
  echo -e "  TensorBoard: ${CYAN}$SCRIPT_DIR/logs/${RESET}"
fi

if [[ -f "$SCRIPT_DIR/validation_results.json" ]]; then
  echo -e "  Validation:  ${CYAN}$SCRIPT_DIR/validation_results.json${RESET}"
fi

if [[ -f "$VIDEO_PATH" ]]; then
  SIZE=$(du -h "$VIDEO_PATH" | cut -f1)
  echo -e "  Demo video:  ${CYAN}${VIDEO_PATH}${RESET} (${SIZE})"
fi

echo ""
echo -e "${BOLD}Next steps:${RESET}"
echo "  python play.py                    # Interactive viewer"
echo "  python export_policy.py           # Export to ONNX"
echo "  python upload_model.py --local    # Package for distribution"
echo "  tensorboard --logdir logs/        # Training curves"
