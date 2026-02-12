# ml — Codebase Manifest

All models, training configs, perception scripts, and locomotion modules owned by the ML team.
Every file added or removed MUST be reflected here. Build will fail on untracked files.

## Top Level

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `README.md` | ML division overview and setup instructions | active | 2026-02-12 |
| `demo_webcam.py` | Webcam-based litter detection demo script | active | 2026-02-12 |

## Perception Pipeline

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `perception/requirements.txt` | Python dependencies for perception | active | 2026-02-12 |
| `perception/src/__init__.py` | Perception package init | active | 2026-02-12 |
| `perception/src/config.py` | Model paths, thresholds, class labels config | active | 2026-02-12 |
| `perception/src/detect.py` | Core YOLO detection inference logic | active | 2026-02-12 |
| `perception/src/demo.py` | Perception demo runner with visualization | active | 2026-02-12 |

## Training Pipeline

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `training/requirements.txt` | Python dependencies for training | active | 2026-02-12 |
| `training/src/__init__.py` | Training package init | active | 2026-02-12 |
| `training/src/train.py` | YOLO model training script (fine-tuning) | active | 2026-02-12 |
| `training/data/convert_taco_to_yolo.py` | TACO dataset → YOLO format converter | active | 2026-02-12 |
| `training/data/download_taco.sh` | TACO dataset download script | active | 2026-02-12 |

## Proof of Concept

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `poc/replicate_detect.py` | Replicate API-based detection POC | active | 2026-02-12 |

## Locomotion (RL Policy Training)

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `locomotion/README.md` | Locomotion training overview and architecture | active | 2026-02-12 |
| `locomotion/QUICKSTART.md` | One-click cloud GPU training quickstart | active | 2026-02-12 |
| `locomotion/__init__.py` | Locomotion package init and exports | active | 2026-02-12 |
| `locomotion/cleanwalker_env.py` | IsaacGym environment for CW-1 quadruped | active | 2026-02-12 |
| `locomotion/train.py` | RL policy training entry point | active | 2026-02-12 |
| `locomotion/train_config.py` | Training hyperparameters and schedule | active | 2026-02-12 |
| `locomotion/terrain_config.py` | Terrain generation config for curriculum | active | 2026-02-12 |
| `locomotion/rewards.py` | Reward function definitions for locomotion | active | 2026-02-12 |
| `locomotion/play.py` | Policy evaluation and visualization | active | 2026-02-12 |
| `locomotion/export_policy.py` | Export trained policy to ONNX/TorchScript | active | 2026-02-12 |
| `locomotion/convert_urdf.py` | URDF to IsaacGym asset converter | active | 2026-02-12 |
| `locomotion/docker-compose.yml` | Docker setup for GPU training environment | active | 2026-02-12 |
| `locomotion/Dockerfile` | Dockerfile for locomotion training | active | 2026-02-12 |
| `locomotion/scripts/download-model.sh` | Pre-trained model download script | active | 2026-02-12 |
