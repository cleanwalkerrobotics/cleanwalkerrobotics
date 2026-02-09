# Copyright (c) MB Software Studio LLC. All rights reserved.
# Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"""Training script for the CleanWalker trash detection model.

Fine-tunes YOLOv8n on the TACO dataset mapped to 10 CleanWalker trash classes.
Supports GPU and CPU training with configurable hyperparameters.

Usage:
    python -m training.src.train --data training/data/dataset.yaml
    python -m training.src.train --data training/data/dataset.yaml --device cpu --epochs 50
"""

import argparse
import logging
import shutil
import sys
from pathlib import Path

from ultralytics import YOLO

# Resolve paths relative to the ml/ directory
ML_ROOT = Path(__file__).resolve().parent.parent.parent
WEIGHTS_DIR = ML_ROOT / "perception" / "weights"

logger = logging.getLogger("cleanwalker.train")


def setup_logging(verbose: bool = False) -> None:
    """Configure logging for training."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler(sys.stdout)],
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train the CleanWalker trash detection model")
    parser.add_argument("--data", type=str, required=True, help="Path to dataset YAML config")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Base model to fine-tune")
    parser.add_argument("--epochs", type=int, default=100, help="Number of training epochs")
    parser.add_argument("--batch-size", type=int, default=16, help="Batch size")
    parser.add_argument("--img-size", type=int, default=640, help="Input image size")
    parser.add_argument("--lr", type=float, default=0.01, help="Initial learning rate")
    parser.add_argument("--project", type=str, default="cleanwalker", help="W&B project name")
    parser.add_argument("--name", type=str, default="trash-detect", help="Run name")
    parser.add_argument("--device", type=str, default="0", help="CUDA device (e.g. 0 or cpu)")
    parser.add_argument("--workers", type=int, default=4, help="Number of data loader workers")
    parser.add_argument("--patience", type=int, default=20, help="Early stopping patience (epochs)")
    parser.add_argument("--resume", action="store_true", help="Resume training from last checkpoint")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser.parse_args()


def validate_dataset(data_path: str) -> Path:
    """Validate that the dataset YAML config exists and is readable."""
    path = Path(data_path).resolve()
    if not path.exists():
        logger.error("Dataset config not found: %s", path)
        logger.info(
            "Run the TACO download script first:\n"
            "  cd ml/training/data && bash download_taco.sh"
        )
        sys.exit(1)
    logger.info("Dataset config: %s", path)
    return path


def detect_device(requested: str) -> str:
    """Resolve the training device, falling back to CPU if CUDA unavailable."""
    if requested == "cpu":
        logger.info("Using CPU for training (this will be slow)")
        return "cpu"

    try:
        import torch

        if torch.cuda.is_available():
            gpu_name = torch.cuda.get_device_name(int(requested))
            logger.info("Using GPU %s: %s", requested, gpu_name)
            return requested
        else:
            logger.warning("CUDA not available, falling back to CPU")
            return "cpu"
    except (ImportError, RuntimeError, ValueError):
        logger.warning("Could not detect GPU, falling back to CPU")
        return "cpu"


def copy_best_weights(train_dir: Path) -> None:
    """Copy the best model weights to the perception weights directory."""
    best_pt = train_dir / "weights" / "best.pt"
    if not best_pt.exists():
        logger.warning("best.pt not found at %s", best_pt)
        return

    WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
    dest = WEIGHTS_DIR / "best.pt"
    shutil.copy2(best_pt, dest)
    logger.info("Best weights saved to %s", dest)


def train(args: argparse.Namespace) -> None:
    """Run model training.

    Args:
        args: Parsed command-line arguments.
    """
    setup_logging(args.verbose)

    logger.info("=" * 60)
    logger.info("CleanWalker Trash Detection — Training")
    logger.info("=" * 60)

    # Validate dataset
    data_path = validate_dataset(args.data)

    # Resolve device
    device = detect_device(args.device)

    # Load base model
    if args.resume:
        # Resume from last checkpoint
        last_pt = Path(args.project) / args.name / "weights" / "last.pt"
        if last_pt.exists():
            logger.info("Resuming training from %s", last_pt)
            model = YOLO(str(last_pt))
        else:
            logger.error("No checkpoint found at %s to resume from", last_pt)
            sys.exit(1)
    else:
        logger.info("Loading base model: %s", args.model)
        model = YOLO(args.model)

    # Log training configuration
    logger.info("Training configuration:")
    logger.info("  Dataset:       %s", data_path)
    logger.info("  Base model:    %s", args.model)
    logger.info("  Epochs:        %d", args.epochs)
    logger.info("  Batch size:    %d", args.batch_size)
    logger.info("  Image size:    %d", args.img_size)
    logger.info("  Learning rate: %f", args.lr)
    logger.info("  Device:        %s", device)
    logger.info("  Workers:       %d", args.workers)
    logger.info("  Patience:      %d", args.patience)

    # Run training
    results = model.train(
        data=str(data_path),
        epochs=args.epochs,
        batch=args.batch_size,
        imgsz=args.img_size,
        lr0=args.lr,
        device=device,
        workers=args.workers,
        patience=args.patience,
        project=args.project,
        name=args.name,
        exist_ok=True,
        pretrained=True,
        verbose=args.verbose,
        save=True,
        save_period=10,
        val=True,
    )

    # Copy best weights to perception module
    train_dir = Path(args.project) / args.name
    copy_best_weights(train_dir)

    # Log final metrics
    logger.info("=" * 60)
    logger.info("Training complete!")
    if results and hasattr(results, "results_dict"):
        metrics = results.results_dict
        logger.info("Final metrics:")
        for key, value in metrics.items():
            if isinstance(value, float):
                logger.info("  %s: %.4f", key, value)
    logger.info("Results saved to %s", train_dir)
    logger.info("Best weights at %s", WEIGHTS_DIR / "best.pt")
    logger.info("=" * 60)


if __name__ == "__main__":
    train(parse_args())
