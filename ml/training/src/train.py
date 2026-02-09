# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2026 CleanWalker Robotics

"""Training script for the CleanWalker trash detection model."""

import argparse


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
    return parser.parse_args()


def train(args: argparse.Namespace) -> None:
    """Run model training.

    Args:
        args: Parsed command-line arguments.
    """
    # TODO: Initialize ultralytics YOLO model and run training
    print(f"Training with dataset: {args.data}")
    print(f"Base model: {args.model}")
    print(f"Epochs: {args.epochs}, Batch size: {args.batch_size}")


if __name__ == "__main__":
    train(parse_args())
