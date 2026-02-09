# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2026 CleanWalker Robotics

"""Configuration constants for the perception pipeline."""

# Model configuration
MODEL_INPUT_SIZE = (640, 640)
DEFAULT_CONFIDENCE_THRESHOLD = 0.5
NMS_IOU_THRESHOLD = 0.45

# Trash classes the model is trained to detect
TRASH_CLASSES = [
    "plastic_bottle",
    "can",
    "cigarette_butt",
    "paper",
    "plastic_bag",
    "food_wrapper",
    "glass_bottle",
    "cardboard",
    "styrofoam",
    "other_trash",
]

# Model weights path
DEFAULT_MODEL_PATH = "weights/best.pt"
