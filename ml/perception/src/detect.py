# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2026 CleanWalker Robotics

"""Trash detection pipeline using YOLOv8."""

from dataclasses import dataclass

from .config import DEFAULT_CONFIDENCE_THRESHOLD, TRASH_CLASSES


@dataclass
class Detection:
    """A single trash detection result."""

    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]  # x1, y1, x2, y2


class TrashDetector:
    """Detects trash objects in camera frames using a YOLO model."""

    def __init__(self, model_path: str | None = None, confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model = None

    def load_model(self) -> None:
        """Load the YOLO model from disk."""
        # TODO: Load ultralytics YOLO model
        pass

    def detect(self, frame) -> list[Detection]:
        """Run trash detection on a single frame.

        Args:
            frame: Input image as numpy array (BGR format).

        Returns:
            List of Detection objects found in the frame.
        """
        # TODO: Run inference and return detections
        return []
