# Copyright (c) MB Software Studio LLC. All rights reserved.
# Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"""Trash detection pipeline using YOLOv8."""

import logging
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from ultralytics import YOLO

from .config import DEFAULT_CONFIDENCE_THRESHOLD, DEFAULT_MODEL_PATH, TRASH_CLASSES

logger = logging.getLogger("cleanwalker.detect")


@dataclass
class Detection:
    """A single trash detection result."""

    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]  # x1, y1, x2, y2


class TrashDetector:
    """Detects trash objects in camera frames using a YOLO model."""

    def __init__(
        self,
        model_path: str | None = None,
        confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD,
    ):
        if model_path is None:
            # Resolve default weights relative to this file's package
            model_path = str(Path(__file__).resolve().parent.parent / DEFAULT_MODEL_PATH)
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model: YOLO | None = None
        self.class_names = TRASH_CLASSES

    def load_model(self) -> None:
        """Load the YOLO model from disk.

        Raises:
            FileNotFoundError: If the model weights file does not exist.
        """
        path = Path(self.model_path)
        if not path.exists():
            raise FileNotFoundError(
                f"Model weights not found at {path}. "
                "Train the model first or download pre-trained weights."
            )

        logger.info("Loading model from %s", path)
        self.model = YOLO(str(path))
        logger.info("Model loaded successfully (%d classes)", len(self.class_names))

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Run trash detection on a single frame.

        Args:
            frame: Input image as numpy array (BGR format, HxWxC).

        Returns:
            List of Detection objects found in the frame.
        """
        if self.model is None:
            self.load_model()

        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        detections: list[Detection] = []

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()

                # Map class ID to our class name
                if cls_id < len(self.class_names):
                    class_name = self.class_names[cls_id]
                else:
                    class_name = "other_trash"

                detections.append(
                    Detection(
                        class_name=class_name,
                        confidence=conf,
                        bbox=(x1, y1, x2, y2),
                    )
                )

        return detections

    def detect_batch(self, frames: list[np.ndarray]) -> list[list[Detection]]:
        """Run detection on a batch of frames.

        Args:
            frames: List of input images as numpy arrays (BGR format).

        Returns:
            List of detection lists, one per input frame.
        """
        if self.model is None:
            self.load_model()

        results = self.model(frames, conf=self.confidence_threshold, verbose=False)
        batch_detections: list[list[Detection]] = []

        for result in results:
            detections: list[Detection] = []
            boxes = result.boxes
            if boxes is not None:
                for i in range(len(boxes)):
                    cls_id = int(boxes.cls[i].item())
                    conf = float(boxes.conf[i].item())
                    x1, y1, x2, y2 = boxes.xyxy[i].tolist()

                    if cls_id < len(self.class_names):
                        class_name = self.class_names[cls_id]
                    else:
                        class_name = "other_trash"

                    detections.append(
                        Detection(
                            class_name=class_name,
                            confidence=conf,
                            bbox=(x1, y1, x2, y2),
                        )
                    )
            batch_detections.append(detections)

        return batch_detections
