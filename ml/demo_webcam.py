# Copyright (c) MB Software Studio LLC. All rights reserved.
# Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"""Live webcam / video demo for CleanWalker trash detection.

Processes a webcam feed or video file, draws bounding boxes around detected
trash, and displays the annotated stream in real time.

Usage:
    # Webcam (default camera)
    python demo_webcam.py

    # Specific camera index
    python demo_webcam.py --source 1

    # Video file
    python demo_webcam.py --source path/to/video.mp4

    # Save output to file
    python demo_webcam.py --source video.mp4 --save output.mp4

    # Custom model and confidence
    python demo_webcam.py --model perception/weights/best.pt --conf 0.3
"""

import argparse
import logging
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Add ml/ directory to path so we can import perception
sys.path.insert(0, str(Path(__file__).resolve().parent))

from perception.src.config import TRASH_CLASSES
from perception.src.detect import Detection, TrashDetector

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("cleanwalker.webcam")

# Color palette for each class (BGR)
CLASS_COLORS = [
    (255, 100, 0),    # plastic_bottle
    (0, 200, 255),    # can
    (0, 0, 255),      # cigarette_butt
    (200, 200, 200),  # paper
    (255, 0, 255),    # plastic_bag
    (0, 165, 255),    # food_wrapper
    (0, 255, 0),      # glass_bottle
    (100, 100, 255),  # cardboard
    (255, 255, 0),    # styrofoam
    (128, 128, 128),  # other_trash
]


def draw_detections(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """Draw bounding boxes, labels, and detection count on a frame."""
    annotated = frame.copy()

    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det.bbox]

        try:
            cls_idx = TRASH_CLASSES.index(det.class_name)
            color = CLASS_COLORS[cls_idx]
        except (ValueError, IndexError):
            color = (128, 128, 128)

        # Bounding box
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

        # Label
        label = f"{det.class_name} {det.confidence:.2f}"
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(annotated, (x1, y1 - th - baseline - 4), (x1 + tw, y1), color, -1)
        cv2.putText(
            annotated, label, (x1, y1 - baseline - 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )

    return annotated


def draw_hud(
    frame: np.ndarray,
    detections: list[Detection],
    fps: float,
) -> np.ndarray:
    """Draw a heads-up display overlay with detection stats and FPS."""
    h, w = frame.shape[:2]

    # Semi-transparent header bar
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 36), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    # FPS counter
    cv2.putText(
        frame, f"FPS: {fps:.1f}", (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA,
    )

    # Detection count
    count_text = f"Detections: {len(detections)}"
    cv2.putText(
        frame, count_text, (w - 200, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA,
    )

    # Title
    cv2.putText(
        frame, "CleanWalker Trash Detection", (w // 2 - 150, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA,
    )

    return frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CleanWalker live trash detection demo")
    parser.add_argument("--source", type=str, default="0",
                        help="Video source: camera index (0, 1, ...) or video file path")
    parser.add_argument("--model", type=str, default=None,
                        help="Path to model weights (default: perception/weights/best.pt)")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Confidence threshold (default: 0.5)")
    parser.add_argument("--save", type=str, default=None,
                        help="Path to save output video (e.g. output.mp4)")
    parser.add_argument("--no-display", action="store_true",
                        help="Disable live display (useful for headless servers)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Load detector
    logger.info("Loading trash detection model...")
    detector = TrashDetector(model_path=args.model, confidence_threshold=args.conf)
    detector.load_model()
    logger.info("Model loaded successfully")

    # Open video source
    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        logger.error("Could not open video source: %s", args.source)
        sys.exit(1)

    fps_video = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    is_webcam = isinstance(source, int)

    logger.info("Source: %s (%dx%d @ %.1f fps)",
                "webcam" if is_webcam else args.source, width, height, fps_video)

    # Set up video writer if saving
    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.save, fourcc, fps_video, (width, height))
        logger.info("Saving output to %s", args.save)

    logger.info("Press 'q' to quit, 's' to save a screenshot")
    logger.info("Starting detection loop...")

    frame_count = 0
    fps_display = 0.0
    t_prev = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                if is_webcam:
                    logger.error("Lost webcam connection")
                break

            frame_count += 1

            # Run detection
            detections = detector.detect(frame)

            # Draw results
            annotated = draw_detections(frame, detections)

            # Calculate FPS
            t_now = time.time()
            dt = t_now - t_prev
            if dt > 0:
                fps_display = 0.9 * fps_display + 0.1 * (1.0 / dt)
            t_prev = t_now

            # Draw HUD
            annotated = draw_hud(annotated, detections, fps_display)

            # Save frame
            if writer:
                writer.write(annotated)

            # Display
            if not args.no_display:
                cv2.imshow("CleanWalker Trash Detection", annotated)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    logger.info("Quit requested")
                    break
                elif key == ord("s"):
                    screenshot_path = f"screenshot_{frame_count:06d}.jpg"
                    cv2.imwrite(screenshot_path, annotated)
                    logger.info("Screenshot saved: %s", screenshot_path)

            # Log periodically
            if frame_count % 100 == 0:
                logger.info("Frame %d | FPS: %.1f | Detections: %d",
                             frame_count, fps_display, len(detections))

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

    logger.info("Processed %d frames. Done.", frame_count)


if __name__ == "__main__":
    main()
