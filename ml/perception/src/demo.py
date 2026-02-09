# Copyright (c) MB Software Studio LLC. All rights reserved.
# Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"""Demo script for running CleanWalker trash detection on images or video.

Usage:
    # Single image
    python -m perception.src.demo --input photo.jpg

    # Video file
    python -m perception.src.demo --input video.mp4

    # Directory of images
    python -m perception.src.demo --input ./test_images/

    # Custom model and threshold
    python -m perception.src.demo --input photo.jpg --model weights/best.pt --conf 0.3
"""

import argparse
import logging
import sys
from pathlib import Path

import cv2
import numpy as np

from .config import TRASH_CLASSES
from .detect import Detection, TrashDetector

logger = logging.getLogger("cleanwalker.demo")

# Color palette for each class (BGR)
CLASS_COLORS = [
    (255, 100, 0),    # plastic_bottle - blue
    (0, 200, 255),    # can - yellow
    (0, 0, 255),      # cigarette_butt - red
    (200, 200, 200),  # paper - light gray
    (255, 0, 255),    # plastic_bag - magenta
    (0, 165, 255),    # food_wrapper - orange
    (0, 255, 0),      # glass_bottle - green
    (100, 100, 255),  # cardboard - salmon
    (255, 255, 0),    # styrofoam - cyan
    (128, 128, 128),  # other_trash - gray
]


def draw_detections(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """Draw bounding boxes and labels on a frame.

    Args:
        frame: Input BGR image.
        detections: List of Detection objects.

    Returns:
        Annotated image.
    """
    annotated = frame.copy()

    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det.bbox]

        # Get color for this class
        try:
            cls_idx = TRASH_CLASSES.index(det.class_name)
            color = CLASS_COLORS[cls_idx]
        except (ValueError, IndexError):
            color = (128, 128, 128)

        # Draw bounding box
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

        # Draw label background
        label = f"{det.class_name} {det.confidence:.2f}"
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(annotated, (x1, y1 - th - baseline - 4), (x1 + tw, y1), color, -1)

        # Draw label text
        cv2.putText(
            annotated, label, (x1, y1 - baseline - 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )

    return annotated


def process_image(detector: TrashDetector, image_path: Path, output_dir: Path | None) -> None:
    """Run detection on a single image and display/save results."""
    frame = cv2.imread(str(image_path))
    if frame is None:
        logger.error("Could not read image: %s", image_path)
        return

    detections = detector.detect(frame)
    logger.info("%s: %d detections", image_path.name, len(detections))
    for det in detections:
        logger.info("  %s (%.2f) at [%d,%d,%d,%d]",
                     det.class_name, det.confidence,
                     int(det.bbox[0]), int(det.bbox[1]),
                     int(det.bbox[2]), int(det.bbox[3]))

    annotated = draw_detections(frame, detections)

    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        out_path = output_dir / f"det_{image_path.name}"
        cv2.imwrite(str(out_path), annotated)
        logger.info("Saved: %s", out_path)
    else:
        cv2.imshow("CleanWalker Detection", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def process_video(detector: TrashDetector, video_path: Path, output_dir: Path | None) -> None:
    """Run detection on a video file frame by frame."""
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        logger.error("Could not open video: %s", video_path)
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    logger.info("Video: %dx%d @ %.1f fps, %d frames", width, height, fps, total_frames)

    writer = None
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        out_path = output_dir / f"det_{video_path.name}"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(out_path), fourcc, fps, (width, height))
        logger.info("Writing output to %s", out_path)

    frame_num = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_num += 1
            detections = detector.detect(frame)
            annotated = draw_detections(frame, detections)

            if frame_num % 30 == 0:
                logger.info("Frame %d/%d: %d detections",
                             frame_num, total_frames, len(detections))

            if writer:
                writer.write(annotated)
            else:
                cv2.imshow("CleanWalker Detection", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    logger.info("Stopped by user")
                    break
    finally:
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

    logger.info("Processed %d frames", frame_num)


def main() -> None:
    parser = argparse.ArgumentParser(description="CleanWalker trash detection demo")
    parser.add_argument("--input", type=str, required=True,
                        help="Path to image, video, or directory of images")
    parser.add_argument("--model", type=str, default=None,
                        help="Path to model weights (default: perception/weights/best.pt)")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Confidence threshold (default: 0.5)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output directory for annotated results (default: display on screen)")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    detector = TrashDetector(model_path=args.model, confidence_threshold=args.conf)
    detector.load_model()

    input_path = Path(args.input)
    output_dir = Path(args.output) if args.output else None

    if input_path.is_dir():
        # Process all images in directory
        image_exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}
        images = sorted(p for p in input_path.iterdir() if p.suffix.lower() in image_exts)
        if not images:
            logger.error("No images found in %s", input_path)
            sys.exit(1)
        logger.info("Processing %d images from %s", len(images), input_path)
        for img_path in images:
            process_image(detector, img_path, output_dir)
    elif input_path.suffix.lower() in {".mp4", ".avi", ".mov", ".mkv", ".webm"}:
        process_video(detector, input_path, output_dir)
    elif input_path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}:
        process_image(detector, input_path, output_dir)
    else:
        logger.error("Unsupported file type: %s", input_path.suffix)
        sys.exit(1)


if __name__ == "__main__":
    main()
