#!/usr/bin/env python3
# Copyright (c) MB Software Studio LLC. All rights reserved.
# Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"""Proof-of-concept: Real litter detection via Replicate API.

Uses the open-vocabulary YOLOv8s-WorldV2 model to detect litter in images.
The model accepts custom class names, so we specify our 10 litter categories.

Usage:
    export REPLICATE_API_TOKEN="your-token-here"
    python ml/poc/replicate_detect.py --image "https://example.com/photo.jpg"
    python ml/poc/replicate_detect.py --image "https://example.com/photo.jpg" --conf 0.3
"""

import argparse
import json
import os
import sys
import time
import urllib.request
import urllib.error

# CleanWalker litter classes (matches ml/perception/src/config.py)
LITTER_CLASSES = (
    "plastic bottle, can, cigarette butt, paper cup, plastic bag, "
    "food wrapper, glass bottle, cardboard, styrofoam, trash, litter, "
    "garbage, waste, debris, wrapper, disposable cup"
)

REPLICATE_API_URL = "https://api.replicate.com/v1/predictions"

# YOLOv8s-WorldV2: open-vocabulary YOLO that accepts custom class names
MODEL_VERSION = "d28fb092d84b252971af3f98fe3138bf75d69508edb80b5237eec9c4f2f77f27"


def run_detection(
    image_url: str,
    confidence: float = 0.25,
    api_token: str | None = None,
) -> dict:
    """Run litter detection on an image via Replicate API.

    Args:
        image_url: Public URL of the image to analyze.
        confidence: Minimum confidence threshold (0-1).
        api_token: Replicate API token. Falls back to REPLICATE_API_TOKEN env var.

    Returns:
        Dict with 'detections' list and 'annotated_image' URL.
    """
    token = api_token or os.environ.get("REPLICATE_API_TOKEN")
    if not token:
        raise ValueError(
            "No API token provided. Set REPLICATE_API_TOKEN environment variable "
            "or pass api_token parameter."
        )

    payload = json.dumps({
        "version": MODEL_VERSION,
        "input": {
            "image": image_url,
            "conf": confidence,
            "iou": 0.45,
            "imgsz": 640,
            "class_names": LITTER_CLASSES,
            "return_json": True,
        },
    }).encode("utf-8")

    # Create prediction (sync mode — waits up to 60s for result)
    req = urllib.request.Request(
        REPLICATE_API_URL,
        data=payload,
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
            "Prefer": "wait",
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=90) as resp:
            result = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"Replicate API error {e.code}: {body}") from e

    # If prediction is still processing (sync timeout), poll for completion
    status = result.get("status", "")
    if status not in ("succeeded", "failed", "canceled"):
        result = _poll_prediction(result["id"], token)

    if result.get("status") == "failed":
        raise RuntimeError(f"Prediction failed: {result.get('error', 'unknown error')}")

    return _parse_output(result.get("output", {}))


def _poll_prediction(prediction_id: str, token: str, max_wait: int = 120) -> dict:
    """Poll a Replicate prediction until it completes."""
    url = f"{REPLICATE_API_URL}/{prediction_id}"
    req = urllib.request.Request(
        url,
        headers={"Authorization": f"Bearer {token}"},
        method="GET",
    )

    start = time.time()
    while time.time() - start < max_wait:
        time.sleep(2)
        with urllib.request.urlopen(req, timeout=30) as resp:
            result = json.loads(resp.read().decode("utf-8"))

        status = result.get("status", "")
        if status in ("succeeded", "failed", "canceled"):
            return result

    raise TimeoutError(f"Prediction {prediction_id} did not complete within {max_wait}s")


def _parse_output(output: dict | list | str) -> dict:
    """Parse Replicate model output into a standardized format."""
    detections = []
    annotated_image = None

    # Output can be a dict with image + json_str keys
    if isinstance(output, dict):
        annotated_image = output.get("image")
        json_str = output.get("json_str", "")
        if json_str:
            try:
                parsed = json.loads(json_str) if isinstance(json_str, str) else json_str
                if isinstance(parsed, dict):
                    detections = parsed.get("detections", [])
                elif isinstance(parsed, list):
                    detections = parsed
            except (json.JSONDecodeError, TypeError):
                pass

    # Output can also be a list (some model versions)
    elif isinstance(output, list) and len(output) >= 2:
        annotated_image = output[0] if isinstance(output[0], str) else None
        if isinstance(output[1], str):
            try:
                parsed = json.loads(output[1])
                if isinstance(parsed, list):
                    detections = parsed
                elif isinstance(parsed, dict):
                    detections = parsed.get("detections", [])
            except (json.JSONDecodeError, TypeError):
                pass

    return {
        "detections": detections,
        "annotated_image": annotated_image,
        "detection_count": len(detections),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="CleanWalker litter detection PoC via Replicate API"
    )
    parser.add_argument(
        "--image", type=str, required=True,
        help="Public URL of the image to analyze",
    )
    parser.add_argument(
        "--conf", type=float, default=0.25,
        help="Confidence threshold (default: 0.25)",
    )
    args = parser.parse_args()

    print(f"Detecting litter in: {args.image}")
    print(f"Confidence threshold: {args.conf}")
    print(f"Classes: {LITTER_CLASSES}")
    print("Running inference...")
    print()

    try:
        result = run_detection(args.image, confidence=args.conf)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Detections: {result['detection_count']}")
    print()

    if result["detections"]:
        print("Results:")
        print("-" * 60)
        for det in result["detections"]:
            cls = str(det.get("name", det.get("class", det.get("label", "unknown"))))
            conf = float(det.get("confidence", det.get("score", 0)))
            bbox = det.get("box", det.get("bbox", []))
            print(f"  {cls:20s}  conf={conf:.3f}  bbox={bbox}")
        print("-" * 60)
    else:
        print("No litter detected (try lowering --conf threshold)")

    if result["annotated_image"]:
        print(f"\nAnnotated image: {result['annotated_image']}")

    # Output full JSON for programmatic use
    print(f"\nFull JSON output:")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
