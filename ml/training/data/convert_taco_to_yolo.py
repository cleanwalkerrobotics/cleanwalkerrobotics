# Copyright (c) MB Software Studio LLC. All rights reserved.
# Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"""Convert TACO dataset from COCO JSON format to YOLO format.

Maps TACO's ~60 fine-grained categories into CleanWalker's 10 trash classes
and writes YOLO-format label files alongside a dataset.yaml config.

Usage:
    python convert_taco_to_yolo.py --taco-dir ./TACO --output-dir ./yolo_dataset
"""

import argparse
import json
import random
import shutil
import sys
from pathlib import Path

# CleanWalker class definitions (must match ml/perception/src/config.py)
CLASSES = [
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

# Mapping from TACO category names (lowercase) to CleanWalker class index.
# TACO has ~60 categories; we map them into our 10.
TACO_TO_CLEANWALKER: dict[str, int] = {
    # 0: plastic_bottle
    "plastic bottle": 0,
    "clear plastic bottle": 0,
    "other plastic bottle": 0,
    "plastic bottle cap": 0,
    "water bottle": 0,
    "drink bottle": 0,
    # 1: can
    "drink can": 1,
    "food can": 1,
    "can": 1,
    "aerosol": 1,
    "aluminium foil": 1,
    "metal bottle cap": 1,
    "pop tab": 1,
    "scrap metal": 1,
    # 2: cigarette_butt
    "cigarette": 2,
    "cigarette butt": 2,
    # 3: paper
    "paper": 3,
    "normal paper": 3,
    "tissues": 3,
    "wrapping paper": 3,
    "paper cup": 3,
    "magazine paper": 3,
    "paper bag": 3,
    "receipt": 3,
    "newspaper": 3,
    # 4: plastic_bag
    "plastic bag": 4,
    "single-use carrier bag": 4,
    "polypropylene bag": 4,
    "garbage bag": 4,
    "plastic bag & wrapper": 4,
    "carrier bag": 4,
    "bin bag": 4,
    # 5: food_wrapper
    "food wrapper": 5,
    "crisp packet": 5,
    "snack packet": 5,
    "spread tub": 5,
    "tupperware": 5,
    "meal carton": 5,
    "pizza box": 5,
    "food container": 5,
    "food waste": 5,
    "squeezable tube": 5,
    "sweet wrapper": 5,
    # 6: glass_bottle
    "glass bottle": 6,
    "broken glass": 6,
    "glass cup": 6,
    "glass jar": 6,
    # 7: cardboard
    "cardboard": 7,
    "corrugated carton": 7,
    "egg carton": 7,
    "drink carton": 7,
    "toilet tube": 7,
    "carton": 7,
    # 8: styrofoam
    "styrofoam piece": 8,
    "styrofoam": 8,
    "foam cup": 8,
    "foam food container": 8,
    "foam": 8,
    # 9: other_trash
    "other": 9,
    "shoe": 9,
    "rope": 9,
    "rope & string": 9,
    "battery": 9,
    "blister pack": 9,
    "carded blister pack": 9,
    "other plastic": 9,
    "other plastic wrapper": 9,
    "plastic film": 9,
    "six pack rings": 9,
    "plastic straw": 9,
    "straw": 9,
    "plastic utensils": 9,
    "plastic lid": 9,
    "lid": 9,
    "cup": 9,
    "disposable food container": 9,
    "disposable plastic cup": 9,
    "plastic container": 9,
    "plastic glooves": 9,
    "plastic gloves": 9,
    "rubber gloves": 9,
    "mask": 9,
    "unlabeled litter": 9,
}


def map_category(name: str) -> int:
    """Map a TACO category name to a CleanWalker class index."""
    key = name.strip().lower()
    if key in TACO_TO_CLEANWALKER:
        return TACO_TO_CLEANWALKER[key]
    # Fuzzy matching: check if any known key is a substring
    for known, idx in TACO_TO_CLEANWALKER.items():
        if known in key or key in known:
            return idx
    return 9  # Default: other_trash


def convert_bbox_coco_to_yolo(
    img_width: int, img_height: int, bbox: list[float]
) -> tuple[float, float, float, float]:
    """Convert COCO bbox [x, y, w, h] to YOLO [cx, cy, w, h] normalized."""
    x, y, w, h = bbox
    cx = (x + w / 2) / img_width
    cy = (y + h / 2) / img_height
    nw = w / img_width
    nh = h / img_height
    # Clamp to [0, 1]
    cx = max(0.0, min(1.0, cx))
    cy = max(0.0, min(1.0, cy))
    nw = max(0.0, min(1.0, nw))
    nh = max(0.0, min(1.0, nh))
    return cx, cy, nw, nh


def convert(taco_dir: Path, output_dir: Path, val_split: float = 0.2, seed: int = 42) -> None:
    """Convert TACO dataset to YOLO format with train/val split."""
    annotations_file = taco_dir / "data" / "annotations.json"
    if not annotations_file.exists():
        # Try alternative location
        annotations_file = taco_dir / "annotations.json"
    if not annotations_file.exists():
        print(f"Error: annotations.json not found in {taco_dir}/data/ or {taco_dir}/")
        sys.exit(1)

    print(f"Loading annotations from {annotations_file}")
    with open(annotations_file) as f:
        coco = json.load(f)

    # Build category lookup
    cat_lookup: dict[int, str] = {}
    for cat in coco["categories"]:
        cat_lookup[cat["id"]] = cat["name"]

    # Build image lookup
    img_lookup: dict[int, dict] = {}
    for img in coco["images"]:
        img_lookup[img["id"]] = img

    # Group annotations by image
    img_annotations: dict[int, list] = {}
    skipped = 0
    for ann in coco["annotations"]:
        img_id = ann["image_id"]
        cat_name = cat_lookup.get(ann["category_id"], "other")
        bbox = ann.get("bbox")
        if not bbox or len(bbox) != 4:
            skipped += 1
            continue
        if any(v <= 0 for v in bbox[2:4]):  # skip zero-area boxes
            skipped += 1
            continue
        if img_id not in img_annotations:
            img_annotations[img_id] = []
        img_annotations[img_id].append((cat_name, bbox))

    print(f"Found {len(img_annotations)} images with annotations ({skipped} annotations skipped)")

    # Create output directory structure
    for split in ("train", "val"):
        (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    # Split images into train/val
    random.seed(seed)
    image_ids = sorted(img_annotations.keys())
    random.shuffle(image_ids)
    val_count = max(1, int(len(image_ids) * val_split))
    val_ids = set(image_ids[:val_count])
    train_ids = set(image_ids[val_count:])

    print(f"Split: {len(train_ids)} train, {len(val_ids)} val")

    class_counts = [0] * len(CLASSES)
    total_annotations = 0

    for img_id in image_ids:
        img_info = img_lookup.get(img_id)
        if img_info is None:
            continue

        split = "val" if img_id in val_ids else "train"
        img_w = img_info["width"]
        img_h = img_info["height"]

        # Find and copy image file
        file_name = img_info["file_name"]
        src_img = taco_dir / "data" / file_name
        if not src_img.exists():
            src_img = taco_dir / file_name
        if not src_img.exists():
            continue

        # Use a flat filename to avoid subdirectory issues
        safe_name = file_name.replace("/", "_").replace("\\", "_")
        stem = Path(safe_name).stem
        suffix = Path(safe_name).suffix

        dst_img = output_dir / "images" / split / f"{stem}{suffix}"
        shutil.copy2(src_img, dst_img)

        # Write YOLO label file
        label_path = output_dir / "labels" / split / f"{stem}.txt"
        lines = []
        for cat_name, bbox in img_annotations[img_id]:
            cls_idx = map_category(cat_name)
            cx, cy, nw, nh = convert_bbox_coco_to_yolo(img_w, img_h, bbox)
            lines.append(f"{cls_idx} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}")
            class_counts[cls_idx] += 1
            total_annotations += 1

        with open(label_path, "w") as f:
            f.write("\n".join(lines) + "\n")

    # Write dataset.yaml
    yaml_path = output_dir / "dataset.yaml"
    yaml_content = (
        f"# CleanWalker TACO Dataset Configuration\n"
        f"# Auto-generated by convert_taco_to_yolo.py\n\n"
        f"path: {output_dir.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n\n"
        f"# Number of classes\n"
        f"nc: {len(CLASSES)}\n\n"
        f"# Class names\n"
        f"names:\n"
    )
    for i, name in enumerate(CLASSES):
        yaml_content += f"  {i}: {name}\n"

    with open(yaml_path, "w") as f:
        f.write(yaml_content)

    # Print summary
    print(f"\nConversion complete!")
    print(f"  Total annotations: {total_annotations}")
    print(f"  Dataset YAML:      {yaml_path}")
    print(f"\nClass distribution:")
    for i, name in enumerate(CLASSES):
        print(f"  {i:2d}. {name:20s} {class_counts[i]:5d}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert TACO to YOLO format")
    parser.add_argument(
        "--taco-dir", type=str, default="./TACO",
        help="Path to cloned TACO repository",
    )
    parser.add_argument(
        "--output-dir", type=str, default="./yolo_dataset",
        help="Output directory for YOLO-format dataset",
    )
    parser.add_argument(
        "--val-split", type=float, default=0.2,
        help="Fraction of images for validation (default: 0.2)",
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="Random seed for train/val split",
    )
    args = parser.parse_args()
    convert(Path(args.taco_dir), Path(args.output_dir), args.val_split, args.seed)


if __name__ == "__main__":
    main()
