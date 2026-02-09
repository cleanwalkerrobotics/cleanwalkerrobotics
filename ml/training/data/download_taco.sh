#!/usr/bin/env bash
# Copyright (c) MB Software Studio LLC. All rights reserved.
# Licensed under the AGPL-3.0 License. See LICENSE in the project root.

# Download and prepare the TACO dataset in YOLO format for CleanWalker training.
#
# This script:
#   1. Clones the TACO repository
#   2. Downloads all dataset images via TACO's download script
#   3. Converts COCO annotations to YOLO format with our 10-class mapping
#   4. Creates train/val split and dataset.yaml
#
# Usage:
#   cd ml/training/data
#   bash download_taco.sh
#
# Requirements:
#   - Python 3.10+
#   - pip packages: pycocotools (installed automatically)
#   - ~3 GB disk space for images + labels

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TACO_DIR="${SCRIPT_DIR}/TACO"
YOLO_DIR="${SCRIPT_DIR}/yolo_dataset"

echo "============================================"
echo "  CleanWalker — TACO Dataset Setup"
echo "============================================"
echo ""

# -----------------------------------------------------------
# Step 1: Clone TACO repository
# -----------------------------------------------------------
if [ -d "${TACO_DIR}" ]; then
    echo "[1/4] TACO repository already cloned at ${TACO_DIR}"
    echo "      Pulling latest changes..."
    cd "${TACO_DIR}" && git pull --quiet && cd "${SCRIPT_DIR}"
else
    echo "[1/4] Cloning TACO repository..."
    git clone https://github.com/pedropro/TACO.git "${TACO_DIR}"
fi
echo ""

# -----------------------------------------------------------
# Step 2: Install Python dependencies for download
# -----------------------------------------------------------
echo "[2/4] Installing Python dependencies..."
pip install --quiet pycocotools Pillow requests
echo ""

# -----------------------------------------------------------
# Step 3: Download dataset images
# -----------------------------------------------------------
echo "[3/4] Downloading TACO images..."
echo "      This may take a while depending on your connection."
echo ""

cd "${TACO_DIR}"

if [ -f "download.py" ]; then
    python download.py
elif [ -f "download_taco.py" ]; then
    python download_taco.py
else
    echo "      TACO download script not found, attempting direct annotation use..."
    echo "      Images will be downloaded from URLs in annotations.json"

    # Fallback: use the annotations.json and download images directly
    python -c "
import json, os, sys
from pathlib import Path
try:
    from urllib.request import urlretrieve
except ImportError:
    from urllib import urlretrieve

ann_path = Path('data/annotations.json')
if not ann_path.exists():
    print('Error: annotations.json not found')
    sys.exit(1)

with open(ann_path) as f:
    data = json.load(f)

total = len(data.get('images', []))
print(f'Downloading {total} images...')

for i, img in enumerate(data['images'], 1):
    url = img.get('flickr_url') or img.get('coco_url', '')
    file_name = img['file_name']
    dest = Path('data') / file_name
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        continue
    if url:
        try:
            urlretrieve(url, str(dest))
            if i % 100 == 0:
                print(f'  [{i}/{total}] downloaded')
        except Exception as e:
            print(f'  Warning: failed to download {file_name}: {e}')

print(f'Download complete: {total} images')
"
fi

cd "${SCRIPT_DIR}"
echo ""

# -----------------------------------------------------------
# Step 4: Convert to YOLO format
# -----------------------------------------------------------
echo "[4/4] Converting TACO annotations to YOLO format..."
python "${SCRIPT_DIR}/convert_taco_to_yolo.py" \
    --taco-dir "${TACO_DIR}" \
    --output-dir "${YOLO_DIR}" \
    --val-split 0.2

echo ""
echo "============================================"
echo "  Dataset setup complete!"
echo "============================================"
echo ""
echo "Dataset location: ${YOLO_DIR}"
echo "Config file:      ${YOLO_DIR}/dataset.yaml"
echo ""
echo "To start training:"
echo "  cd $(dirname "${SCRIPT_DIR}") && cd .."
echo "  python -m training.src.train --data training/data/yolo_dataset/dataset.yaml"
echo ""
