# ML Pipelines

Machine learning pipelines for CleanWalker trash detection — from dataset preparation through training to real-time inference.

## Structure

```
ml/
├── perception/              # Runtime inference pipeline
│   ├── src/
│   │   ├── config.py        # Class definitions, thresholds, model paths
│   │   ├── detect.py        # TrashDetector class (YOLO inference)
│   │   └── demo.py          # Image/video inference demo
│   ├── weights/             # Trained model weights (best.pt)
│   └── requirements.txt
├── training/                # Model training pipeline
│   ├── src/
│   │   └── train.py         # YOLOv8 fine-tuning script
│   ├── data/
│   │   ├── download_taco.sh       # Download & prepare TACO dataset
│   │   └── convert_taco_to_yolo.py  # COCO → YOLO format converter
│   └── requirements.txt
└── demo_webcam.py           # Live webcam/video demo
```

## Trash Classes

The model detects 10 categories of outdoor litter:

| Index | Class | Examples |
|-------|-------|----------|
| 0 | `plastic_bottle` | Water bottles, soda bottles, caps |
| 1 | `can` | Drink cans, food cans, aerosol |
| 2 | `cigarette_butt` | Cigarettes, cigarette butts |
| 3 | `paper` | Loose paper, cups, tissues, receipts |
| 4 | `plastic_bag` | Carrier bags, bin bags, wrappers |
| 5 | `food_wrapper` | Crisp packets, food containers |
| 6 | `glass_bottle` | Bottles, jars, broken glass |
| 7 | `cardboard` | Boxes, cartons, egg cartons |
| 8 | `styrofoam` | Foam cups, foam containers |
| 9 | `other_trash` | Straws, gloves, batteries, rope, etc. |

## Quick Start

### 1. Install Dependencies

```bash
# From the ml/ directory
pip install -r perception/requirements.txt
pip install -r training/requirements.txt
```

**Requirements:** Python 3.10+, ~2 GB for PyTorch + Ultralytics.

### 2. Download & Prepare Dataset

```bash
cd training/data
bash download_taco.sh
```

This clones the [TACO dataset](https://github.com/pedropro/TACO), downloads all images, converts COCO annotations to YOLO format, creates a train/val split (80/20), and generates `dataset.yaml`.

Disk space: ~3 GB for images + labels.

### 3. Train the Model

```bash
# From the ml/ directory

# GPU training (recommended)
python -m training.src.train --data training/data/yolo_dataset/dataset.yaml

# CPU training (slower, good for testing)
python -m training.src.train --data training/data/yolo_dataset/dataset.yaml --device cpu --epochs 10

# Custom hyperparameters
python -m training.src.train \
    --data training/data/yolo_dataset/dataset.yaml \
    --model yolov8n.pt \
    --epochs 100 \
    --batch-size 16 \
    --img-size 640 \
    --lr 0.01 \
    --device 0

# Resume interrupted training
python -m training.src.train \
    --data training/data/yolo_dataset/dataset.yaml \
    --resume
```

Training outputs:
- Best weights automatically saved to `perception/weights/best.pt`
- Full run artifacts in `cleanwalker/trash-detect/`
- Metrics, confusion matrix, and sample predictions

### 4. Run Inference

#### Python API

```python
from perception.src.detect import TrashDetector
import cv2

detector = TrashDetector(
    model_path="perception/weights/best.pt",
    confidence_threshold=0.5,
)
detector.load_model()

frame = cv2.imread("test_image.jpg")
detections = detector.detect(frame)

for det in detections:
    print(f"{det.class_name}: {det.confidence:.2f} at {det.bbox}")
```

#### Demo on Images/Video

```bash
# Single image
python -m perception.src.demo --input photo.jpg

# Video file
python -m perception.src.demo --input video.mp4

# Directory of images, save results
python -m perception.src.demo --input ./test_images/ --output ./results/

# Custom confidence threshold
python -m perception.src.demo --input photo.jpg --conf 0.3
```

#### Live Webcam Demo

```bash
# Default webcam
python demo_webcam.py

# Video file input
python demo_webcam.py --source path/to/video.mp4

# Save processed video
python demo_webcam.py --source video.mp4 --save output.mp4

# Headless mode (no display window)
python demo_webcam.py --source video.mp4 --save output.mp4 --no-display

# Lower confidence for more detections
python demo_webcam.py --conf 0.3
```

Controls during live demo:
- `q` — Quit
- `s` — Save screenshot

## Training CLI Reference

```
python -m training.src.train [OPTIONS]

Options:
  --data PATH        Path to dataset.yaml (required)
  --model NAME       Base model: yolov8n.pt, yolov8s.pt, etc. (default: yolov8n.pt)
  --epochs N         Training epochs (default: 100)
  --batch-size N     Batch size (default: 16)
  --img-size N       Input resolution (default: 640)
  --lr FLOAT         Initial learning rate (default: 0.01)
  --device DEV       CUDA device index or "cpu" (default: 0, auto-fallback to cpu)
  --workers N        Data loader workers (default: 4)
  --patience N       Early stopping patience (default: 20)
  --project NAME     Output project directory (default: cleanwalker)
  --name NAME        Run name (default: trash-detect)
  --resume           Resume from last checkpoint
  --verbose          Debug logging
```

## Dataset Format

The pipeline uses standard YOLO format:

```
yolo_dataset/
├── dataset.yaml          # Points to images/labels, lists class names
├── images/
│   ├── train/            # Training images
│   └── val/              # Validation images
└── labels/
    ├── train/            # YOLO .txt labels (one per image)
    └── val/
```

Each label file has one line per object: `class_id cx cy w h` (normalized 0–1).

You can use any YOLO-format dataset — just update `dataset.yaml` to match your class names.

## GPU vs CPU

| Setup | Recommended Batch Size | Approximate Time (100 epochs, TACO) |
|-------|----------------------|--------------------------------------|
| NVIDIA GPU (8+ GB VRAM) | 16–32 | ~1–2 hours |
| NVIDIA GPU (4 GB VRAM) | 8 | ~3–4 hours |
| CPU only | 4–8 | ~12–24 hours |

The training script automatically falls back to CPU if no CUDA GPU is detected.

## Architecture

- **Model:** YOLOv8n (nano) — smallest and fastest variant, suitable for edge/embedded deployment
- **Framework:** [Ultralytics](https://docs.ultralytics.com/) — handles training, validation, and export
- **Dataset:** [TACO](http://tacodataset.org/) — Trash Annotations in Context, 1500+ images with ~5000 annotations
- **Input size:** 640x640 pixels
- **NMS IoU threshold:** 0.45
- **Confidence threshold:** 0.5 (configurable)
