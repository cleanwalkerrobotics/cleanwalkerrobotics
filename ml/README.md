# ML Pipelines

Machine learning pipelines for CleanWalker trash detection.

## Structure

### `perception/`

Runtime inference pipeline for detecting trash in camera frames.

- `src/detect.py` — `TrashDetector` class for running YOLO-based object detection
- `src/config.py` — Model configuration constants (confidence thresholds, class labels)

### `training/`

Model training pipeline for fine-tuning YOLOv8 on custom trash datasets.

- `src/train.py` — Training script with CLI interface
- `data/` — Dataset directory (add training data here)

## Getting Started

```bash
# Install perception dependencies
pip install -r perception/requirements.txt

# Install training dependencies
pip install -r training/requirements.txt

# Run training
python -m training.src.train --data path/to/dataset.yaml --epochs 100
```
