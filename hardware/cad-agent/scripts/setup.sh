#!/bin/bash
set -e

echo "=== CAD Agent Project Setup ==="

# Detect environment
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "macOS detected"
    PLATFORM="macos"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Linux detected"
    PLATFORM="linux"
else
    echo "Unknown OS: $OSTYPE"
    PLATFORM="unknown"
fi

# Check Python
PYTHON=$(command -v python3 || command -v python)
if [ -z "$PYTHON" ]; then
    echo "❌ Python not found. Install Python 3.10+"
    exit 1
fi
echo "Using: $($PYTHON --version)"

# Create venv if not active
if [ -z "$VIRTUAL_ENV" ]; then
    if [ ! -d ".venv" ]; then
        echo "Creating virtual environment..."
        $PYTHON -m venv .venv
    fi
    echo "Activating virtual environment..."
    source .venv/bin/activate
fi

# Install CadQuery — conda preferred, pip fallback
if command -v conda &> /dev/null; then
    echo "Conda detected. Installing CadQuery via conda..."
    conda install -c cadquery -c conda-forge cadquery=2.4 -y
    pip install pyvista vtk trimesh pillow
else
    echo "Installing via pip..."
    pip install --upgrade pip
    pip install cadquery pyvista vtk trimesh pillow numpy
fi

# Headless rendering deps (Linux)
if [[ "$PLATFORM" == "linux" ]]; then
    echo "Checking headless rendering dependencies..."
    if ! dpkg -s xvfb &> /dev/null 2>&1; then
        echo "Installing xvfb for headless rendering..."
        sudo apt-get update && sudo apt-get install -y xvfb libgl1-mesa-glx libgl1-mesa-dev || echo "⚠️  Could not install xvfb (may need sudo). Headless rendering may fail."
    fi
fi

# Verify installation
echo ""
echo "=== Verifying installation ==="
python -c "import cadquery; print(f'CadQuery {cadquery.__version__} ✅')" 2>/dev/null || echo "❌ CadQuery not found"
python -c "import pyvista; print(f'PyVista {pyvista.__version__} ✅')" 2>/dev/null || echo "❌ PyVista not found"
python -c "import trimesh; print(f'Trimesh {trimesh.__version__} ✅')" 2>/dev/null || echo "❌ Trimesh not found"
python -c "import numpy; print(f'NumPy {numpy.__version__} ✅')" 2>/dev/null || echo "❌ NumPy not found"

echo ""
echo "=== Setup complete ==="
echo "Write your design description in design_description.md and run the agent!"
