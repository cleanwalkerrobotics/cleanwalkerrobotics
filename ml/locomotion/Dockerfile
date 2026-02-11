# CleanWalker CW-1 — Isaac Sim Training Container
#
# Provides a GPU-accelerated environment for RL locomotion training
# using NVIDIA Isaac Sim + IsaacLab + rsl_rl.
#
# Build:
#   docker build -t cleanwalker-train -f ml/locomotion/Dockerfile .
#
# Run (with GPU):
#   docker run --gpus all --rm -it \
#     -v $(pwd):/workspace \
#     -v $(pwd)/logs:/workspace/logs \
#     cleanwalker-train \
#     python ml/locomotion/train.py --task CleanWalker-CW1-Flat-v0 --headless
#
# Or use docker-compose.yml for easier configuration.

# --- Base: NVIDIA Isaac Sim container ---
# Isaac Sim 4.5+ includes PhysX, Omniverse Kit, and GPU rendering
FROM nvcr.io/nvidia/isaac-sim:4.5.0 AS base

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ACCEPT_EULA=Y

# --- System dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    vim \
    curl \
    htop \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# --- Install IsaacLab ---
WORKDIR /opt
RUN git clone --depth 1 https://github.com/isaac-sim/IsaacLab.git && \
    cd IsaacLab && \
    ./isaaclab.sh --install

# --- Install rsl_rl and Python dependencies ---
RUN /opt/IsaacLab/isaaclab.sh -p -m pip install --no-cache-dir \
    rsl-rl \
    tensorboard \
    onnx \
    onnxruntime

# --- Set up workspace ---
WORKDIR /workspace

# Copy only the ML training code (not the whole repo)
# In practice, mount the repo as a volume instead
COPY ml/locomotion/ /workspace/ml/locomotion/
COPY hardware/urdf/cleanwalker-cw1/ /workspace/hardware/urdf/cleanwalker-cw1/

# Add locomotion package to PYTHONPATH
ENV PYTHONPATH="/workspace/ml:${PYTHONPATH}"

# --- Default command ---
# Override with docker run or docker-compose command
CMD ["python", "ml/locomotion/train.py", "--task", "CleanWalker-CW1-Flat-v0", "--headless"]
