# CW-1 Simulation Training — Quickstart Guide

Train the CleanWalker CW-1 walking gait on a cloud GPU. No Isaac Sim experience needed — just copy-paste the commands.

**What you get:** A trained ONNX neural network that makes the CW-1 walk on flat terrain, rough terrain, and pick up litter. Runs at 50 Hz on Jetson Orin Nano Super.

**Total training time:** ~16 hours (3h flat + 8h rough + 5h litter)

---

## Pick a Cloud GPU Provider

| Provider | GPU | $/hr | 24h Cost | Link |
|----------|-----|------|----------|------|
| **Vast.ai** | RTX 4090 | $0.25 | **~$6** | vast.ai |
| RunPod | RTX 4090 | $0.44 | ~$11 | runpod.io |
| Lambda Labs | A100 80GB | $1.29 | ~$31 | lambdalabs.com |

All three options have enough VRAM (24 GB+) and time (24h covers ~16h training + margin).

---

## Option 1: Vast.ai (~$6 for 24h)

### Step 1 — Rent a GPU

1. Go to [vast.ai](https://vast.ai) and create an account
2. Add $10 credit (minimum)
3. Click **Search** in the sidebar
4. Set filters:
   - GPU: **RTX 4090**
   - Disk: **50 GB+**
   - Docker: **enabled** (should be by default)
5. Sort by **price (lowest)** and rent the cheapest RTX 4090 instance
6. Select the **SSH** connection type
7. Once the instance is running, copy the SSH command from the dashboard — it looks like:

```bash
ssh -p 12345 root@<ip-address>
```

### Step 2 — Connect and set up

```bash
# SSH into the instance (paste the command from Vast.ai dashboard)
ssh -p <PORT> root@<IP>

# Install git and Docker Compose (if not pre-installed)
apt-get update && apt-get install -y git docker-compose-plugin

# Verify GPU is visible
nvidia-smi
```

**Expected output:** You should see `NVIDIA RTX 4090` with ~24 GB VRAM.

### Step 3 — Clone the repo

```bash
git clone https://github.com/AIMBSStudio/cleanwalkerrobotics.git
cd cleanwalkerrobotics
```

### Step 4 — Build the training container

```bash
docker compose -f ml/locomotion/docker-compose.yml build train-flat
```

**Expected output:** After 10-20 minutes, you see `Successfully tagged cleanwalker-train:latest`.

This downloads NVIDIA Isaac Sim (~15 GB), installs IsaacLab, and sets up the Python environment. It only needs to run once — all phases reuse the same image.

### Step 5 — Phase 1: Flat terrain (~3 hours)

```bash
docker compose -f ml/locomotion/docker-compose.yml up train-flat
```

**Expected output after ~1 min:**
```
[train] Starting training: CleanWalker-CW1-Flat-v0
[train] Num envs: 4096
[train] Max iterations: 1500
[train] Device: cuda
[train] Policy: [512, 256, 128]
```

Then you'll see iteration logs every ~7 seconds:
```
Learning iteration 1/1500 ...
  mean reward: -12.34
  mean episode length: 200.0
```

**How to know it's working:**
- Reward should go from negative (~-12) to positive (~+5 to +15) over 1500 iterations
- Each iteration takes ~7 seconds on RTX 4090
- Total: ~3 hours

When it finishes:
```
[train] Training complete. Logs saved to: /workspace/logs/rsl_rl/cleanwalker_cw1_flat/<timestamp>
[train] Final checkpoint: .../model_1500.pt
```

### Step 6 — Phase 2: Rough terrain (~8 hours)

```bash
docker compose -f ml/locomotion/docker-compose.yml run train-rough
```

**Expected output:** Same as Phase 1, but 3000 iterations. The robot learns to walk on procedurally generated uneven terrain. Reward starts lower (the terrain is harder) and climbs back up.

When it finishes:
```
[train] Training complete. Logs saved to: /workspace/logs/rsl_rl/cleanwalker_cw1_rough/<timestamp>
[train] Final checkpoint: .../model_3000.pt
```

### Step 7 — Phase 3: Litter collection (~5 hours)

```bash
docker compose -f ml/locomotion/docker-compose.yml run train-litter
```

**Expected output:** 2000 iterations. The robot learns to walk, stop, extend its arm, and close its gripper. Observations expand from 48 to 58 dims (arm joints added), actions from 12 to 17.

### Step 8 — Export to ONNX

```bash
docker compose -f ml/locomotion/docker-compose.yml run export-policy
```

**Expected output:**
```
[export] Actor network architecture:
  Sequential(Linear(48, 512), ELU(), Linear(512, 256), ELU(), Linear(256, 128), ELU(), Linear(128, 12))
  Total parameters: 200,000+
[export] ONNX model saved: ml/locomotion/assets/cleanwalker_cw1_policy.onnx
[export] File size: ~800 KB
[export] ONNX validation: max diff = 0.000000 (should be ~0)
```

### Step 9 — Monitor training (optional, in a second terminal)

Open a second SSH session to the same instance:

```bash
ssh -p <PORT> root@<IP>
cd cleanwalkerrobotics
docker compose -f ml/locomotion/docker-compose.yml up tensorboard
```

Then set up a port forward from your local machine:

```bash
# On your LOCAL machine (not the cloud instance)
ssh -p <PORT> -L 6006:localhost:6006 root@<IP>
```

Open [http://localhost:6006](http://localhost:6006) in your browser.

**What to look for in TensorBoard:**
- `Train/mean_reward` — should trend upward
- `Train/mean_episode_length` — should increase (robot stays alive longer)
- `Loss/value_loss` — should decrease over time

### Step 10 — Download the trained model

From your **local machine:**

```bash
# Download the ONNX model
scp -P <PORT> root@<IP>:~/cleanwalkerrobotics/ml/locomotion/assets/cleanwalker_cw1_policy.onnx .

# Download all training logs (optional, for TensorBoard analysis)
scp -P <PORT> -r root@<IP>:~/cleanwalkerrobotics/logs/ ./logs/
```

Or use the download script (see below):

```bash
./ml/locomotion/scripts/download-model.sh <IP> <PORT>
```

### Step 11 — Destroy the instance

Go back to [vast.ai](https://vast.ai), click your instance, and click **Destroy**. You stop paying immediately.

---

## Option 2: RunPod (~$11 for 24h)

### Step 1 — Rent a GPU

1. Go to [runpod.io](https://runpod.io) and create an account
2. Add $15 credit
3. Click **GPU Cloud** > **Deploy**
4. Select **RTX 4090** (24 GB, ~$0.44/hr)
5. Set disk to **50 GB**
6. Choose template: **RunPod Pytorch** (has CUDA pre-installed)
7. Click **Deploy**
8. Once running, click **Connect** > **SSH** and copy the SSH command

### Step 2 — Connect and set up

```bash
# SSH into the instance
ssh <user>@<ip> -p <port> -i ~/.ssh/id_ed25519

# Verify GPU
nvidia-smi
```

**Expected output:** `NVIDIA RTX 4090` with ~24 GB VRAM.

### Step 3 — Clone and build

```bash
git clone https://github.com/AIMBSStudio/cleanwalkerrobotics.git
cd cleanwalkerrobotics

# RunPod instances sometimes need Docker daemon started
dockerd &>/dev/null &
sleep 5

docker compose -f ml/locomotion/docker-compose.yml build train-flat
```

### Steps 4-8 — Train (same commands)

```bash
# Phase 1: Flat terrain (~3h)
docker compose -f ml/locomotion/docker-compose.yml up train-flat

# Phase 2: Rough terrain (~8h)
docker compose -f ml/locomotion/docker-compose.yml run train-rough

# Phase 3: Litter collection (~5h)
docker compose -f ml/locomotion/docker-compose.yml run train-litter

# Export ONNX
docker compose -f ml/locomotion/docker-compose.yml run export-policy
```

### Step 9 — Download

```bash
# From your LOCAL machine
scp -P <PORT> <USER>@<IP>:~/cleanwalkerrobotics/ml/locomotion/assets/cleanwalker_cw1_policy.onnx .
```

Or use the download script:

```bash
./ml/locomotion/scripts/download-model.sh <IP> <PORT> <USER>
```

### Step 10 — Terminate

Go to RunPod dashboard > click the instance > **Terminate**.

---

## Option 3: Lambda Labs (~$31 for 24h)

### Step 1 — Rent a GPU

1. Go to [lambdalabs.com/cloud](https://lambdalabs.com/cloud) and create an account
2. Add SSH key in account settings
3. Click **Launch Instance**
4. Select **1x A100 80GB** ($1.29/hr)
5. Choose a region and launch
6. Copy the SSH command from the dashboard

> Lambda A100s are often sold out. If unavailable, use the API waitlist or try Vast.ai/RunPod.

### Step 2 — Connect and set up

```bash
# SSH into the instance
ssh ubuntu@<IP>

# Lambda instances come with CUDA + Docker pre-installed
nvidia-smi
```

**Expected output:** `NVIDIA A100-SXM4-80GB`. The A100 trains ~30% faster than RTX 4090.

### Step 3 — Clone and build

```bash
git clone https://github.com/AIMBSStudio/cleanwalkerrobotics.git
cd cleanwalkerrobotics
docker compose -f ml/locomotion/docker-compose.yml build train-flat
```

### Steps 4-8 — Train (same commands)

```bash
# Phase 1: Flat terrain (~2h on A100)
docker compose -f ml/locomotion/docker-compose.yml up train-flat

# Phase 2: Rough terrain (~6h on A100)
docker compose -f ml/locomotion/docker-compose.yml run train-rough

# Phase 3: Litter collection (~4h on A100)
docker compose -f ml/locomotion/docker-compose.yml run train-litter

# Export ONNX
docker compose -f ml/locomotion/docker-compose.yml run export-policy
```

> A100 times are ~30% faster than RTX 4090. Total: ~12h instead of ~16h.

### Step 9 — Download

```bash
# From your LOCAL machine
scp ubuntu@<IP>:~/cleanwalkerrobotics/ml/locomotion/assets/cleanwalker_cw1_policy.onnx .
```

Or use the download script:

```bash
./ml/locomotion/scripts/download-model.sh <IP> 22 ubuntu
```

### Step 10 — Terminate

Go to Lambda dashboard > **Terminate** the instance.

---

## After Training: Deploy to Jetson

Copy the ONNX model to the Jetson Orin Nano Super and convert to TensorRT:

```bash
# Copy model to Jetson
scp cleanwalker_cw1_policy.onnx jetson:/home/cleanwalker/models/

# SSH into Jetson
ssh jetson

# Convert to TensorRT engine (FP16 for speed)
/usr/src/tensorrt/bin/trtexec \
    --onnx=/home/cleanwalker/models/cleanwalker_cw1_policy.onnx \
    --saveEngine=/home/cleanwalker/models/cleanwalker_cw1_policy.engine \
    --fp16 --workspace=256
```

**Expected:** `<1ms` inference latency at 50 Hz policy frequency.

---

## Troubleshooting

### "CUDA out of memory"
Reduce parallel environments:
```bash
docker run --gpus all --rm -it \
  -v $(pwd):/workspace \
  cleanwalker-train:latest \
  python ml/locomotion/train.py --task CleanWalker-CW1-Flat-v0 --headless --num_envs 2048
```

### Docker build fails
Make sure NVIDIA Container Toolkit is installed:
```bash
nvidia-smi                    # Should show your GPU
docker run --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi  # Should work too
```

If the second command fails, install the toolkit:
```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Training reward stays negative
- Check GPU utilization: `nvidia-smi` should show ~80-100% GPU usage
- Check logs for errors: `docker compose -f ml/locomotion/docker-compose.yml logs train-flat`
- Negative reward in early iterations is normal — it should turn positive by iteration ~300-500

### TensorBoard not loading
Make sure port forwarding is active:
```bash
# On your LOCAL machine
ssh -p <PORT> -L 6006:localhost:6006 root@<IP>
```
Then open [http://localhost:6006](http://localhost:6006).

### "No space left on device"
The Isaac Sim container is large (~15 GB). Make sure you rented an instance with 50 GB+ disk.

---

## Cost Summary

| Scenario | Provider | GPU | Training Time | Total Cost |
|----------|----------|-----|---------------|------------|
| Budget | Vast.ai | RTX 4090 | ~16h | **~$4-6** |
| Reliable | RunPod | RTX 4090 | ~16h | **~$8-11** |
| Fast | Lambda | A100 80GB | ~12h | **~$16-31** |

Recommendation: Start with **Vast.ai** for the cheapest run. Use **RunPod** if you want more reliable uptime. Use **Lambda A100** for final production training runs.
