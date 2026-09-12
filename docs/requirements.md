# System Requirements

Install dimOS with the [official installer](/docs/installation/index.md). It provisions Python and system dependencies for the supported installation paths.

## Hardware

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | NVIDIA RTX 3000+ (8 GB VRAM) | RTX 4070+ (12 GB+ VRAM) |
| CPU | 8-core Intel / AMD | 12+ cores |
| RAM | 16 GB | 32 GB+ |
| Disk | 10 GB SSD | 25 GB+ SSD |
| OS | Ubuntu 22.04, macOS 14+ | Ubuntu 24.04 |

> GPU is optional for basic robot control. Required for perception, VLMs, and AI features.

## Tested Configurations

| Config | GPU | CPU | RAM | Status |
|--------|-----|-----|-----|--------|
| Dev workstation | RTX 4090 (24 GB) | i9-13900K | 64 GB | ✅ Primary dev |
| Mid-range | RTX 4070 (12 GB) | i7-12700 | 32 GB | ✅ Tested |
| Laptop | RTX 4060 Mobile (8 GB) | i7-13700H | 16 GB | ✅ Tested |
| Headless server | No GPU | Xeon | 32 GB | ✅ Control only |
| Jetson AGX Orin | Orin (32 GB shared) | ARM A78AE | 32 GB | ✅ Tested |
| Jetson Orin Nano | Orin (8 GB shared) | ARM A78AE | 8 GB | 🟧 Experimental |

## Dependency Tiers

The macOS 14 minimum follows the current developer dependencies, including ONNX Runtime and Drake. It does not mean that every older dimOS package requires macOS 14. See the [macOS installation guide](/docs/installation/osx.md).

The installer requires an explicit **navigation**, **manipulation**, or combined capability selection. It includes the supporting features for common blueprints and detects the accelerator backend. For example:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --capabilities navigation
```

For advanced direct `uv`/`pip` installs, granular package extras remain available:

| Extra | What it adds | Key packages | GPU? |
|-------|-------------|--------------|------|
| *(core)* | Transport, streams, CLI, blueprints, occupancy maps | dimos-lcm, numpy, scipy, opencv, open3d, numba, Pinocchio, typer, textual | No |
| `agents` | LLM agent, speech, tool use | langchain, openai, ollama, faster-whisper | No |
| `perception` | Object detection, VLMs, tracking | ultralytics, transformers, moondream | **Yes** |
| `visualization` | Rerun viewer + bridge | rerun-sdk, dimos-viewer | No |
| `web` | FastAPI web interface, audio | fastapi, uvicorn, ffmpeg-python | No |
| `sim` | MuJoCo simulation | mujoco, playground, pygame | No |
| `unitree` | Unitree Go2 / G1 support | unitree-webrtc-connect | No |
| `unitree-dds` | Unitree DDS bridge (superset of 'unitree') | unitree-sdk2py, cyclonedds | No |
| `drone` | DJI Tello / MAVLink drones | pymavlink | No |
| `manipulation` | Arm planning + control | Drake, piper-sdk, xarm-sdk | No |
| `mapping` | GTSAM-backed pose graph optimization (relocalization) | gtsam-extended | No |
| `cuda` | CUDA PyTorch and GPU acceleration | torch, torchvision, cupy, onnxruntime-gpu | **Yes** |
| `cpu` | CPU PyTorch and inference backends | torch, torchvision, onnxruntime | No |
| `misc` | Extra models, embeddings, hardware SDKs | edgetam, timm, torchreid, xarm-sdk | Varies |
| `base` | Standard stack (agents + web + viz) | langchain, fastapi, rerun-sdk | No |
| `dds` | DDS transport (CycloneDDS) | cyclonedds | No |

Cockpit voice input and the legacy browser audio upload require the `ffmpeg`
executable in addition to the Python `web` extra. The installer supplies it through system packages on Ubuntu and macOS.

## Headless / Server Environments

The Ubuntu installer includes `libgl1` and `libegl1` for visualization imports on headless servers.

Nix users (`nix develop`) don't need this. The flake provides `libGL`, `libGLU`, and `mesa`.
