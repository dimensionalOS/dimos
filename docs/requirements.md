# System Requirements

## Hardware

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | NVIDIA RTX 3000+ (8 GB VRAM) | RTX 4070+ (12 GB+ VRAM) |
| CPU | 8-core Intel / AMD | 12+ cores |
| RAM | 16 GB | 32 GB+ |
| Disk | 10 GB SSD | 25 GB+ SSD |
| OS | Ubuntu 22.04, macOS 12.6+ | Ubuntu 24.04 |

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

Bare `pip install dimos` installs the **core** tier. Extras add capabilities on top.

```bash
pip install dimos                                    # Core only
pip install 'dimos[base,unitree]'                    # Unitree robot control (no torch)
pip install 'dimos[base,unitree,perception]'         # + Object detection, VLMs (requires torch)
pip install 'dimos[base,unitree,sim]'                # + MuJoCo simulation
pip install 'dimos[base,unitree,perception,sim]'     # Full stack
pip install 'dimos[base,unitree,drone]'              # + Drone support
pip install 'dimos[base,unitree,manipulation]'       # + Arm control
```

| Extra | What it adds | Key packages | GPU? |
|-------|-------------|--------------|------|
| *(core)* | Transport, streams, CLI, blueprints, occupancy maps | dimos-lcm, numpy, scipy, opencv, open3d, numba, Pinocchio, typer, textual | No |
| `agents` | LLM agent, speech, tool use | langchain, openai, faster-whisper, anthropic | No |
| `perception` | Object detection, VLMs, tracking | ultralytics, transformers, moondream | **Yes** |
| `visualization` | Rerun viewer + bridge | rerun-sdk, dimos-viewer | No |
| `web` | FastAPI web interface, audio | fastapi, uvicorn, ffmpeg-python | No |
| `sim` | MuJoCo simulation | mujoco, playground, pygame | No |
| `unitree` | Unitree Go2 / G1 support | unitree-webrtc-connect | No |
| `unitree-dds` | Unitree DDS bridge (superset of `unitree`) | unitree-sdk2py, cyclonedds | No |
| `drone` | DJI Tello / MAVLink drones | pymavlink | No |
| `manipulation` | Arm planning + control | Drake, piper-sdk, xarm-sdk | No |
| `navigation` | GTSAM-backed pose graph optimization | gtsam-extended | No |
| `cuda` | GPU acceleration | cupy, onnxruntime-gpu, xformers | **Yes** |
| `cpu` | CPU inference backends | onnxruntime, ctransformers | No |
| `misc` | Extra models, embeddings, hardware SDKs | cerebras, edgetam, sentence-transformers, tiktoken | Varies |
| `docker` | Minimal set for Docker sidecar modules | dimos-lcm, numpy, opencv-headless, rerun-sdk | No |
| `base` | Standard stack (agents + web + viz) | langchain, fastapi, rerun-sdk | No |
| `psql` | PostgreSQL storage | psycopg2 | No |
| `dds` | DDS transport (CycloneDDS) | cyclonedds | No |

## Headless / Server Environments

If running on a headless Ubuntu server (no display), install OpenGL libraries for visualization dependencies:

```bash
sudo apt-get install -y libgl1 libegl1
```

Nix users (`nix develop`) don't need this — the flake provides `libGL`, `libGLU`, and `mesa`.
