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
| Jetson Orin | Orin (8 GB shared) | ARM A78AE | 16 GB | 🟧 Experimental |

## Dependency Tiers

Bare `pip install dimos` installs the **core** tier. Extras add capabilities on top.

```bash
pip install dimos                           # Core only
pip install dimos[base,unitree]             # Full stack + Unitree
pip install dimos[base,unitree,sim]         # + MuJoCo simulation
pip install dimos[base,unitree,drone]       # + Drone support
pip install dimos[base,unitree,manipulation] # + Arm control
```

| Extra | What it adds | Key packages | GPU? |
|-------|-------------|--------------|------|
| *(core)* | Transport, streams, CLI, blueprints, occupancy maps | dimos-lcm, numpy, scipy, opencv, open3d, numba, Pinocchio, typer, textual | No |
| `agents` | LLM agent, speech, tool use | langchain, openai, whisper, anthropic | No |
| `perception` | Object detection, VLMs, tracking | ultralytics, transformers, moondream | **Yes** |
| `visualization` | Rerun viewer + bridge | rerun-sdk, dimos-viewer | No |
| `web` | FastAPI web interface, audio | fastapi, uvicorn, ffmpeg-python | No |
| `sim` | MuJoCo simulation | mujoco, playground, pygame | No |
| `unitree` | Unitree Go2 / G1 support | unitree-webrtc-connect | No |
| `drone` | DJI Tello / MAVLink drones | pymavlink | No |
| `manipulation` | Arm planning + control | Drake, piper-sdk, xarm-sdk | No |
| `cuda` | GPU acceleration | cupy, onnxruntime-gpu, xformers | **Yes** |
| `cpu` | CPU inference backends | onnxruntime, ctransformers | No |
| `misc` | Extra models, embeddings, hardware SDKs | cerebras, edgetam, sentence-transformers, tiktoken | Varies |
| `docker` | Minimal set for Docker sidecar modules | dimos-lcm, numpy, opencv-headless, rerun-sdk | No |
| `base` | Kitchen sink (agents + web + perception + viz + sim) | All of the above | **Yes** |
| `dev` | Linting, testing, type stubs | ruff, mypy, pytest, pre-commit | No |

### Core Dependencies (bare `pip install dimos`)

The core tier is **not** lightweight — it includes `open3d`, `opencv`, `numba`, and `Pinocchio (pin)`. This means:

- ✅ x86_64 Linux and macOS — works
- ✅ Headless servers — works (opencv-python pulls in headless-compatible builds)
- 🟧 Jetson / aarch64 — works with `open3d-unofficial-arm` (auto-selected)
- ❌ Raspberry Pi — **not supported** for core install (open3d, numba unavailable on armv7)
- ✅ Raspberry Pi — can run `dimos[docker]` extra as a sidecar module (minimal transport-only deps)

### Docker Extra (minimal transport)

For constrained environments (Raspberry Pi, Docker sidecars, edge nodes), use the `docker` extra:

```bash
pip install dimos[docker]
```

This installs only LCM transport, numpy, opencv-headless, rerun-sdk, and pydantic — enough to write modules that communicate with a DimOS host over the network.

## Headless / Server Environments

If running on a headless Ubuntu server (no display), install OpenGL libraries for visualization dependencies:

```bash
sudo apt-get install -y libgl1 libegl1
```

Nix users (`nix develop`) don't need this — the flake provides `libGL`, `libGLU`, and `mesa`.
