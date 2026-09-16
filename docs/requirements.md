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

Choose extras with the installer's `--extras` option. The default is `all`, adjusted for the platform. For example:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --extras base,unitree,sim
```

Blueprints declare which extras they need: `dimos deps <blueprint>` prints them
together with the matching install command, and `dimos run` prepares a managed
environment when the current one is not enough (see
[Dependencies and Runtime Environments](/docs/usage/dependencies.md)).

| Extra | What it adds | Key packages | GPU? |
|-------|-------------|--------------|------|
| *(core)* | Transport, streams, CLI, blueprints, occupancy maps, viewer | dimos-lcm, zenoh, numpy, scipy, opencv, open3d, numba, Pinocchio, rerun-sdk, typer, textual | No |
| `agents` | LLM agent, speech, tool use | langchain, openai, ollama, faster-whisper | No |
| `perception` | Object detection, VLMs, tracking, spatial memory | ultralytics, transformers (torch), moondream, chromadb | **Yes** |
| `visualization` | Rerun URDF robot visualization | yourdfpy | No |
| `web` | FastAPI web interface, cockpit relay bridge, audio | fastapi, uvicorn, python-socketio, aioquic, ffmpeg-python | No |
| `sim` | MuJoCo simulation | mujoco, playground, pygame | No |
| `unitree` | Unitree Go2 / G1 WebRTC connection | unitree-webrtc-connect | No |
| `unitree-dds` | Unitree DDS bridge (superset of `unitree`) | unitree-sdk2py, cyclonedds, mcap | No |
| `drone` | DJI Tello / MAVLink drones | pymavlink | No |
| `control` | Arm hardware adapters and keyboard control | piper-sdk, xarm-python-sdk, can-motor-control, pygame | No |
| `planning` | Motion planning and its visualization (includes `control`) | drake, roboplan, viser, trimesh | No |
| `manipulation` | Complete manipulation workflow (`planning`, `base`, `sim`, `cpu`) | | No |
| `mapping` | GTSAM-backed pose graph optimization (relocalization) | gtsam-extended | No |
| `cuda` | GPU inference backends | cupy, onnxruntime-gpu | **Yes** |
| `cpu` | CPU inference backend | onnxruntime | No |
| `misc` | Extra models and embeddings | open_clip_torch, torchreid, googlemaps | Varies |
| `base` | Standard stack (`agents`, `web`, `perception`, `visualization`) | | No |
| `dds` | DDS transport (CycloneDDS) | cyclonedds | No |
| `webrtc` | WebRTC data channels (Cloudflare Realtime) | aiortc, aiohttp | No |
| `apriltag` | AprilTag fixture printing and 3D plates | reportlab, trimesh, manifold3d | No |
| `scene` | Scene package cooking (USD import, convex decomposition) | usd-core, coacd, trimesh | No |
| `learning` | Imitation datasets (LeRobot, HDF5) | pyarrow, pandas, h5py | No |
| `spot` | Boston Dynamics Spot | bosdyn-client | No |
| `all` | Everything except `dds`, `unitree-dds`, `spot`, `learning` | | Varies |

`perception` brings PyTorch through `transformers[torch]`; `base` therefore
includes PyTorch. Cockpit voice input and the legacy browser audio upload
require the `ffmpeg` executable in addition to the Python `web` extra. The
installer supplies it through system packages on Ubuntu and macOS.

## Headless / Server Environments

The Ubuntu installer includes `libgl1` and `libegl1` for visualization imports on headless servers.

Nix users (`nix develop`) don't need this. The flake provides `libGL`, `libGLU`, and `mesa`.
