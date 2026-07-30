# System Requirements

## Hardware

:::{list-table}
   :header-rows: 1
   :widths: 20 40 40

   * - Component
     - Minimum
     - Recommended
   * - GPU
     - NVIDIA RTX 3000+ with 8 GB VRAM
     - RTX 4070+ with 12 GB+ VRAM
   * - CPU
     - 8-core Intel or AMD
     - 12+ cores
   * - RAM
     - 16 GB
     - 32 GB+
   * - Disk
     - 10 GB SSD
     - 25 GB+ SSD
   * - Operating system
     - Ubuntu 22.04 or macOS 12.6+
     - Ubuntu 24.04
:::

:::{note}
Basic robot control does not require a GPU. Perception, VLM, and other AI
features do.
:::

## Tested configurations

:::{list-table}
   :header-rows: 1
   :widths: 24 24 24 12 16

   * - Configuration
     - GPU
     - CPU
     - RAM
     - Status
   * - Development workstation
     - RTX 4090 with 24 GB
     - i9-13900K
     - 64 GB
     - Primary
   * - Mid-range workstation
     - RTX 4070 with 12 GB
     - i7-12700
     - 32 GB
     - Tested
   * - Laptop
     - RTX 4060 Mobile with 8 GB
     - i7-13700H
     - 16 GB
     - Tested
   * - Headless server
     - No GPU
     - Xeon
     - 32 GB
     - Control only
   * - Jetson AGX Orin
     - Orin with 32 GB shared
     - ARM A78AE
     - 32 GB
     - Tested
   * - Jetson Orin Nano
     - Orin with 8 GB shared
     - ARM A78AE
     - 8 GB
     - Experimental
:::

## Dependency tiers

A bare `pip install dimos` installs the core tier. Extras add capabilities:

```bash
pip install dimos                                    # Core only
pip install 'dimos[base,unitree]'                    # Unitree robot control
pip install 'dimos[base,unitree,perception]'         # Object detection and VLMs
pip install 'dimos[base,unitree,sim]'                # MuJoCo simulation
pip install 'dimos[base,unitree,perception,sim]'     # Full stack
pip install 'dimos[base,unitree,drone]'              # Drone support
pip install 'dimos[base,unitree,manipulation]'       # Arm control
```

:::{list-table}
   :header-rows: 1
   :widths: 16 38 34 12

   * - Extra
     - Adds
     - Key packages
     - GPU
   * - Core
     - Transports, streams, CLI, blueprints, and occupancy maps
     - dimos-lcm, NumPy, SciPy, OpenCV, Open3D, Numba, Pinocchio, Typer, Textual
     - No
   * - ``agents``
     - LLM agents, speech, and tool use
     - LangChain, OpenAI, Ollama, faster-whisper
     - No
   * - ``perception``
     - Object detection, VLMs, and tracking
     - Ultralytics, Transformers, Moondream
     - Yes
   * - ``visualization``
     - Rerun viewer and bridge
     - rerun-sdk, dimos-viewer
     - No
   * - ``web``
     - Web interface and audio
     - FastAPI, Uvicorn, ffmpeg-python
     - No
   * - ``sim``
     - MuJoCo simulation
     - MuJoCo, playground, Pygame
     - No
   * - ``unitree``
     - Unitree Go2 and G1 support
     - unitree-webrtc-connect
     - No
   * - ``unitree-dds``
     - Unitree DDS bridge and ``unitree``
     - unitree-sdk2py, CycloneDDS
     - No
   * - ``drone``
     - DJI Tello and MAVLink drone support
     - pymavlink
     - No
   * - ``manipulation``
     - Arm planning and control
     - Drake, piper-sdk, xarm-sdk
     - No
   * - ``mapping``
     - GTSAM pose-graph optimization
     - gtsam-extended
     - No
   * - ``cuda``
     - GPU acceleration
     - CuPy, onnxruntime-gpu
     - Yes
   * - ``cpu``
     - CPU inference backends
     - onnxruntime
     - No
   * - ``misc``
     - Additional models, embeddings, and hardware SDKs
     - EdgeTAM, timm, torchreid, xarm-sdk
     - Varies
   * - ``base``
     - Standard agents, web, and visualization stack
     - LangChain, FastAPI, rerun-sdk
     - No
   * - ``dds``
     - CycloneDDS transport
     - cyclonedds
     - No
:::

## Headless servers

On a headless Ubuntu server, install the OpenGL libraries required by the
visualization dependencies:

```bash
sudo apt-get install -y libgl1 libegl1
```

The Nix development shell already provides libGL, libGLU, and Mesa.
