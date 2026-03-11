# System Requirements

Detailed hardware and software requirements for running DimOS.

## GPU

- **NVIDIA RTX 4070 or better** (for perception, VLMs, and AI features)
- CUDA support required
- Minimum 8GB VRAM recommended
- 12GB+ VRAM recommended for larger vision-language models

DimOS may run on older hardware (e.g., NVIDIA 3000-series GPUs), but performance and stability are not guaranteed. If you encounter issues, please upgrade to the recommended specifications.

> **GPU is optional** for basic robot control, teleoperation, and non-AI workflows. It is required for perception pipelines, VLMs, and AI-driven features.

## CPU

- **Intel Core i7 (recent generation) or better**
- AMD Ryzen 7 or better
- 8+ cores minimum, 12+ cores recommended
- Multi-core strongly recommended for distributed execution and multi-robot deployments

## Memory

- **16GB RAM minimum**
- 32GB+ recommended for larger models and multi-robot deployments

## Storage

- **10GB+ free disk space** (library install)
- SSD strongly recommended for model loading and data logging
- 25GB+ recommended for developer mode (full git history includes LFS assets)
- 50GB+ if recording datasets or running multiple large models

## Operating System

- **Ubuntu 22.04 / 24.04** (primary supported platform)
- **macOS 12.6+** (beta support)
- NixOS / general Linux (via Nix flake)

## Tested Configurations

| Config | GPU | CPU | RAM | Status |
|--------|-----|-----|-----|--------|
| Dev workstation | RTX 4090 (24GB) | i9-13900K | 64GB | ✅ Primary dev |
| Mid-range | RTX 4070 (12GB) | i7-12700 | 32GB | ✅ Tested |
| Laptop | RTX 4060 Mobile (8GB) | i7-13700H | 16GB | ✅ Tested |
| Headless server | No GPU | Xeon | 32GB | ✅ Control only |
| Jetson Orin | Orin (8GB shared) | ARM A78AE | 16GB | 🟧 Experimental |

## Headless / Server Environments

If running on a headless Ubuntu server (no display), install OpenGL libraries for visualization dependencies:

```bash
sudo apt-get install -y libgl1 libegl1
```
