# Nav Container Image Size Estimate (arm64/aarch64)

**Date**: 2026-03-12
**Target**: M20 NOS (NVIDIA Orin, aarch64)
**Dockerfile**: `docker/navigation/Dockerfile`
**Bead**: di-daa85u.6.4 (OQ6)

## Summary

**Estimated image size: ~7-12 GB (arm64)**
**Available after pruning: ~50 GB**
**Verdict: FITS with 38-43 GB headroom**

## NOS Storage Layout (verified via SSH)

| Partition | Mount | Total | Used | Free |
|-----------|-------|-------|------|------|
| mmcblk0p11 | /var/opt/robot/data (Docker root) | 62 GB | 55 GB | 4.1 GB |
| mmcblk0p13 | /userdata (overlayroot /) | 18 GB | 9.1 GB | 9.0 GB |

Docker is stored on the 62 GB data partition. Currently 23 images consume
48.13 GB, but **46 GB is reclaimable** (95% of images are unused/untagged).
After `docker image prune`, ~50 GB is available.

## Existing arm64 Images on NOS

| Image | Size |
|-------|------|
| ghcr.io/aphexcx/m20-nos:latest | 7.85 GB |
| ghcr.io/aphexcx/m20-deps:latest | 5.72 GB |

Both are arm64 images built from Ubuntu Jammy with a custom ROS base.

## Layer-by-Layer Estimate

| Layer | Estimated Size | Notes |
|-------|---------------|-------|
| Base (ROS Humble desktop-full equiv.) | 4-5 GB | See blocker below |
| apt packages (pcl-ros, foxglove, rqt, etc.) | 300-600 MB | Many already in desktop-full |
| C++ libs (Livox-SDK2, Sophus, Ceres, GTSAM) | ~660 MB | Copied from builder stage |
| ROS workspace install (autonomy stack) | 300-500 MB | colcon build output |
| Python venv + dimos[unitree] | 1.5-2.5 GB | open3d ~500 MB, numpy/scipy, rerun-sdk |
| Config/scripts/entrypoint | ~10 MB | Negligible |
| **Total** | **~7-12 GB** | |

**Reference**: README.md states "approximately 24 GB" — this is for amd64, likely
from before multi-stage build optimization or including Docker build cache.
The comparable m20-nos arm64 image is 7.85 GB.

## Blocker: No arm64 Base Image

`osrf/ros:humble-desktop-full` is **amd64-only** on Docker Hub. No arm64 variant
exists. The Dockerfile cannot build for arm64 as-is.

Available arm64 ROS images (from arm64v8/ros):
- `humble-perception`: 916 MB compressed
- `humble-ros-base`: 256 MB compressed
- `humble-ros-core`: 137 MB compressed

**No `humble-desktop` or `humble-desktop-full` for arm64.**

### Alternatives

1. **Build from `ros:humble-perception`** + install missing desktop packages via apt
2. **Reuse m20-deps base** (5.72 GB, already arm64, already on NOS)
3. **Use NVIDIA L4T ROS image** for native Jetson GPU support
4. **Build custom base** from Ubuntu Jammy arm64 + ROS Humble from source

Option 2 maximizes layer sharing with existing m20-nos image and avoids
downloading a separate base.

## Coexistence Scenarios

| Scenario | Combined Size | Fits in 50 GB? |
|----------|--------------|----------------|
| Nav container alone | 7-12 GB | Yes (38-43 GB free) |
| Nav + m20-nos (no shared layers) | 15-20 GB | Yes (30-35 GB free) |
| Nav + m20-nos (shared base) | 12-16 GB | Yes (34-38 GB free) |

## Risks

1. **arm64 base image**: Must be sourced or built — not available from OSRF
2. **open3d arm64**: May lack prebuilt wheels; building from source adds significant
   build time and may require different dependencies
3. **Partition size**: 62 GB is tight for multiple large images; old images MUST be
   pruned regularly
4. **Build cache**: Cross-compilation (buildx) or on-device builds may need
   temporary space exceeding free disk
