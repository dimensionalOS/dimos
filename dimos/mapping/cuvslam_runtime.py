# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Picking the cuVSLAM SDK build and NVIDIA driver environment for the host.

Shared by every module that runs cuVSLAM (cuvslam, dim_slam)."""

from __future__ import annotations

import ctypes
import os
from pathlib import Path
import platform
import sys
from typing import Literal
from uuid import uuid4

from dimos.constants import CACHE_DIR
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The nix loader ignores ld.so.cache, so dlopen("libcuda.so.1") fails. Jetson needs the
# whole directory: its libcuda.so.1 depends on its siblings.
_DRIVER_ONLY_LIB_DIRS = (
    Path("/run/opengl-driver/lib"),
    # Jetson: which of these two names exists varies by release.
    Path("/usr/lib/aarch64-linux-gnu/nvidia"),
    Path("/usr/lib/aarch64-linux-gnu/tegra"),
)
# Adding one of these whole would shadow the binary's libstdc++, so symlink the driver.
_HOST_LIB_DIRS = (
    Path("/usr/lib/x86_64-linux-gnu"),
    Path("/usr/lib/aarch64-linux-gnu"),
)
# nixpkgs tracks a newer CUDA than JetPack ships and the mismatch fails at
# cusolverDnCreate, so the host's own runtime goes first where it exists.
_HOST_CUDA_LIB_DIR = Path("/usr/local/cuda/lib64")
_DRIVER_LIBS = (
    "libcuda.so.1",
    "libnvidia-ptxjitcompiler.so.1",
    "libnvidia-nvvm.so.4",
    "libnvidia-ml.so.1",
)
_DRIVER_LINK_DIR = CACHE_DIR / "nvidia-driver-libs"


def driver_library_dir() -> Path | None:
    dedicated = next(
        (d for d in _DRIVER_ONLY_LIB_DIRS if (d / "libcuda.so.1").exists()),
        None,
    )
    if dedicated is not None:
        return dedicated
    host = next((d for d in _HOST_LIB_DIRS if (d / "libcuda.so.1").exists()), None)
    if host is None:
        return None
    _DRIVER_LINK_DIR.mkdir(parents=True, exist_ok=True)
    for name in _DRIVER_LIBS:
        source = host / name
        link = _DRIVER_LINK_DIR / name
        if not source.exists():
            continue
        target = source.resolve()
        if link.is_symlink() and link.readlink() == target:
            continue
        # Two callers can reach this at once, and symlink_to over an existing path raises.
        staging = link.with_name(f"{name}.{uuid4().hex}.tmp")
        staging.symlink_to(target)
        os.replace(staging, link)
    return _DRIVER_LINK_DIR


Hardware = Literal[
    "thor",
    "orin",
    "xavier",
    "nano",
    "linux-x86-nvidia",
    "linux-x86-no-nvidia",
    "linux-arm-no-nvidia",
    "darwin-apple-silicon",
    "darwin-intel",
]


def detect_hardware() -> Hardware:
    if sys.platform == "darwin":
        if platform.machine() == "arm64":
            return "darwin-apple-silicon"
        return "darwin-intel"
    if platform.machine() == "aarch64":
        compatible = Path("/proc/device-tree/compatible")
        chip = compatible.read_bytes() if compatible.exists() else b""
        if b"tegra264" in chip:
            return "thor"
        if b"tegra234" in chip:
            return "orin"
        if b"tegra194" in chip:
            return "xavier"
        if b"tegra210" in chip:
            return "nano"
        return "linux-arm-no-nvidia"
    if detect_cuda_major() > 0:
        return "linux-x86-nvidia"
    return "linux-x86-no-nvidia"


def detect_cuda_major() -> int:
    """0 when there is no NVIDIA driver (always on darwin)."""
    if sys.platform == "darwin":
        return 0
    candidates = ["libcuda.so.1"] + [
        str(directory / "libcuda.so.1")
        for directory in (*_DRIVER_ONLY_LIB_DIRS, *_HOST_LIB_DIRS)
        if (directory / "libcuda.so.1").exists()
    ]
    for candidate in candidates:
        try:
            driver = ctypes.CDLL(candidate)
        except OSError:
            continue
        version = ctypes.c_int()
        if driver.cuDriverGetVersion(ctypes.byref(version)) == 0:
            return version.value // 1000
    return 0


def sdk_variant() -> str:
    """Nix cannot see the installed driver (cuda12 vs cuda13) or /proc/device-tree
    (orin vs thor), so the flake's default package cannot make this choice.
    """
    hardware = detect_hardware()
    if hardware in ("thor", "orin"):
        return hardware
    if hardware in ("xavier", "nano"):
        # cuVSLAM ships no JetPack 4/5 build, so their GPUs are unusable here
        logger.warning("cuVSLAM has no %s GPU build; only use_gpu=False will work.", hardware)
        return "aarch64"
    if hardware == "darwin-apple-silicon":
        return "metal"
    if hardware == "darwin-intel":
        raise RuntimeError("cuVSLAM has no Intel-mac build; it needs Apple silicon.")
    if hardware == "linux-arm-no-nvidia":
        # non-Jetson ARM: the CPU-fallback build
        return "aarch64"
    if hardware == "linux-x86-no-nvidia":
        # same derivation as x86_64-cuda12 (ENFORCE_GPU=OFF, so it runs without a
        # driver); the alias exists so the choice reads as deliberate
        logger.warning("No NVIDIA driver found; only use_gpu=False will work.")
        return "x86_64"
    major = detect_cuda_major()
    if major < 12:
        logger.warning(
            "This NVIDIA driver supports CUDA %d and the GPU path needs 12+; "
            "only use_gpu=False will work until the driver is upgraded.",
            major,
        )
    return f"x86_64-cuda{major}"


def driver_env() -> dict[str, str]:
    if sys.platform == "darwin":
        return {"CUMETAL_USE_METAL_DEVICE_ADDRESSES": "1"}
    parts = [str(_HOST_CUDA_LIB_DIR)] if _HOST_CUDA_LIB_DIR.is_dir() else []
    driver_dir = driver_library_dir()
    if driver_dir is not None:
        parts.append(str(driver_dir))
    if not parts:
        return {}
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    if existing:
        parts.append(existing)
    return {"LD_LIBRARY_PATH": ":".join(parts)}
