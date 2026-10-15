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

"""The NVIDIA driver environment a nix-built CUDA binary needs on this host."""

from __future__ import annotations

import os
from pathlib import Path
import sys
from uuid import uuid4

from dimos.constants import CACHE_DIR

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


def driver_env() -> dict[str, str]:
    if sys.platform == "darwin":
        return {"CUMETAL_USE_METAL_DEVICE_ADDRESSES": "1"}
    driver_dir = driver_library_dir()
    if driver_dir is None:
        return {}
    parts = [str(driver_dir)]
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    if existing:
        parts.append(existing)
    return {"LD_LIBRARY_PATH": ":".join(parts)}
