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

"""Native C++ NVIDIA cuVSLAM stereo visual odometry module."""

from __future__ import annotations

import os
from pathlib import Path

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

MODULE_DIR = Path(__file__).resolve().parent
_REPO_ROOT = MODULE_DIR.parents[2]

# The recordings carry right P[3] == 0, so the baseline cannot be read off
# CameraInfo; this is the D455 factory extrinsic.
D455_FACTORY_BASELINE_M = 0.09486231207847595

# The nix binary runs under the nix loader, whose ld.so.cache does not list the
# host driver, so dlopen("libcuda.so.1") fails and cudart reports the misleading
# "driver version is insufficient". NixOS and CUDA containers expose the driver
# at the path below; elsewhere we hand the loader a directory holding only these
# libs, because the whole system lib dir would shadow the binary's own libstdc++.
_NIXOS_DRIVER_LIB_DIR = Path("/run/opengl-driver/lib")
_HOST_LIB_DIRS = (Path("/usr/lib/x86_64-linux-gnu"), Path("/usr/lib/aarch64-linux-gnu"))
_DRIVER_LIBS = (
    "libcuda.so.1",
    "libnvidia-ptxjitcompiler.so.1",
    "libnvidia-nvvm.so.4",
    "libnvidia-ml.so.1",
)
_DRIVER_LINK_DIR = Path.home() / ".cache/dimos/nvidia-driver-libs"


def driver_library_dir() -> Path | None:
    """Directory the loader needs on LD_LIBRARY_PATH to find the NVIDIA driver."""
    if (_NIXOS_DRIVER_LIB_DIR / "libcuda.so.1").exists():
        return _NIXOS_DRIVER_LIB_DIR
    host = next((d for d in _HOST_LIB_DIRS if (d / "libcuda.so.1").exists()), None)
    if host is None:
        return None
    _DRIVER_LINK_DIR.mkdir(parents=True, exist_ok=True)
    for name in _DRIVER_LIBS:
        source = host / name
        link = _DRIVER_LINK_DIR / name
        if source.exists() and not link.is_symlink():
            link.symlink_to(source.resolve())
    return _DRIVER_LINK_DIR


def _driver_env() -> dict[str, str]:
    driver_dir = driver_library_dir()
    if driver_dir is None:
        return {}
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    return {"LD_LIBRARY_PATH": f"{driver_dir}:{existing}" if existing else str(driver_dir)}


class CuvslamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/cuvslam_odometry"
    build_command: str | None = f"nix build '{_REPO_ROOT}?dir=dimos/mapping/cuvslam_native'"
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    baseline_m: float = D455_FACTORY_BASELINE_M
    rectified: bool = True
    publish_landmarks: bool = True
    # cuVSLAM's own default. Measured better than synchronous SBA on the airbnb
    # replay (windowed ATE 0.73 m vs 0.94 m), so there is no reason to override it.
    async_sba: bool = True


class CuvslamOdometry(NativeModule):
    """Stereo visual odometry on the GPU.

    cuVSLAM restarts its world frame after a tracking loss, so the segment id
    rides along in ``child_frame_id`` as ``cuvslam_rig/segment_<n>``. Poses from
    different segments must never be differenced.
    """

    config: CuvslamConfig

    image_left: In[Image]
    image_right: In[Image]
    camera_info: In[CameraInfo]

    odometry: Out[Odometry]
    landmarks: Out[PointCloud2]
