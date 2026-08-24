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

from __future__ import annotations

import ctypes
import os
from pathlib import Path
import platform
import sys
from typing import Literal
from uuid import uuid4

from pydantic import Field

from dimos.constants import CACHE_DIR
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MODULE_DIR = Path(__file__).resolve().parent

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
        # Re-point after a driver upgrade; a stale link would dangle.
        if link.is_symlink() and link.readlink() == target:
            continue
        # Two callers can reach this at once, and symlink_to over an existing path raises.
        staging = link.with_name(f"{name}.{uuid4().hex}.tmp")
        staging.symlink_to(target)
        os.replace(staging, link)
    return _DRIVER_LINK_DIR


def driver_cuda_major() -> int:
    """The CUDA major the installed driver supports, or 0 if there is no driver."""
    # Absolute paths too: a nix-built python ignores ld.so.cache.
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
    if sys.platform == "darwin":
        return "metal"
    if platform.machine() == "aarch64":
        compatible = Path("/proc/device-tree/compatible")
        chip = compatible.read_bytes() if compatible.exists() else b""
        return "thor" if b"tegra264" in chip else "orin"
    major = driver_cuda_major()
    if 0 < major < 12:
        logger.warning(
            "This NVIDIA driver supports CUDA %d and cuVSLAM ships nothing older "
            "than 12; the build will not run until the driver is upgraded.",
            major,
        )
    return "x86_64-cuda13" if major >= 13 else "x86_64-cuda12"


def _driver_env() -> dict[str, str]:
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


class CuvslamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/cuvslam_odometry"
    # `nix build` drops the `result` symlink in the cwd.
    build_command: str | None = Field(
        default_factory=lambda: f"nix build github:dimensionalOS/dimSLAM/v0.2.0#{sdk_variant()}"
    )
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    # "stereo" is two or more overlapping cameras; "mono" is accurate up to scale.
    camera_mode: Literal["stereo", "mono", "rgbd"] = "stereo"
    # Empty discovers a single camera or a single pair off camera_info.
    camera_frames: list[str] = Field(default_factory=list)
    # Asserts the images arrive rectified; it does not rectify them.
    rectified: bool = True
    # Off runs on the CPU, deterministic, and needs a libcuvslam built with
    # ENFORCE_GPU=OFF (the jeff-hykin/cuVSLAM fork); the stock SDK is GPU-only.
    use_gpu: bool = True

    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    # Frame the cuVSLAM rig is built in. Empty means base_frame. Output stays on base_frame
    # either way, the two differing by a fixed transform.
    rig_frame: str = ""
    # Only read when Slam is off, where map->odom can only be identity.
    publish_map_to_odom: bool = True

    # Without it map->odom is identity.
    enable_slam: bool = True
    # Slam's GetPose() carries no timestamp, so a thread running behind cannot be matched
    # to the odometry pose it corrects.
    slam_async: bool = False
    # Poses in the pose graph, not a distance. 0 is unlimited.
    slam_max_poses: int = 300
    slam_throttling_ms: int = 0
    # On, the tracker waits for the noise model on ``imu_info`` before building the rig.
    enable_imu: bool = False
    # Rebase guard: a frame whose translation std exceeds this has its motion dropped and
    # the path rebased onto the held pose. Metres; 0 publishes the raw integrator.
    # Well-constrained frames report 0.01-0.3 m, degenerate bursts 5-330 m.
    covariance_gate_translation_std: float = 1.0
    # The same rebase for implausible motion against the previous tracked frame, catching
    # confident teleports the covariance gate misses. Metres/second and radians/second;
    # above jogging (~4 m/s) and a fast handheld pan (~5 rad/s). 0 disables that limit.
    speed_gate_max_linear: float = 5.0
    speed_gate_max_angular: float = 12.0
    # rgbd only: raw depth units per metre. cuVSLAM assumes 1, and depth images are
    # 16-bit millimetres.
    depth_units_per_meter: float = 1000.0


class CuvslamOdometry(NativeModule):
    """Visual odometry on one to thirty-two cameras.

    Every camera publishes onto the same ``image`` and ``camera_info`` streams and is
    told apart by ``frame_id``; ``camera_frames`` fixes which frames are on the rig and
    in what order. Extrinsics come from tf against ``rig_frame``. ``rgbd`` pairs one
    camera with ``depth_image``, reprojected onto the rig camera through
    ``depth_camera_info`` and tf when the depth sensor differs.

    ``odometry`` is one continuous ``odom`` -> ``base_link`` path, rebased onto the last
    published pose after a tracking loss. ``corrected_odometry`` is the pose-graph
    ``map`` -> ``base_link`` and jumps at loop closures. ``tf`` carries
    ``odom`` -> ``base_link`` and ``map`` -> ``odom``.
    """

    config: CuvslamConfig

    image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    imu: In[Imu]
    imu_info: In[ImuInfo]

    odometry: Out[Odometry]
    corrected_odometry: Out[Odometry]
    tf: IO[TFMessage]
