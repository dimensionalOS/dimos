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

from pydantic import Field, model_validator

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
    cwd: str | None = "."
    executable: str = "result/bin/cuvslam_odometry"
    build_command: str | None = Field(
        default_factory=lambda: f"nix build github:dimensionalOS/dimSLAM/v0.3.2#{sdk_variant()}"
    )
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    # no default: this choice changes what inputs are required
    # multisensor (experimental in cuVSLAM): any mix of RGB and RGB-D cameras, optional IMU;
    # cameras listed in frame_id_to_depth_units_per_meter are the depth-providing ones
    camera_mode: Literal["mono", "stereo", "rgbd", "multisensor"]
    # reject way-too-fast movements, like moving a large box that takes up 90% of the camera view
    speed_gate_max_linear: float = 5.0  # meters per sec
    speed_gate_max_angular: float = 12.0  # radians per sec
    # "reject movements that cuVSLAM is not confident about"
    # good movements usually have a confidence of around 0.01-0.3, degenerate ones are 5-330
    # units = standard deviations of translation in meters
    # 0 disables
    covariance_gate_translation_std: float = 1.0

    # note: this is auto detected (including left/right via tf),
    # basically only use this for multi-cam setups, or for selecting a subset of cameras by frame_id
    # if in stereo mode, first=left side, second=right side
    # and left/right means relative to the camera
    # e.g. flipping a camera upside down should not change the order here
    camera_frames: list[str] = Field(default_factory=list)
    # have the images been pre-un-distorted (ideally yes, the sensor or API should un-distort)
    rectified: bool = True
    # works with MacOS Metal (apple silicon only) and Nvidia
    use_gpu: bool = True
    # rgbd (required) and multisensor: raw depth units per metre keyed by depth image frame_id
    # (e.g. {"d455_color_optical_frame": 1000.0} for 16-bit millimetre depth)
    frame_id_to_depth_units_per_meter: dict[str, float] = Field(default_factory=dict)

    map_frame_id: str = "map"
    odom_frame_id: str = "odom"
    output_frame_id: str = "base_link"
    # rig_frame_id will default to output_frame_id, which is right 99% of the time
    # sometimes this will need to be the head-frame rather than chest/base_link (ex: X2)
    rig_frame_id: str = ""

    enable_loop_closure: bool = True
    publish_map_to_odom: bool = True
    slam_async: bool = False
    # 0 = unlimited
    slam_max_poses: int = 300
    slam_throttling_ms: int = 0
    # unless you have a REALLY good IMU keep this off
    enable_imu: bool = False

    @model_validator(mode="after")
    def _check_mode_combinations(self) -> CuvslamConfig:
        if self.camera_mode == "rgbd" and not self.frame_id_to_depth_units_per_meter:
            raise ValueError(
                "camera_mode='rgbd' requires frame_id_to_depth_units_per_meter, "
                'e.g. {"d455_color_optical_frame": 1000.0}'
            )
        if self.enable_imu and self.camera_mode not in ("stereo", "multisensor"):
            raise ValueError(
                "enable_imu requires camera_mode='stereo' or 'multisensor'; "
                "cuVSLAM has no inertial mono or rgbd mode"
            )
        if self.camera_mode == "multisensor" and not self.camera_frames:
            raise ValueError(
                "camera_mode='multisensor' requires camera_frames; "
                "auto-discovery cannot know how many cameras the rig has"
            )
        return self


class CuvslamOdometry(NativeModule):
    """Visual odometry on one to thirty-two cameras.

    Usage
    1. Funnel all ir and rgb camera images to ``image``
    2. Funnel their respective camera info's to ``camera_info``
    3. Make sure all of those^ have correct frame_id's
    4. Publish static tf's to relate all those frame_id's
    5. Set `camera_mode` to mono/stereo/rgbd
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
