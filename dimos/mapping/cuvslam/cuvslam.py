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

import ctypes
import os
from pathlib import Path
import platform
import sys
from typing import Any, Literal

from pydantic import BaseModel, Field, model_validator

from dimos.constants import CACHE_DIR
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MODULE_DIR = Path(__file__).resolve().parent

# The nix loader ignores ld.so.cache, so dlopen("libcuda.so.1") fails and cudart blames
# the driver version instead. Jetson needs the whole directory: libcuda.so.1 there
# depends on its siblings.
_DRIVER_ONLY_LIB_DIRS = (
    Path("/run/opengl-driver/lib"),
    # Jetson / L4T. Which of the two names exists has varied across releases.
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
    """Directory to put on LD_LIBRARY_PATH for the NVIDIA driver."""
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
        # Re-point after a driver upgrade; a stale link would dangle forever.
        if link.is_symlink() and link.resolve() != source.resolve():
            link.unlink()
        if not link.is_symlink():
            link.symlink_to(source.resolve())
    return _DRIVER_LINK_DIR


def driver_cuda_major() -> int:
    """The CUDA major the installed driver supports, or 0 if there is no driver."""
    # By absolute path too: a nix-built python ignores ld.so.cache, where the bare
    # name resolves everywhere else.
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
    """Pick the dim-slam flake attr for this host.

    Python picks instead of the flake's default package because nix evaluation is
    hermetic: it can branch on arch/OS only, and cannot see the installed driver
    (cuda12 vs cuda13 on x86) or /proc/device-tree (orin vs thor, both
    aarch64-linux). Those need this host-side probe.
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


class ImuCalibration(BaseModel):
    """Noise model for one physical IMU, measured per unit. Where it sits comes from tf."""

    gyro_noise_density: float
    gyro_random_walk: float
    accel_noise_density: float
    accel_random_walk: float
    # The rate actually fed. Declaring more than arrives disables fusion, silently.
    frequency: float


class CuvslamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/cuvslam_odometry"
    # The C++ lives in dim-slam (cuVSLAM + the module built on it); dimos just
    # builds the pinned tag. `nix build` drops the `result` symlink in the cwd.
    build_command: str | None = Field(
        default_factory=lambda: f"nix build github:jeff-hykin/dim-slam/v0.1.0#{sdk_variant()}"
    )
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    # "stereo" is two or more overlapping cameras; "mono" is accurate up to scale.
    camera_mode: Literal["stereo", "mono", "rgbd"] = "stereo"
    # Empty discovers a single camera or a single pair off camera_info.
    camera_frames: list[str] = Field(default_factory=list)
    # Asserts the images arrive rectified: no distortion, rows already aligned.
    rectified: bool = True
    # Off runs the tracker on the CPU (deterministic, no CUDA). Needs a libcuvslam built
    # with ENFORCE_GPU=OFF (the jeff-hykin/cuVSLAM fork); NVIDIA's stock SDK is GPU-only.
    use_gpu: bool = True

    map_frame: str = "map"
    odom_frame: str = "odom"
    # Poses are published relative to this.
    base_frame: str = "base_link"
    # Frame the cuVSLAM rig is built in. Empty means base_frame. Pointing it at a camera's
    # optical frame reproduces NVIDIA's examples, whose rig is the left camera; output stays
    # on base_frame either way, the two differing by a fixed transform.
    rig_frame: str = ""
    # Only read when Slam is off, where map->odom can only be identity.
    publish_map_to_odom: bool = True

    # Pose graph and loop closure; without it map->odom is identity.
    enable_slam: bool = True
    # Runs Slam on its own thread. Its GetPose() carries no timestamp, so a thread running
    # behind cannot be matched to the odometry pose it corrects.
    slam_async: bool = False
    # Poses in the pose graph, not a distance. 0 is unlimited.
    slam_max_poses: int = 300
    slam_throttling_ms: int = 0
    enable_imu: bool = False
    imu_calibration: ImuCalibration | None = None
    # Rebase guard: a frame whose translation standard deviation (root of the largest
    # translation term of cuVSLAM's covariance) exceeds this has its motion dropped and the
    # path rebased onto the held pose, so the published odometry never carries a teleport
    # from an unconstrained scene. Meters; 0 publishes the raw integrator untouched.
    # Measured: well-constrained frames report 0.01-0.3 m, degenerate bursts (blank wall,
    # repeated texture) 5-330 m or NaN, so 1.0 separates them by an order of magnitude.
    covariance_gate_translation_std: float = 1.0
    # Rebase guard on physically implausible frame-to-frame motion, in metres/second and
    # radians/second against the previous tracked frame. Catches confident teleports the
    # covariance gate misses (0.9 m in one 33 ms frame is 27 m/s) without trusting the
    # tracker's self-report; VINS-Mono's failureDetection() gates the same way. Linear sits
    # above any handheld or robot speed (jogging is ~4 m/s), angular deliberately high --
    # a fast handheld pan peaks near 5 rad/s. 0 disables that limit.
    speed_gate_max_linear: float = 5.0
    speed_gate_max_angular: float = 12.0
    # rgbd only: raw depth units per metre. cuVSLAM assumes 1, and depth images are
    # 16-bit millimetres.
    depth_units_per_meter: float = 1000.0

    @model_validator(mode="after")
    def _imu_needs_calibration(self) -> CuvslamConfig:
        if self.enable_imu and self.imu_calibration is None:
            raise ValueError(
                "enable_imu is on but imu_calibration is unset. Measure it for the unit "
                "actually plugged in rather than borrowing another one's noise model."
            )
        return self

    def to_config_dict(self) -> dict[str, Any]:
        """Flatten the calibration into imu_* keys; the native struct is a plain aggregate."""
        blob = super().to_config_dict()
        calibration = blob.pop("imu_calibration", None) or dict.fromkeys(
            ImuCalibration.model_fields, 0.0
        )
        return blob | {f"imu_{key}": value for key, value in calibration.items()}


class CuvslamOdometry(NativeModule):
    """Visual odometry on the GPU, on one to thirty-two cameras.

    Every camera publishes onto the same ``image`` and ``camera_info`` streams and is
    told apart by ``frame_id``; ``camera_frames`` fixes which frames are on the rig and
    in what order. Extrinsics come from tf against ``base_frame``, which is also the
    rig frame. ``rgbd`` pairs one camera with ``depth_image``. Depth recorded against a
    different sensor than the rig camera (a D455 aligns depth to the left IR camera, not
    color) is reprojected onto the rig camera through ``depth_camera_info`` and tf.

    ``odometry`` is one continuous ``odom`` -> ``base_link`` path; restarts after a
    tracking loss are rebased onto the last published pose. ``corrected_odometry`` is
    the pose-graph ``map`` -> ``base_link`` and jumps at loop closures. ``tf`` carries
    ``odom`` -> ``base_link`` and ``map`` -> ``odom``.
    """

    config: CuvslamConfig

    image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    imu: In[Imu]

    odometry: Out[Odometry]
    corrected_odometry: Out[Odometry]
    tf: IO[TFMessage]
