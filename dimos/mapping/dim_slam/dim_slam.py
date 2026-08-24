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

"""Native Rust dimSLAM module: cuVSLAM visual odometry fused with an IMU and any
number of odometry sources by an error-state Kalman filter, in one process."""

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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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
        # Re-point after a driver upgrade.
        if link.is_symlink() and link.readlink() == target:
            continue
        # Two callers can reach this at once, and symlink_to over an existing path raises.
        staging = link.with_name(f"{name}.{uuid4().hex}.tmp")
        staging.symlink_to(target)
        os.replace(staging, link)
    return _DRIVER_LINK_DIR


def driver_cuda_major() -> int:
    """0 if there is no driver."""
    # By absolute path too, since ld.so.cache is not consulted here.
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
    (cuda12 vs cuda13 on x86) or /proc/device-tree (orin vs thor, both aarch64-linux).
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


class DimSlamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/dim_slam"
    # TODO: pin a tag once dimSLAM#depth2depth merges. It carries fused_odom plus the
    # depth2depth_* settings below, which fused_odom alone will not deserialize.
    build_command: str | None = Field(
        default_factory=lambda: (
            f"nix build 'github:dimensionalOS/dimSLAM?ref=depth2depth#{sdk_variant()}'"
        )
    )
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    camera_mode: Literal["stereo", "mono", "rgbd"] = "stereo"
    # Empty: auto-discover from camera_info.
    camera_frames: list[str] = Field(default_factory=list)
    # Asserted, not performed: unrectified input is not corrected.
    rectified: bool = True
    # Off runs on the CPU, deterministic, and needs a libcuvslam built with
    # ENFORCE_GPU=OFF (the jeff-hykin/cuVSLAM fork); the stock SDK is GPU-only.
    use_gpu: bool = True

    # The tracker's own world frame, drifting freely; the filter fuses it as a
    # drifting source, so it must appear in source_frames.
    visual_odom_frame: str = "visual_odom"
    # Frame the cuVSLAM rig is built in. Empty means base_frame.
    rig_frame: str = ""

    # cuVSLAM's Inertial mode: the stereo pair plus one IMU. The noise model and frame
    # come from the imu_info stream. Separate from use_imu, which feeds the fusion
    # filter. Implemented only on the CUDA path.
    cuvslam_enable_imu: bool = False
    # Translation std (m) above which the frame's motion is dropped and the path rebased.
    # Well-constrained frames measure 0.01-0.3 m, degenerate bursts 5-330 m or NaN; 0 disables.
    covariance_gate_translation_std: float = 1.0
    # Frame-to-frame m/s and rad/s, catching teleports the covariance gate misses; 0 disables.
    speed_gate_max_linear: float = 5.0
    speed_gate_max_angular: float = 12.0
    # Raw depth units per metre, 1000 for the usual sixteen-bit millimetres.
    depth_units_per_meter: float = 1.0
    # Range gate on the published depth_cloud, metres; 0 leaves either end open. Where to
    # cut is a property of the depth sensor, not of the tracker.
    depth_cloud_min_range: float = 0.0
    depth_cloud_max_range: float = 0.0
    # One point per k x k depth block (median of in-gate depths). <= 1 emits every pixel.
    depth_cloud_decimation: int = 1
    # Densify the depth image before the cloud is cut from it. Setting both safetensors
    # paths turns it on, and the binary has to be built with the depth2depth cargo
    # feature (depth2depth-cuda/-cudnn/-metal for a GPU backend).
    depth2depth_dinov2_weights: str = ""
    depth2depth_head_weights: str = ""
    # 1.0 = 280x504; 0.5 is ~4x faster and coarser.
    depth2depth_quality: float = 1.0
    # Frame whose images on the image stream feed the model; empty uses the rig camera
    # on the depth frame. Set to the color camera's frame when depth is aligned to a
    # camera that has no color, such as an infrared imager.
    depth2depth_color_frame: str = ""
    # A depth frame with no color inside this window gets an undensified cloud.
    depth2depth_max_color_skew_seconds: float = 0.5

    odom_frame: str = "odom"
    base_frame: str = "base_link"
    # Off publishes odometry only, for when something downstream owns odom -> base_frame.
    publish_tf: bool = True

    # The filter itself steps at the IMU rate; this is only how often it emits.
    publish_rate: float = 50.0
    replay_buffer_seconds: float = 0.5
    # Standard deviations per measurement dimension before a reading is called an
    # outlier. 0 disables the gate.
    mahalanobis_gate: float = 5.0

    # On, the filter propagates on IMU and needs all four noise figures below. Off, it is
    # seeded level from the first source message and coasts at constant world velocity.
    use_imu: bool = False

    # The IMU's datasheet noise figures, in the continuous-time units the filter wants:
    # rad/s/sqrt(Hz), rad/s^2/sqrt(Hz), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz). use_imu requires
    # all four above zero.
    imu_gyro_noise_density: float = 0.0
    imu_gyro_random_walk: float = 0.0
    imu_accel_noise_density: float = 0.0
    imu_accel_random_walk: float = 0.0
    gravity: float = 9.81
    # Averaged while stationary to take the gyro bias; leaving it in cost 19.8 m of final
    # error against 1.6 m on a 517 s drive. At 200 Hz this is one second of standing still.
    imu_init_samples: int = 200

    initial_position_std: float = 0.01
    initial_velocity_std: float = 0.1
    initial_rotation_std: float = 0.05
    initial_bias_std: float = 0.05

    # One entry per source, matched against each message's header.frame_id. A source
    # whose frame is odom_frame is fused absolutely; anything else is fused as
    # filter-anchored deltas, since its own pose has drifted.
    source_frames: list[str] = Field(default_factory=lambda: ["visual_odom"])
    # 6 per source, [x y z roll pitch yaw] then [vx vy vz wx wy wz]: below zero takes the
    # message covariance, zero drops the dimension, above zero is a fixed variance.
    source_pose_variances: list[float] = Field(
        default_factory=lambda: [0.01, 0.01, 0.01, 0.05, 0.05, 0.05]
    )
    source_twist_variances: list[float] = Field(default_factory=lambda: [0.0] * 6)
    # A virtual zero-twist measurement applied with every source message, for the
    # directions the platform cannot move in. Above zero pulls that dimension toward
    # zero with this variance; zero leaves it free.
    constraint_twist_variances: list[float] = Field(default_factory=lambda: [0.0] * 6)

    @model_validator(mode="after")
    def _imu_noise_is_set(self) -> DimSlamConfig:
        missing = [
            name
            for name in (
                "imu_gyro_noise_density",
                "imu_gyro_random_walk",
                "imu_accel_noise_density",
                "imu_accel_random_walk",
            )
            if getattr(self, name) <= 0.0
        ]
        if self.use_imu and missing:
            raise ValueError(f"use_imu needs the IMU's noise figures: {', '.join(missing)}")
        return self


class DimSlam(NativeModule):
    """Every camera publishes onto the same ``image`` and ``camera_info`` streams and is
    told apart by ``frame_id``; ``camera_frames`` fixes which frames are on the rig and
    in what order. Extrinsics come from tf against ``rig_frame``. ``rgbd`` pairs one
    camera with ``depth_image``, reprojected onto the rig camera through
    ``depth_camera_info`` and tf when the depth sensor differs.

    The tracker's pose stream never touches the wire: it enters the filter as a
    drifting source under ``visual_odom_frame``. Any number of external sources
    (wheel odometry, ...) publish onto ``sources`` and are told apart by
    ``header.frame_id``, so adding one is a config change rather than a port change.
    Late messages roll the filter back to their own slot and replay everything after.

    ``odometry`` is the fused ``odom_frame`` -> ``base_frame`` pose, and ``tf`` carries
    the same edge.
    """

    config: DimSlamConfig

    image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    imu: In[Imu]
    imu_info: In[ImuInfo]
    sources: In[Odometry]

    odometry: Out[Odometry]
    depth_cloud: Out[PointCloud2]
    tf: IO[TFMessage]
