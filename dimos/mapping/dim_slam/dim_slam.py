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

from pydantic import BaseModel, Field, model_validator

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
        if link.is_symlink() and link.readlink() == target:
            continue
        # Two callers can reach this at once, and symlink_to over an existing path raises.
        staging = link.with_name(f"{name}.{uuid4().hex}.tmp")
        staging.symlink_to(target)
        os.replace(staging, link)
    return _DRIVER_LINK_DIR


def driver_cuda_major() -> int:
    """0 if there is no driver."""
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


class CameraConfig(BaseModel):
    """Settings that belong to one camera rather than to the tracker, keyed in
    ``DimSlamConfig.cameras`` by the ``frame_id`` its images carry. A depth stream is a
    camera of its own here, and its frame need not be one of ``camera_frames``."""

    # Asserted, not performed. cuVSLAM takes one flag for the whole rig, so the rig
    # cameras have to agree.
    rectified: bool = True
    # Raw depth units per metre; 1000 for the usual sixteen-bit millimetres.
    depth_units_per_meter: float = 1000.0
    # Range gate on the published depth_cloud, metres; 0 leaves either end open.
    depth_cloud_min_range: float = 0.0
    depth_cloud_max_range: float = 0.0
    # One point per k x k depth block (median of in-gate depths). <= 1 emits every pixel.
    depth_cloud_decimation: int = 1


class ImuConfig(BaseModel):
    """One IMU's datasheet noise figures, keyed in ``DimSlamConfig.imus`` by the
    ``frame_id`` its samples carry, in the continuous-time units the filter wants:
    rad/s/sqrt(Hz), rad/s^2/sqrt(Hz), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz)."""

    gyro_noise_density: float = 0.0
    gyro_random_walk: float = 0.0
    accel_noise_density: float = 0.0
    accel_random_walk: float = 0.0


class SourceConfig(BaseModel):
    """One odometry source, keyed in ``DimSlamConfig.sources`` by the transform it
    reports, written ``"parent_frame->child_frame"``. Both halves are needed because two
    sources can share a parent. A source whose parent is ``odom_frame`` is fused
    absolutely; anything else is fused as filter-anchored deltas, since its own pose has
    drifted."""

    # [x y z roll pitch yaw]: below zero takes the message covariance, zero drops the
    # dimension, above zero is a fixed variance.
    pose_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )
    # [vx vy vz wx wy wz], same convention, body frame.
    twist_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )


class DimSlamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR / "rust")
    executable: str = "result/bin/dim_slam"
    # The dimos-repo input is this repo, so writing the lock during evaluation changes the
    # tree being hashed and the build dies on its own edit.
    build_command: str | None = Field(
        default_factory=lambda: f"nix build -L --no-write-lock-file 'path:.#{sdk_variant()}'"
    )
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    camera_mode: Literal["stereo", "mono", "rgbd"] = "stereo"
    # Empty: auto-discover from camera_info. Kept a list rather than folded into
    # cameras because it carries cuVSLAM's rig order, and a JSON object's key order does
    # not survive deserialization.
    camera_frames: list[str] = Field(default_factory=list)
    # Keyed by the frame_id the images carry. An unlisted camera takes the defaults.
    cameras: dict[str, CameraConfig] = Field(default_factory=dict)
    # Off runs the deterministic CPU path, which needs a libcuvslam built
    # -DENFORCE_GPU=OFF. A build carrying only the other backend is used with a warning.
    use_gpu: bool = True

    # The tracker's own world frame, drifting freely; the filter fuses it as a
    # drifting source, so it must be the parent of a key in sources.
    visual_odom_frame: str = "visual_odom"
    # Frame the cuVSLAM rig is built in. Empty means base_frame.
    rig_frame: str = ""

    # cuVSLAM's Inertial mode: the stereo pair plus one IMU. The noise model and frame
    # come from the imu_info stream. Separate from use_imu, which feeds the fusion
    # filter. Implemented only on the CUDA path.
    cuvslam_enable_imu: bool = False
    # Translation std (m) above which the frame's motion is dropped and the path rebased;
    # 0 disables.
    covariance_gate_translation_std: float = 1.0
    # Frame-to-frame m/s and rad/s, catching teleports the covariance gate misses; 0 disables.
    speed_gate_max_linear: float = 5.0
    speed_gate_max_angular: float = 12.0

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
    # Caps the filter's own state rather than an incoming reading. 0 disables it.
    max_position_m: float = 10000.0

    # On, the filter propagates on IMU and needs one fully specified entry in imus. Off,
    # it is seeded level from the first source message and holds its pose between them.
    use_imu: bool = False

    # Keyed by the frame_id the samples carry. Samples from an unlisted frame are
    # dropped. The filter propagates on a single IMU, so use_imu needs exactly one entry.
    imus: dict[str, ImuConfig] = Field(default_factory=dict)
    gravity: float = 9.81
    # Averaged while stationary to take the gyro bias; leaving it in cost 19.8 m of final
    # error against 1.6 m on a 517 s drive. At 200 Hz this is one second of standing still.
    imu_init_samples: int = 200

    initial_position_std: float = 0.01
    initial_velocity_std: float = 0.1
    initial_rotation_std: float = 0.05
    initial_bias_std: float = 0.05

    # Keyed by "parent_frame->child_frame", matched against each message's
    # header.frame_id and child_frame_id. Odometry from an unlisted transform is dropped.
    sources: dict[str, SourceConfig] = Field(
        default_factory=lambda: {
            "visual_odom->base_link": SourceConfig(
                pose_variances=[0.01, 0.01, 0.01, 0.05, 0.05, 0.05]
            )
        }
    )
    # A virtual zero-twist measurement applied with every source message, for the
    # directions the platform cannot move in. Above zero pulls that dimension toward
    # zero with this variance; zero leaves it free.
    constraint_twist_variances: list[float] = Field(default_factory=lambda: [0.0] * 6)

    @model_validator(mode="after")
    def _config_keys_are_well_formed(self) -> DimSlamConfig:
        for key in self.sources:
            parent, arrow, child = key.partition("->")
            if not arrow or not parent.strip() or not child.strip():
                raise ValueError(f'source key {key!r} must be written "parent_frame->child_frame"')
        if not self.use_imu:
            return self
        if len(self.imus) != 1:
            raise ValueError(
                f"use_imu needs exactly one entry in imus, not {len(self.imus)}: "
                "the filter propagates on a single IMU"
            )
        frame_id, imu = next(iter(self.imus.items()))
        missing = [
            name
            for name in (
                "gyro_noise_density",
                "gyro_random_walk",
                "accel_noise_density",
                "accel_random_walk",
            )
            if getattr(imu, name) <= 0.0
        ]
        if missing:
            raise ValueError(f"use_imu needs {frame_id}'s noise figures: {', '.join(missing)}")
        return self


class DimSlam(NativeModule):
    """Every camera publishes onto the same ``image`` and ``camera_info`` streams and is
    told apart by ``frame_id``; ``camera_frames`` fixes which frames are on the rig and
    in what order. Extrinsics come from tf against ``rig_frame``. ``depth_image`` feeds
    ``depth_cloud`` in every mode, and is additionally tracked against in ``rgbd``,
    reprojected onto the rig camera through ``depth_camera_info`` and tf when the depth
    sensor differs.

    The tracker's pose stream never touches the wire: it enters the filter as a
    drifting source under ``visual_odom_frame``. Any number of external sources
    (wheel odometry, ...) publish onto ``sources`` and are told apart by
    the transform they report. Late messages roll the filter back to their own slot and replay
    everything after.
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
