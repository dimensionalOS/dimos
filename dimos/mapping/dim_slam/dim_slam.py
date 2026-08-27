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
    if hardware == "darwin-apple-silicon":
        return "metal"
    if hardware == "darwin-intel":
        raise RuntimeError("cuVSLAM has no Intel-mac build; it needs Apple silicon.")
    # cu_vslam_rs carries no CPU-fallback variant for these, so there is nothing to build.
    if hardware in ("xavier", "nano"):
        raise RuntimeError(f"cuVSLAM ships no JetPack 4/5 build, so {hardware} is unsupported.")
    if hardware == "linux-arm-no-nvidia":
        raise RuntimeError("cuVSLAM ships no build for non-Jetson ARM.")
    if hardware == "linux-x86-no-nvidia":
        # fork-built with ENFORCE_GPU=OFF, so it runs without a driver
        logger.warning("No NVIDIA driver found; only use_gpu=False will work.")
        return "x86_64-cuda12"
    major = detect_cuda_major()
    if major < 12:
        logger.warning(
            "This NVIDIA driver supports CUDA %d and the GPU path needs 12+; "
            "only use_gpu=False will work until the driver is upgraded.",
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
    cwd: str | None = str(MODULE_DIR / "rust")
    executable: str = "result/bin/dim_slam"
    build_command: str | None = Field(
        default_factory=lambda: f"nix build -L 'path:.#{sdk_variant()}'"
    )
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    # no default: this choice changes what inputs are required
    camera_mode: Literal["mono", "stereo", "rgbd"]
    # Empty: auto-discover from camera_info.
    camera_frames: list[str] = Field(default_factory=list)
    # Asserted, not performed.
    rectified: bool = True
    # Off runs the deterministic CPU path, which needs a libcuvslam built
    # -DENFORCE_GPU=OFF. A build carrying only the other backend is used with a warning.
    use_gpu: bool = True

    # The tracker's own world frame, drifting freely; the filter fuses it as a
    # drifting source, so it must appear in source_frames.
    visual_odom_frame_id: str = "visual_odom"
    # Frame the cuVSLAM rig is built in. Empty means output_frame_id.
    rig_frame_id: str = ""
    # Carried for whatever consumes the loop-closed pose; nothing here publishes map -> odom.
    map_frame_id: str = "map"

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
    # Raw depth units per metre, keyed by the depth image's frame_id; 16-bit millimetre
    # depth is 1000. Scaling one camera's depth by another's factor yields a
    # plausible-looking wrong map, so an unlisted frame is dropped rather than guessed at.
    frame_id_to_depth_units_per_meter: dict[str, float] = Field(default_factory=dict)
    # Range gate on the published depth_cloud, metres. Stereo depth error grows as range
    # squared, so the far gate decides whether the cloud is worth mapping with; 0 leaves
    # it open.
    depth_cloud_min_range: float = 0.0
    depth_cloud_max_range: float = 0.0
    # One point per k x k depth block (median of in-gate depths). <= 1 emits every pixel.
    depth_cloud_decimation: int = 1

    odom_frame_id: str = "odom"
    output_frame_id: str = "base_link"
    # Off when something downstream owns odom -> output_frame_id.
    publish_tf: bool = True

    # The filter itself steps at the IMU rate; this is only how often it emits.
    publish_rate: float = 50.0
    replay_buffer_seconds: float = 0.5
    # Standard deviations per measurement dimension before a reading is called an
    # outlier. 0 disables the gate.
    mahalanobis_gate: float = 5.0
    # Caps the filter's own state rather than an incoming reading. 0 disables it.
    max_position_m: float = 10000.0

    # On, the filter propagates on IMU and needs all four noise figures below. Off, it is
    # seeded level from the first source message and holds its pose between them.
    use_imu: bool = False

    # The IMU's datasheet noise figures, in the continuous-time units the filter wants:
    # rad/s/sqrt(Hz), rad/s^2/sqrt(Hz), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz). use_imu requires
    # all four above zero.
    imu_gyro_noise_density: float = 0.0
    imu_gyro_random_walk: float = 0.0
    imu_accel_noise_density: float = 0.0
    imu_accel_random_walk: float = 0.0
    gravity: float = 9.81
    # Averaged while stationary to level the filter and take the gyro bias; leaving that
    # bias in cost 19.8 m of final error against 1.6 m on a 517 s Alfred drive. At 200 Hz
    # this is one second of standing still at startup.
    imu_init_samples: int = 200

    initial_position_std: float = 0.01
    initial_velocity_std: float = 0.1
    initial_rotation_std: float = 0.05
    initial_bias_std: float = 0.05

    # One entry per source, matched against each message's header.frame_id. A source
    # whose frame is odom_frame_id is fused absolutely; anything else is fused as
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

    @model_validator(mode="after")
    def _mode_combinations(self) -> DimSlamConfig:
        # cuVSLAM has no inertial mono or rgbd, so the flag would be dropped on the floor.
        if self.cuvslam_enable_imu and self.camera_mode != "stereo":
            raise ValueError(
                f"cuvslam_enable_imu needs camera_mode='stereo', not {self.camera_mode!r}"
            )
        # cuVSLAM carries a single global depth scale, so rgbd can serve only one stream;
        # a second entry would be applied to the wrong camera.
        if self.camera_mode == "rgbd" and len(self.frame_id_to_depth_units_per_meter) != 1:
            raise ValueError(
                "camera_mode='rgbd' needs exactly one frame_id_to_depth_units_per_meter "
                f"entry, got {self.frame_id_to_depth_units_per_meter}"
            )
        return self

    @model_validator(mode="after")
    def _sources_are_fusable(self) -> DimSlamConfig:
        # The tracker's pose only reaches the filter as a source under this frame.
        if self.visual_odom_frame_id not in self.source_frames:
            raise ValueError(
                f"visual_odom_frame_id {self.visual_odom_frame_id!r} is missing from source_frames "
                f"{self.source_frames}, so the visual odometry would never be fused"
            )
        # The native asserts these lengths; catching it here names the offender.
        for name in ("source_pose_variances", "source_twist_variances"):
            values = getattr(self, name)
            if len(values) != len(self.source_frames) * 6:
                raise ValueError(
                    f"{name} needs 6 entries per source frame: expected "
                    f"{len(self.source_frames) * 6} for {self.source_frames}, got {len(values)}"
                )
        if len(self.constraint_twist_variances) != 6:
            raise ValueError(
                "constraint_twist_variances needs 6 entries, got "
                f"{len(self.constraint_twist_variances)}"
            )
        # Zero drops a dimension, so an all-zero source is fused in no dimension at all.
        for index, frame in enumerate(self.source_frames):
            pose = self.source_pose_variances[index * 6 : index * 6 + 6]
            twist = self.source_twist_variances[index * 6 : index * 6 + 6]
            if not any(pose) and not any(twist):
                raise ValueError(
                    f"source {frame!r} has every pose and twist variance at zero, "
                    "which fuses nothing; use a positive variance or drop the source"
                )
        return self


class DimSlam(NativeModule):
    """Every camera publishes onto the same ``image`` and ``camera_info`` streams and is
    told apart by ``frame_id``; ``camera_frames`` fixes which frames are on the rig and
    in what order. Extrinsics come from tf against ``rig_frame_id``. ``depth_image`` feeds
    ``depth_cloud`` in every mode, and is additionally tracked against in ``rgbd``,
    reprojected onto the rig camera through ``depth_camera_info`` and tf when the depth
    sensor differs.

    The tracker's pose stream never touches the wire: it enters the filter as a
    drifting source under ``visual_odom_frame_id``. Any number of external sources
    (wheel odometry, ...) publish onto ``sources`` and are told apart by
    ``header.frame_id``. Late messages roll the filter back to their own slot and replay
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
