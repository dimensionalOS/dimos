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


class CameraConfig(BaseModel):
    """Settings for one camera, keyed by the frame_id its images carry. A depth stream is a
    camera of its own here, and need not be a rig camera."""

    # Asserted, not performed. cuVSLAM takes one flag for the whole rig, so the rig's
    # cameras have to agree.
    rectified: bool = True
    # Raw depth units per metre; 16-bit millimetre depth is 1000. Scaling one camera's depth
    # by another's factor yields a plausible-looking wrong map.
    depth_units_per_meter: float = 1000.0
    # Range gate on the published depth_cloud, metres. Stereo depth error grows as range
    # squared, so the far gate decides whether the cloud is worth mapping with; 0 leaves it
    # open.
    depth_cloud_min_range: float = 0.0
    depth_cloud_max_range: float = 0.0
    # One point per k x k depth block (median of in-gate depths). <= 1 emits every pixel.
    depth_cloud_decimation: int = 0


# Datasheet values, in the continuous-time units the filter wants: rad/s/sqrt(Hz),
# rad/s^2/sqrt(Hz), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz). Named so the config check can insist on them.
IMU_NOISE_FIGURES = (
    "gyro_noise_density",
    "gyro_random_walk",
    "accel_noise_density",
    "accel_random_walk",
)


class ImuConfig(BaseModel):
    """One physical IMU: its noise figures and how long it has to hold still to init."""

    gyro_noise_density: float = 0.0
    gyro_random_walk: float = 0.0
    accel_noise_density: float = 0.0
    accel_random_walk: float = 0.0

    # Averaged while stationary to level the filter and take the gyro bias; leaving that
    # bias in cost 19.8 m of final error against 1.6 m on a 517 s Alfred drive. At 200 Hz
    # this is one second of standing still at startup.
    init_samples: int = 200
    # rad/s. Above this the robot is called moving and bias calibration restarts, so it
    # belongs above this gyro's own bias and below any real rotation. A noisy gyro that
    # reads above it at rest never finishes init.
    init_gyro_limit: float = 0.05


class SourceConfig(BaseModel):
    """How much one odometry source is trusted. Below zero takes the message covariance,
    zero drops that dimension, above zero is a fixed variance. A drifting source's
    covariance describes its accumulated drift rather than the delta being fused, so a
    fixed value is usually the right answer."""

    # [x y z roll pitch yaw]
    pose_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )
    # [vx vy vz wx wy wz], body frame
    twist_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )


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
    # Keyed by the frame_id the camera's images carry; an absent camera takes the defaults.
    cameras: dict[str, CameraConfig] = Field(default_factory=dict)
    # Off runs the deterministic CPU path, which needs a libcuvslam built
    # -DENFORCE_GPU=OFF. A build carrying only the other backend is used with a warning.
    use_gpu: bool = True

    # The tracker's own world frame, drifting freely; the filter fuses it as a
    # drifting source, so it must key one of `sources`.
    visual_odom_frame_id: str = "visual_odom"
    # Frame the cuVSLAM rig is built in. Empty means output_frame_id.
    rig_frame_id: str = ""
    # Carried for whatever consumes the loop-closed pose; nothing here publishes map -> odom.
    map_frame_id: str = "map"

    # Stamp spread one frame set may span, milliseconds; 0 keeps cuVSLAM's 1 ms contract. A
    # software-triggered rig needs this widened: Spot's images land within ~15 ms of each
    # other and its depth trails its camera by up to ~90 ms, so every set is dropped at 1 ms.
    max_skew_ms: float = 0.0
    # Translation std (m) above which the frame's motion is dropped and the path rebased;
    # 0 disables.
    covariance_gate_translation_std: float = 1.0
    # Frame-to-frame m/s and rad/s, catching teleports the covariance gate misses; 0 disables.
    speed_gate_max_linear: float = 5.0
    speed_gate_max_angular: float = 12.0

    odom_frame_id: str = "odom"
    output_frame_id: str = "base_link"
    # Off when something downstream owns odom -> output_frame_id.
    publish_tf: bool = True

    # The filter itself steps at the IMU rate; this is only how often it emits.
    publish_rate: float = 50.0
    replay_buffer_seconds: float = 0.5
    # Outlier gate in variance units, per measurement dimension. Smaller is more
    # aggressive, 0 is no gate.
    outlier_rejection_allowed_variance: float = 25.0
    # Caps the filter's own state rather than an incoming reading. 0 disables it.
    max_position_m: float = 10000.0

    # Keyed by the frame_id the IMU's samples carry. Empty disables the IMU: the filter is
    # seeded level from the first source message and holds its pose between them. It
    # propagates on a single IMU, so at most one fully specified entry.
    imus: dict[str, ImuConfig] = Field(default_factory=dict)
    # m/s^2, seeding the filter rather than fixing it: a ZUPT is meant to refine it later.
    # Worth setting only on good hardware. Local gravity runs 9.780 at the equator to 9.832
    # at the poles, a 0.07 spread, and altitude is a tenth of that (Everest costs 0.027).
    # The BMI055 in a D455 has a 0.69 zero-g offset, ten times the whole spread, so there
    # the number is unmeasurable; an ADIS16505 repeats to 0.02 and can tell the difference.
    initial_gravity_estimate: float = 9.8

    initial_position_std: float = 0.01
    initial_velocity_std: float = 0.1
    initial_rotation_std: float = 0.05
    initial_bias_std: float = 0.05

    # One entry per source, keyed "parent_frame_id->child_frame_id" and matched against each
    # message's header.frame_id and child_frame_id. Both halves are needed because two
    # sources can share a parent. A source whose parent is odom_frame_id is fused
    # absolutely; anything else is fused as filter-anchored deltas, since its own pose has
    # drifted.
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
    constraint_twist_variances: list[float] = Field(
        default_factory=lambda: [0.0] * 6, min_length=6, max_length=6
    )

    @model_validator(mode="after")
    def _imu_noise_is_set(self) -> DimSlamConfig:
        if not self.imus:
            return self
        # The filter propagates on one IMU, so a second entry would silently go unused.
        if len(self.imus) > 1:
            raise ValueError(f"the filter propagates on a single IMU, got {list(self.imus)}")
        frame, imu = next(iter(self.imus.items()))
        missing = [name for name in IMU_NOISE_FIGURES if getattr(imu, name) <= 0.0]
        if missing:
            raise ValueError(f"IMU {frame!r} needs its noise figures set: {', '.join(missing)}")
        if imu.init_gyro_limit <= 0.0:
            raise ValueError(f"IMU {frame!r}'s init_gyro_limit must be above zero to ever init")
        return self

    @model_validator(mode="after")
    def _sources_are_fusable(self) -> DimSlamConfig:
        # The native parses these keys and refuses a malformed one; catching it here names
        # the offender against the config that wrote it.
        for key in self.sources:
            parent, separator, child = key.partition("->")
            if not separator or not parent.strip() or not child.strip():
                raise ValueError(
                    f"source key {key!r} must be written 'parent_frame_id->child_frame_id'"
                )
        # The tracker's pose only reaches the filter as a source under this transform.
        visual_odom_key = f"{self.visual_odom_frame_id}->{self.output_frame_id}"
        if visual_odom_key not in self.sources:
            raise ValueError(
                f"source {visual_odom_key!r} is missing from sources {list(self.sources)}, "
                "so the visual odometry would never be fused"
            )
        # Zero drops a dimension, so an all-zero source is fused in no dimension at all.
        for key, source in self.sources.items():
            if not any(source.pose_variances) and not any(source.twist_variances):
                raise ValueError(
                    f"source {key!r} has every pose and twist variance at zero, "
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
    ``header.frame_id`` and ``child_frame_id``. Late messages roll the filter back and replay
    everything after.
    """

    config: DimSlamConfig

    image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    imu: In[Imu]
    sources: In[Odometry]

    odometry: Out[Odometry]
    depth_cloud: Out[PointCloud2]
    tf: IO[TFMessage]
