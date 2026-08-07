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
from typing import Any, Literal

from pydantic import BaseModel, Field, model_validator

from dimos.constants import CACHE_DIR, CONFIG_DIR
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

MODULE_DIR = Path(__file__).resolve().parent

# The nix loader ignores ld.so.cache, so dlopen("libcuda.so.1") fails and cudart
# blames the driver version instead. Directories holding nothing but driver
# libraries go on the path whole, which Jetson needs: libcuda.so.1 there depends on
# a dozen siblings that live beside it.
_DRIVER_ONLY_LIB_DIRS = (
    # NixOS and CUDA containers.
    Path("/run/opengl-driver/lib"),
    # Jetson / L4T. Both names are the same set of files on JetPack 6; which one
    # exists has varied across releases, so try both.
    Path("/usr/lib/aarch64-linux-gnu/nvidia"),
    Path("/usr/lib/aarch64-linux-gnu/tegra"),
)
# Directories holding the driver among the system's own libraries: exposing one
# whole would shadow the binary's libstdc++, so symlinks to the driver stand in.
_HOST_LIB_DIRS = (
    Path("/usr/lib/x86_64-linux-gnu"),
    Path("/usr/lib/aarch64-linux-gnu"),
)
# nixpkgs tracks a newer CUDA than JetPack ships, and the mismatch fails at
# cusolverDnCreate, so the host's own runtime goes first where it exists. Holding
# only CUDA runtime libraries, it cannot shadow the binary's libstdc++.
_HOST_CUDA_LIB_DIR = Path("/usr/local/cuda/lib64")
_DRIVER_LIBS = (
    "libcuda.so.1",
    "libnvidia-ptxjitcompiler.so.1",
    "libnvidia-nvvm.so.4",
    "libnvidia-ml.so.1",
)
_DRIVER_LINK_DIR = CACHE_DIR / "nvidia-driver-libs"


def driver_library_dir() -> Path | None:
    """Directory the loader needs on LD_LIBRARY_PATH to find the NVIDIA driver."""
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
        if source.exists() and not link.is_symlink():
            link.symlink_to(source.resolve())
    return _DRIVER_LINK_DIR


def _driver_env() -> dict[str, str]:
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
    """How much one physical IMU lies.

    Where it sits is not here: that comes off tf, published by whatever driver owns the
    device. There is deliberately no default for the rest either, because the noise
    densities come from an Allan-variance run on a single unit and a module default
    means every other unit silently gets someone else's numbers.
    """

    gyro_noise_density: float
    gyro_random_walk: float
    accel_noise_density: float
    accel_random_walk: float
    # The rate actually fed, not the rate requested: cuVSLAM computes expected samples
    # per frame from this, and declaring more than it receives disables fusion silently.
    frequency: float

    @classmethod
    def for_serial(cls, serial: str) -> ImuCalibration | None:
        """Load ``<calibration dir>/imu_<serial>.yaml``, or None if there is none."""
        import yaml

        path = calibration_dir() / f"imu_{serial}.yaml"
        if not path.is_file():
            return None
        with path.open() as handle:
            return cls(**yaml.safe_load(handle))


def calibration_dir() -> Path:
    """Where per-unit calibration lives, keyed by device serial number."""
    return CONFIG_DIR / "dimos" / "calibration"


class CuvslamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/cuvslam_odometry"
    build_command: str | None = "nix build ."
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    # "stereo" tracks on two or more cameras with overlapping views, "rgbd" on one
    # image plus its depth, "mono" on one image alone -- and mono is accurate only up
    # to an unknown scale, so its poses are not metres.
    camera_mode: Literal["stereo", "mono", "rgbd"] = "stereo"
    # One tf frame per camera, in the order cuVSLAM indexes them. Empty discovers them
    # off camera_info, which only has an order for a single camera or one pair.
    camera_frames: list[str] = Field(default_factory=list)
    rectified: bool = True
    # cuVSLAM's async bundle adjustment throws from its own thread, where nothing can
    # catch it, and the process aborts. NVIDIA's launcher defaults it off too.
    async_sba: bool = False
    # A step implying the camera moved faster than this is cuVSLAM restarting its
    # world frame rather than real motion, so the module rebases instead of jumping.
    implausible_speed_meters_per_second: float = 10.0

    map_frame: str = "map"
    odom_frame: str = "odom"
    # Also the rig frame: every camera is placed against this one, so the pose cuVSLAM
    # returns is this frame's and nothing has to be re-referenced afterwards.
    base_frame: str = "base_link"
    # Only used when Slam is off: pure visual odometry has no global correction,
    # so its map->odom can only be identity. Turn it off whenever something else
    # in the graph owns that edge -- two publishers of one tf edge fight.
    publish_map_to_odom: bool = True

    # cuvslam::Slam on top of the odometry: pose graph, loop closure, and the
    # same-run relocalization that pulls a revisit back onto itself. Without it
    # map->odom carries nothing and the landmark map smears with the drift.
    enable_slam: bool = True
    slam_sync_mode: bool = True
    slam_max_map_size: int = 300
    slam_throttling_ms: int = 0
    # Off by default because for non-drone applications it usually hurts. Turning it on
    # requires imu_calibration and imu_frame: there is no default noise model, because
    # it belongs to one physical unit.
    enable_imu: bool = False
    imu_calibration: ImuCalibration | None = None
    # rgbd only: how many raw depth units make a metre. cuVSLAM divides by this, and
    # assumes 1 -- i.e. that the raw values are already metres. Depth images are 16-bit
    # millimetres, so left at 1 every point lands a thousand times too far away.
    depth_units_per_meter: float = 1000.0

    @model_validator(mode="after")
    def _imu_needs_calibration(self) -> CuvslamConfig:
        if self.enable_imu and self.imu_calibration is None:
            raise ValueError(
                "enable_imu is on but imu_calibration is unset. Load it for the camera "
                "actually plugged in -- ImuCalibration.for_serial(<serial>) reads "
                f"{calibration_dir()}/imu_<serial>.yaml -- rather than borrowing another "
                "unit's noise model."
            )
        return self

    def to_config_dict(self) -> dict[str, Any]:
        """Flatten the calibration, because the native struct is a plain aggregate.

        With the IMU off the values are never read, so zeros go across rather than
        stand-in numbers that would look like a calibration to anyone reading a log.
        """
        blob = super().to_config_dict()
        calibration = blob.pop("imu_calibration", None) or dict.fromkeys(
            ImuCalibration.model_fields, 0.0
        )
        return blob | {f"imu_{key}": value for key, value in calibration.items()}


class CuvslamOdometry(NativeModule):
    """Visual odometry on the GPU, on one to thirty-two cameras.

    Every camera publishes onto the same ``image`` and ``camera_info`` streams and is
    told apart by ``frame_id``, so a rig is a wiring change and not a new pair of ports.
    ``camera_frames`` fixes which frames are on it and in which order; leaving it empty
    discovers a single camera or a single pair. ``rgbd`` pairs one camera with
    ``depth_image``, whose ``frame_id`` says which camera the depth is aligned with.

    Where each camera sits is read from tf against ``base_frame``, which is therefore
    also the rig frame: the pose cuVSLAM returns is already the body's, and a stereo
    baseline is the distance between two frames rather than a number kept here.

    ``odometry`` is one continuous ``odom`` -> ``base_link`` path. cuVSLAM restarts its
    world frame after a tracking loss and each restart is rebased onto the last pose
    published, so the stream never jumps; the motion across a restart is lost, and only
    the log says so.

    ``corrected_odometry`` is the pose-graph pose, ``map`` -> ``base_link``, and it
    jumps at a loop closure -- which is the point. ``tf`` carries ``odom`` ->
    ``base_link`` and ``map`` -> ``odom``, so the jump lands on the edge allowed to
    jump. With ``enable_slam`` off, ``map`` -> ``odom`` is identity.
    """

    config: CuvslamConfig

    image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    imu: In[Imu]

    odometry: Out[Odometry]
    corrected_odometry: Out[Odometry]
    # Read for the mount tree the rig is built from, written with the pose.
    tf: IO[TFMessage]
