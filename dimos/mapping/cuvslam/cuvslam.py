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
import time
from typing import Any

from pydantic import BaseModel, Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

MODULE_DIR = Path(__file__).resolve().parent
_REPO_ROOT = MODULE_DIR.parents[2]

# baseline_m left at this reads the baseline off the right camera_info instead.
DERIVE_BASELINE_FROM_CAMERA_INFO = 0.0

# The nix binary runs under the nix loader, whose ld.so.cache does not list the
# host driver, so dlopen("libcuda.so.1") fails and cudart reports the misleading
# "driver version is insufficient for CUDA runtime version" -- with the driver
# version it printed alongside it being 0.0, because there is no driver loaded to
# ask.
#
# Directories that hold *nothing but* driver libraries can go on LD_LIBRARY_PATH
# whole. That is required on Jetson (L4T), where libcuda.so.1 is not
# self-contained: it NEEDs libnvrm_gpu.so and libnvrm_mem.so, which pull in a
# dozen more siblings, all living beside it. Exposing libcuda.so.1 on its own
# leaves those unresolvable and the dlopen still fails.
_DRIVER_ONLY_LIB_DIRS = (
    # NixOS and CUDA containers.
    Path("/run/opengl-driver/lib"),
    # Jetson / L4T. Both names are the same set of files on JetPack 6; which one
    # exists has varied across releases, so try both.
    Path("/usr/lib/aarch64-linux-gnu/nvidia"),
    Path("/usr/lib/aarch64-linux-gnu/tegra"),
)
# Directories that hold the driver among the rest of the system's libraries.
# Exposing one of these whole would shadow the binary's own libstdc++, so a
# directory of symlinks to just the driver libs stands in for it. That is enough
# here only because on these systems libcuda.so.1 depends on nothing but libc.
_HOST_LIB_DIRS = (
    Path("/usr/lib/x86_64-linux-gnu"),
    Path("/usr/lib/aarch64-linux-gnu"),
)
# nixpkgs tracks a newer CUDA than JetPack ships, and a 12.9 cuSOLVER against a 12.6
# driver fails at cusolverDnCreate with INTERNAL_ERROR. Where the host has its own
# matching runtime, it goes first so the versions agree. This directory holds only CUDA
# runtime libraries, so unlike a full system lib dir it cannot shadow the binary's
# libstdc++.
_HOST_CUDA_LIB_DIR = Path("/usr/local/cuda/lib64")
_DRIVER_LIBS = (
    "libcuda.so.1",
    "libnvidia-ptxjitcompiler.so.1",
    "libnvidia-nvvm.so.4",
    "libnvidia-ml.so.1",
)
_DRIVER_LINK_DIR = Path.home() / ".cache/dimos/nvidia-driver-libs"


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
    """One physical camera's IMU calibration: where it sits, and how much it lies.

    There is deliberately no default. The extrinsic belongs to a single unit, and the
    noise densities come from an Allan-variance run on that unit; carrying either as a
    module default means every other camera silently gets someone else's numbers.
    """

    # rig_from_imu, the rig being the left camera.
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float
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
    override = os.environ.get("DIMOS_CALIBRATION_DIR")
    if override:
        return Path(override)
    return Path.home() / ".config/dimos/calibration"


class CuvslamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/cuvslam_odometry"
    build_command: str | None = f"nix build '{_REPO_ROOT}?dir=dimos/mapping/cuvslam'"
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    # Metric scale. Read from the right camera_info's P[3] unless set here, so a
    # different camera model is a different baseline without touching this file.
    baseline_m: float = DERIVE_BASELINE_FROM_CAMERA_INFO
    rectified: bool = True
    # Off because the port is ``_landmarks``, which the coordinator does not wire.
    # Rename it to publish, and expect the bandwidth: every tracked point, every frame.
    publish_landmarks: bool = False
    # cuVSLAM's asynchronous bundle adjustment thread races and throws
    # std::out_of_range from that thread, where no caller-side handler can catch it,
    # so the process aborts. NVIDIA's own launcher already defaults it off. Off also
    # makes cuVSLAM deterministic; on, ATE varied 0.406-3.418 m across identical runs.
    async_sba: bool = False
    # No indoor robot travels this fast, so a step implying it is cuVSLAM
    # restarting its world frame rather than motion.
    max_speed_mps: float = 10.0

    map_frame: str = "map"
    odom_frame: str = "odom"
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
    # Inertial mode. Off because it lost on every recording measured -- mildly at
    # walking pace, 4x at jogging pace -- and no gravity, excitation or time-offset
    # correction recovered it. Turning it on requires imu_calibration: there is no
    # default, because a camera-to-IMU extrinsic belongs to one physical unit.
    enable_imu: bool = False
    imu_calibration: ImuCalibration | None = None

    def to_config_dict(self) -> dict[str, Any]:
        """Flatten the calibration, because the native struct is a plain aggregate.

        With the IMU off the values are never read, so zeros go across rather than
        stand-in numbers that would look like a calibration to anyone reading a log.
        """
        blob = super().to_config_dict()
        calibration = blob.pop("imu_calibration", None)
        if blob.get("enable_imu") and calibration is None:
            raise ValueError(
                "enable_imu is on but imu_calibration is unset. Load it for the camera "
                f"actually plugged in -- ImuCalibration.for_serial(<serial>) reads "
                f"{calibration_dir()}/imu_<serial>.yaml -- rather than borrowing another "
                "unit's extrinsic."
            )
        empty = dict.fromkeys(ImuCalibration.model_fields, 0.0)
        for key, value in (calibration or empty).items():
            blob[f"imu_{key}"] = value
        return blob


class CuvslamOdometry(NativeModule):
    """Stereo visual odometry on the GPU.

    ``odometry`` is one continuous ``odom`` -> ``base_link`` path: cuVSLAM restarts
    its world frame after a tracking loss and the module rebases each restart onto
    the last published pose, so the stream never jumps. A restart costs the motion
    that happened across it, and is reported only on the log.

    ``_landmarks`` are the 3D points cuVSLAM is tracking this frame, carried into
    the ``odom`` frame. They are a live view rather than an accumulated map, so
    they carry the odometry's drift; put them through ``map`` -> ``odom`` to get
    the corrected positions. Underscored, so nothing subscribes and nothing is
    published unless the port is renamed and ``publish_landmarks`` turned on.

    ``camera_info`` is the left imager's, and ``camera_info_right`` the right's.
    Both are required: the left carries the intrinsics, and the right carries the
    baseline in ``P[3]``, which is the only per-unit source of metric scale.

    ``corrected_odometry`` is the pose-graph pose, ``map`` -> ``base_link``. It jumps
    at a loop closure, which is the point: that is where a revisit gets pulled back
    onto itself. ``tf`` carries ``odom`` -> ``base_link`` from the odometry and
    ``map`` -> ``odom`` from the correction, so the jump lands on the edge that is
    allowed to jump. With ``enable_slam`` off, ``map`` -> ``odom`` is identity.

    The pose is the *left camera's*. Publishing it as ``base_frame`` assumes the
    camera is the body origin; on a real robot either set ``base_frame`` to the
    camera's own frame and let the static tree carry it to the body, or feed the
    mount extrinsic in.
    """

    config: CuvslamConfig

    image_left: In[Image]
    image_right: In[Image]
    camera_info: In[CameraInfo]
    camera_info_right: In[CameraInfo]
    imu: In[Imu]

    odometry: Out[Odometry]
    _landmarks: Out[PointCloud2]
    corrected_odometry: Out[Odometry]
    map_tf: Out[Odometry]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odometry, self.odometry))
        )
        self.register_disposable(
            Disposable(self.map_tf.transport.subscribe(self._on_map_tf, self.map_tf))
        )

    def _on_map_tf(self, message: Odometry) -> None:
        """Slam's correction. Replaces the identity map->odom once Slam is running."""
        self.tf.publish(
            TFMessage(self._transform(message, self.config.map_frame, self.config.odom_frame))
        )

    @staticmethod
    def _transform(message: Odometry, frame_id: str, child_frame_id: str) -> Transform:
        return Transform(
            frame_id=frame_id,
            child_frame_id=child_frame_id,
            translation=Vector3(
                message.pose.position.x, message.pose.position.y, message.pose.position.z
            ),
            rotation=Quaternion(
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ),
            ts=message.ts or time.time(),
        )

    def _on_odometry(self, message: Odometry) -> None:
        transforms = [self._transform(message, self.config.odom_frame, self.config.base_frame)]
        # With Slam running, map->odom is its correction and arrives on _on_map_tf.
        # Only one publisher may own that edge.
        if not self.config.enable_slam and self.config.publish_map_to_odom:
            transforms.insert(
                0,
                Transform(
                    frame_id=self.config.map_frame,
                    child_frame_id=self.config.odom_frame,
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    ts=message.ts or time.time(),
                ),
            )
        self.tf.publish(TFMessage(*transforms))
