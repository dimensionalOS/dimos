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

from pydantic import Field
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

# The recordings carry right P[3] == 0, so the baseline cannot be read off
# CameraInfo; this is the D455 factory extrinsic.
D455_FACTORY_BASELINE_M = 0.09486231207847595

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


class CuvslamConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/cuvslam_odometry"
    build_command: str | None = f"nix build '{_REPO_ROOT}?dir=dimos/mapping/cuvslam'"
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=_driver_env)

    baseline_m: float = D455_FACTORY_BASELINE_M
    rectified: bool = True
    publish_landmarks: bool = True
    # cuVSLAM's own default. Measured better than synchronous SBA on the airbnb
    # replay (windowed ATE 0.73 m vs 0.94 m), so there is no reason to override it.
    async_sba: bool = True
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
    # Inertial mode. NVIDIA's mode table: "1 stereo pair + 1 IMU ... adds
    # robustness to brief visual failures", which is what a world-frame restart
    # is. Extrinsic is Kalibr's T_cam0_imu from calib_d455_imucam.yaml; the rig
    # frame is the left camera, so that transform is rig_from_imu directly.
    enable_imu: bool = True
    imu_tx: float = 0.030829037
    imu_ty: float = -0.004349224
    imu_tz: float = -0.015419659
    imu_qx: float = 0.00119659
    imu_qy: float = -0.003184443
    imu_qz: float = -0.003371026
    imu_qw: float = 0.999988532
    gyro_noise_density: float = 6.07e-3
    gyro_random_walk: float = 3.6e-5
    accel_noise_density: float = 3.36e-2
    accel_random_walk: float = 9.8e-4
    imu_frequency: float = 400.0

    # Off because masking the Mid-360's dots measurably *hurts*: over three full
    # airbnb runs each way it left ATE unchanged inside the run-to-run spread and
    # raised world-frame restarts. A controlled probe (none / mask / inverted
    # mask) confirmed the masks do reach the tracker, so this is a real result,
    # not a no-op: the dots are apparently usable texture on blank indoor walls,
    # and their count at a restart is a proxy for "pointed at a featureless
    # surface" rather than the cause.
    mask_speckle: bool = False
    speckle_threshold: int = 6
    speckle_grow: int = 2


class CuvslamOdometry(NativeModule):
    """Stereo visual odometry on the GPU.

    ``odometry`` is one continuous ``odom`` -> ``base_link`` path: cuVSLAM restarts
    its world frame after a tracking loss and the module rebases each restart onto
    the last published pose, so the stream never jumps. A restart costs the motion
    that happened across it, and is reported only on the log.

    ``landmarks`` are the 3D points cuVSLAM is tracking this frame, carried into
    the ``odom`` frame. They are a live view rather than an accumulated map, so
    they carry the odometry's drift; put them through ``map`` -> ``odom`` to get
    the corrected positions.

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
    imu: In[Imu]

    odometry: Out[Odometry]
    landmarks: Out[PointCloud2]
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
