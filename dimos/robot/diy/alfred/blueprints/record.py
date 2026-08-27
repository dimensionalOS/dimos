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

"""Alfred office-teleop recording: the full ``alfred`` stack plus both RealSense
cameras, the Mid-360 lidar, and a memory recorder.

Records into a memory SQLite db: front (D455) and back (D435if) color + aligned
depth + camera infos, the raw Mid-360 lidar + IMU streams (Point-LIO is not run
online — this worktree has no native binary; the raw streams support offline
LIO/mapping afterwards), the coordinator's aggregate joint state, FlowBase wheel
odometry, and operator ``cmd_vel``. The wrist fisheye USB cameras are
intentionally NOT recorded.

The two RealSense modules are bound to their physical cameras by serial number,
so front/back never depend on USB enumeration order. The lidar IP comes from the
module env::

    export DIMOS_MID360_LIDAR_IP=192.168.1.189
    dimos run alfred-record --left-can-port can2 --right-can-port can3 \
        --device-path /dev/serial/by-id/<nano> \
        --rerun-host <bot-ip> --rerun-open none --visualization.host <bot-ip>
"""

from __future__ import annotations

import fcntl
import time

from pydantic import Field

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig, RustRecorder
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.memory.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.diy.alfred.blueprints.alfred import alfred

# Physical camera identity on the Alfred rig (orin-nx-7837), by USB serial.
FRONT_REALSENSE_SERIAL = "260922302422"  # D455, front-facing (librealsense serial)
BACK_REALSENSE_SERIAL = "327122071721"  # D435if, back-facing (librealsense serial)

# Highest resolution both models stream color AND aligned depth at together.
_CAM_WIDTH, _CAM_HEIGHT, _CAM_FPS = 1280, 720, 15

_CAMERA_PORTS = (
    "color_image",
    "depth_image",
    "camera_info",
    "depth_camera_info",
    "infrared_left",
    "infrared_right",
    "infrared_left_camera_info",
    "infrared_right_camera_info",
)


class SerializedStartRealSense(RealSenseCamera):
    """RealSense whose pipeline open is serialized across processes.

    Two processes calling rs.pipeline.start() concurrently deadlock
    librealsense on this platform (the second start blocks forever, observed
    via py-spy at camera.py start). A shared flock makes the opens take turns.
    """

    _START_LOCK = "/tmp/dimos_realsense_start.lock"

    @rpc
    def start(self) -> None:
        with open(self._START_LOCK, "w") as lockfile:
            fcntl.flock(lockfile, fcntl.LOCK_EX)
            try:
                super().start()
                # Let the pipeline settle before the peer starts its open.
                time.sleep(1.0)
            finally:
                fcntl.flock(lockfile, fcntl.LOCK_UN)


def _rig_realsense(prefix: str, serial: str) -> object:
    """One serial-bound RealSense with its streams namespaced ``<prefix>_*``."""
    return SerializedStartRealSense.blueprint(
        instance_name=f"realsense_{prefix}",
        camera_name=f"{prefix}_realsense",
        serial_number=serial,
        width=_CAM_WIDTH,
        height=_CAM_HEIGHT,
        fps=_CAM_FPS,
    ).remappings([(SerializedStartRealSense, port, f"{prefix}_{port}") for port in _CAMERA_PORTS])


class AlfredRecorderConfig(RecorderConfig):
    # No static mount frames are published for the cameras or lidar yet, and the
    # command/state streams inherently have no pose: record them poseless rather
    # than flooding the log with failed world<-frame tf lookups.
    poseless_streams: list[str] = Field(
        default_factory=lambda: [
            f"{prefix}_{port}" for prefix in ("front", "back") for port in _CAMERA_PORTS
        ]
        + ["livox_lidar", "livox_imu", "coordinator_joint_state", "wheel_odometry", "cmd_vel"]
    )
    # Depth must round-trip losslessly; JPEG would corrupt the 16-bit range.
    stream_codecs: dict[str, str] = Field(
        default_factory=lambda: {
            "front_depth_image": "lz4+lcm",
            "back_depth_image": "lz4+lcm",
        }
    )


class AlfredRecorder(Recorder):
    """Records the Alfred office-teleop rig streams into a memory db."""

    config: AlfredRecorderConfig

    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]

    front_color_image: In[Image]
    front_depth_image: In[Image]
    front_camera_info: In[CameraInfo]
    front_depth_camera_info: In[CameraInfo]

    back_color_image: In[Image]
    back_depth_image: In[Image]
    back_camera_info: In[CameraInfo]
    back_depth_camera_info: In[CameraInfo]

    front_infrared_left: In[Image]
    front_infrared_right: In[Image]
    front_infrared_left_camera_info: In[CameraInfo]
    front_infrared_right_camera_info: In[CameraInfo]
    back_infrared_left: In[Image]
    back_infrared_right: In[Image]
    back_infrared_left_camera_info: In[CameraInfo]
    back_infrared_right_camera_info: In[CameraInfo]

    coordinator_joint_state: In[JointState]
    wheel_odometry: In[Odometry]
    cmd_vel: In[Twist]


alfred_record = autoconnect(
    alfred,
    _rig_realsense("front", FRONT_REALSENSE_SERIAL),
    _rig_realsense("back", BACK_REALSENSE_SERIAL),
    Mid360.blueprint().remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    AlfredRecorder.blueprint(),
).global_config(n_workers=10)


class AlfredRustRecorder(RustRecorder):
    """Native (Rust) recorder variant: same streams, mcap artifact.

    High-throughput path for the full color+depth+IR configuration; see
    dimos/experimental/memory/README.md (cc's PR #3615).
    """

    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]

    front_color_image: In[Image]
    front_depth_image: In[Image]
    front_camera_info: In[CameraInfo]
    front_depth_camera_info: In[CameraInfo]

    back_color_image: In[Image]
    back_depth_image: In[Image]
    back_camera_info: In[CameraInfo]
    back_depth_camera_info: In[CameraInfo]

    front_infrared_left: In[Image]
    front_infrared_right: In[Image]
    front_infrared_left_camera_info: In[CameraInfo]
    front_infrared_right_camera_info: In[CameraInfo]
    back_infrared_left: In[Image]
    back_infrared_right: In[Image]
    back_infrared_left_camera_info: In[CameraInfo]
    back_infrared_right_camera_info: In[CameraInfo]

    coordinator_joint_state: In[JointState]
    wheel_odometry: In[Odometry]
    cmd_vel: In[Twist]
    tf: In[TFMessage]


alfred_record_mcap = autoconnect(
    alfred,
    _rig_realsense("front", FRONT_REALSENSE_SERIAL),
    _rig_realsense("back", BACK_REALSENSE_SERIAL),
    Mid360.blueprint().remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    AlfredRustRecorder.blueprint(
        store=RustMcapStoreConfig(path="alfred_record.mcap"),
        encoding_threads=4,
        stream_codecs={
            "front_depth_image": "lz4+lcm",
            "back_depth_image": "lz4+lcm",
            # IR must be lossless: JPEG artifacts corrupt feature tracking.
            "front_infrared_left": "lz4+lcm",
            "front_infrared_right": "lz4+lcm",
            "back_infrared_left": "lz4+lcm",
            "back_infrared_right": "lz4+lcm",
        },
    ),
).global_config(n_workers=10)
