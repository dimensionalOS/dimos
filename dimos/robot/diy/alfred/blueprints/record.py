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
depth + camera infos, Point-LIO odom + lidar (trajectory baked into
``pointlio_lidar`` via the inherited pose setters), the coordinator's aggregate
joint state, FlowBase wheel odometry, and operator ``cmd_vel``. The wrist
fisheye USB cameras are intentionally NOT recorded.

The two RealSense modules are bound to their physical cameras by serial number,
so front/back never depend on USB enumeration order. Lidar IPs come from the
module envs::

    export DIMOS_MID360_LIDAR_IP=192.168.1.189 DIMOS_POINTLIO_LIDAR_IP=192.168.1.189
    dimos run alfred-record --left-can-port can2 --right-can-port can3 \
        --device-path /dev/serial/by-id/<nano> \
        --rerun-host <bot-ip> --rerun-open none --visualization.host <bot-ip>
"""

from __future__ import annotations

from pydantic import Field

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.recorder import (
    PointlioRecorder,
    PointlioRecorderConfig,
)
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.diy.alfred.blueprints.alfred import alfred

# Physical camera identity on the Alfred rig (orin-nx-7837), by USB serial.
FRONT_REALSENSE_SERIAL = "260843060204"  # D455, front-facing
BACK_REALSENSE_SERIAL = "349643060044"  # D435if, back-facing

# Highest resolution both models stream color AND aligned depth at together.
_CAM_WIDTH, _CAM_HEIGHT, _CAM_FPS = 1280, 720, 15

_CAMERA_PORTS = ("color_image", "depth_image", "camera_info", "depth_camera_info")


def _rig_realsense(prefix: str, serial: str) -> object:
    """One serial-bound RealSense with its streams namespaced ``<prefix>_*``."""
    return RealSenseCamera.blueprint(
        instance_name=f"realsense_{prefix}",
        camera_name=f"{prefix}_realsense",
        serial_number=serial,
        width=_CAM_WIDTH,
        height=_CAM_HEIGHT,
        fps=_CAM_FPS,
    ).remappings([(RealSenseCamera, port, f"{prefix}_{port}") for port in _CAMERA_PORTS])


class AlfredRecorderConfig(PointlioRecorderConfig):
    # No static mount frames are published for the cameras yet, and the command/
    # state streams inherently have no pose: record them poseless rather than
    # flooding the log with failed world<-frame tf lookups.
    poseless_streams: list[str] = Field(
        default_factory=lambda: [
            f"{prefix}_{port}" for prefix in ("front", "back") for port in _CAMERA_PORTS
        ]
        + ["coordinator_joint_state", "wheel_odometry", "cmd_vel"]
    )
    # Depth must round-trip losslessly; JPEG would corrupt the 16-bit range.
    stream_codecs: dict[str, str] = Field(
        default_factory=lambda: {
            "front_depth_image": "lz4+lcm",
            "back_depth_image": "lz4+lcm",
        }
    )


class AlfredRecorder(PointlioRecorder):
    """Records the Alfred rig streams; pointlio odom/lidar come from the base."""

    config: AlfredRecorderConfig

    front_color_image: In[Image]
    front_depth_image: In[Image]
    front_camera_info: In[CameraInfo]
    front_depth_camera_info: In[CameraInfo]

    back_color_image: In[Image]
    back_depth_image: In[Image]
    back_camera_info: In[CameraInfo]
    back_depth_camera_info: In[CameraInfo]

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
    PointLio.blueprint(frame_id="world").remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    AlfredRecorder.blueprint(),
).global_config(n_workers=11)
