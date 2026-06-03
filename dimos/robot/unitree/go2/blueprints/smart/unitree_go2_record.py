#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

import math
import os
from pathlib import Path
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.fastlio2 import module as _fastlio2_module
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.fastlio2.recorder import FastLio2Recorder, _default_recording_dir
from dimos.hardware.sensors.lidar.fastlio2.speed_warner import SpeedWarner
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.logging_config import set_run_log_dir, setup_logger

logger = setup_logger()

# mid360_link is physically measured relative to camera (easier than measuring to base_link):
# relative to camera the lidar is 3.2cm back, 12cm up, pitched 44deg down.
# base_link -> front_camera -> mid360_link
BASE_TO_FRONT_CAMERA = Transform(
    translation=Vector3(0.32715, -0.00003, 0.04297),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="base_link",
    child_frame_id="front_camera",
)

_MID360_PITCH_HALF = math.radians(44.0) / 2.0
BASE_TO_MID360 = BASE_TO_FRONT_CAMERA + Transform(
    translation=Vector3(-0.032, 0.0, 0.12),
    rotation=Quaternion(0.0, math.sin(_MID360_PITCH_HALF), 0.0, math.cos(_MID360_PITCH_HALF)),
    frame_id="front_camera",
    child_frame_id="mid360_link",
)
MID360_TO_BASE = BASE_TO_MID360.inverse()

# base_link -> camera_optical using the URDF front_camera mount plus the
# standard ROS optical-frame rotation (x-right, y-down, z-forward).
BASE_TO_CAMERA_OPTICAL = BASE_TO_FRONT_CAMERA + Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="front_camera",
    child_frame_id="camera_optical",
)


_LIDAR_IP = os.getenv("LIDAR_IP", "192.168.1.107")


class Go2TfHackRecorder(FastLio2Recorder):
    """Records with statically-applied transforms instead of querying tf.

    FastLio2 tracks the Mid-360 (``mid360_link``) and reports its pose in the
    ``world`` frame as ``fastlio_odometry``; its registered cloud is likewise
    already in that world frame. We anchor recorded observations to the robot
    body, building every pose from the latest fastlio odom and fixed mounts:

    - ``fastlio_lidar`` -> ``base_link`` pose in world (odom, then mid360_link -> base_link)
    - ``color_image``   -> ``camera_optical`` pose in world (odom, mid360_link -> base_link,
      then base_link -> camera_optical)
    - everything else (odom streams included) -> no pose
    """

    fastlio_lidar: In[PointCloud2]
    fastlio_odometry: In[Odometry]
    go2_lidar: In[PointCloud2]
    go2_odom: In[PoseStamped]
    color_image: In[Image]
    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]
    # Shadow the parent's generic companion ports so they're not recorded as
    # empty `lidar`/`odom` streams; the go2-prefixed ports above take their place.
    lidar: None = None  # type: ignore[assignment]
    odom: None = None  # type: ignore[assignment]
    # sanity check
    fastlio_lidar_no_cap: In[PointCloud2]
    fastlio_odometry_no_cap: In[Odometry]

    _latest_fastlio_odom: Odometry | None = None
    _warning_names: set[str] = set()

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        def on_msg(msg: Any) -> None:
            ts = time.time()
            pose = None
            if name == "fastlio_odometry":
                self._latest_fastlio_odom = msg
            elif name == "fastlio_lidar":
                world_to_base = self._world_to_base_from_fastlio()
                if world_to_base is not None:
                    pose = world_to_base.to_pose()
            elif name == "color_image":
                # anchor images to world frame as defined by fastlio odom
                world_to_base = self._world_to_base_from_fastlio()
                if world_to_base is not None:
                    pose = (world_to_base + BASE_TO_CAMERA_OPTICAL).to_pose()
            elif "odom" in name:
                pass
            else:
                if name not in self._warning_names:
                    self._warning_names.add(name)
                    logger.warning(f"cannot compute pose for {name}; recording without pose")

            stream.append(msg, ts=ts, pose=pose)

        self.register_disposable(Disposable(input_topic.subscribe(on_msg)))

    def _world_to_base_from_fastlio(self) -> Transform | None:
        odom = self._latest_fastlio_odom
        if odom is None:
            return None
        world_to_mid360 = Transform(
            translation=odom.position,
            rotation=odom.orientation,
            frame_id="world",
            child_frame_id="mid360_link",
            ts=odom.ts,
        )
        return world_to_mid360 + MID360_TO_BASE


class FastLio2NoCap(FastLio2):
    pass


# Absolute path to FastLio2's cpp build dir; passed to FastLio2NoCap so the
# trivial subclass doesn't try to resolve `cpp` next to this file.
_FASTLIO2_CPP = str(Path(_fastlio2_module.__file__).resolve().parent / "cpp")

unitree_go2_record = autoconnect(
    KeyboardTeleop.blueprint(),
    MovementManager.blueprint(),
    GO2Connection.blueprint().remappings(
        [
            (GO2Connection, "lidar", "go2_lidar"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    ),
    Mid360.blueprint(
        lidar_ip=_LIDAR_IP,
    ).remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    FastLio2.blueprint(
        frame_id="world",
        map_freq=-1,
        lidar_ip=_LIDAR_IP,
        max_velocity_norm_ms=3.1,  # meters/sec, 3.1 => 7mph, 5=>12mph. We want some padding
    ).remappings(
        [
            (FastLio2, "lidar", "fastlio_lidar"),
            (FastLio2, "odometry", "fastlio_odometry"),
        ]
    ),
    FastLio2NoCap.blueprint(
        frame_id="world",
        map_freq=-1,
        lidar_ip=_LIDAR_IP,
        max_velocity_norm_ms=100,
        cwd=_FASTLIO2_CPP,
    ).remappings(
        [
            (FastLio2, "lidar", "fastlio_lidar_no_cap"),
            (FastLio2, "odometry", "fastlio_odometry_no_cap"),
        ]
    ),
    Go2TfHackRecorder.blueprint(lidar_ip=_LIDAR_IP, record_pcap=True),
    SpeedWarner.blueprint().remappings(
        [
            (SpeedWarner, "odometry", "fastlio_odometry_no_cap"),
        ]
    ),
).global_config(n_workers=10, robot_model="unitree_go2")


if __name__ == "__main__":
    recording_dir = _default_recording_dir().resolve()
    recording_dir.mkdir(parents=True, exist_ok=True)
    set_run_log_dir(recording_dir)
    global_config.obstacle_avoidance = False
    coordinator = ModuleCoordinator.build(
        unitree_go2_record,
        {Go2TfHackRecorder.name: {"recording_dir": recording_dir}},
    )
    coordinator.loop()
