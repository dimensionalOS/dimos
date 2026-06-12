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

import os
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.fastlio2.recorder import FastLio2Recorder, _default_recording_dir
from dimos.hardware.sensors.lidar.fastlio2.speed_warner import SpeedWarner
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.mapping.recording.go2_mid360.static_transforms import (
    BASE_TO_CAMERA_OPTICAL,
    MID360_TO_BASE,
)
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop_tui import KeyboardTeleopTUI
from dimos.utils.logging_config import set_run_log_dir, setup_logger

logger = setup_logger()

_LIDAR_IP = os.getenv("LIDAR_IP", "192.168.1.171")
_LIDAR_HOST_IP = os.getenv("LIDAR_HOST_IP", "192.168.1.100")


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
    lidar: In[PointCloud2]  # Go2 onboard L1 lidar
    odom: In[PoseStamped]  # Go2 onboard leg odometry
    color_image: In[Image]
    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]
    # Shadow the parent's generic companion ports so they're not recorded as
    # empty `lidar`/`odom` streams; the go2-prefixed ports above take their place.
    lidar: None = None  # type: ignore[assignment]
    odom: None = None  # type: ignore[assignment]
    tf: In[Transform]

    _latest_fastlio_odom: Odometry | None = None
    _warning_names: set[str] = set()

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        def on_msg(msg: Any) -> None:
            ts = time.time()
            pose = None
            if name == "fastlio_odometry" or name == "fastlio_odometry_no_cap":
                self._latest_fastlio_odom = msg
                world_to_base = self._world_to_base_from_fastlio()
                if world_to_base is not None:
                    pose = world_to_base.to_pose()
            elif name == "fastlio_lidar" or name == "fastlio_lidar_no_cap":
                world_to_base = self._world_to_base_from_fastlio()
                if world_to_base is not None:
                    pose = world_to_base.to_pose()
            elif name in ("color_image", "go2_color_image"):
                # anchor images to world frame as defined by fastlio odom
                world_to_base = self._world_to_base_from_fastlio()
                if world_to_base is not None:
                    pose = (world_to_base + BASE_TO_CAMERA_OPTICAL).to_pose()
            elif name == "odom":
                pose = msg
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


unitree_go2_record = autoconnect(
    MovementManager.blueprint(),
    GO2Connection.blueprint(),
    Mid360.blueprint(
        lidar_ip=_LIDAR_IP,
        host_ip=_LIDAR_HOST_IP,
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
    Go2TfHackRecorder.blueprint(lidar_ip=_LIDAR_IP, record_pcap=True),
    SpeedWarner.blueprint().remappings(
        [
            (SpeedWarner, "odometry", "fastlio_odometry"),
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

    # Sit/stand drive the Go2 directly via its RPCs. After standing the dog must
    # re-enter BalanceStand before it will walk again, so on_stand chains
    # standup -> balance_stand (mirrors GO2Connection.start()).
    go2 = coordinator.get_instance(GO2Connection)

    def stand_and_ready() -> None:
        go2.standup()
        time.sleep(3.0)
        go2.balance_stand()

    # Launch the TUI teleop in THIS process (not via the coordinator) so it owns
    # the terminal's stdin and can read WASD keypresses. Its output is wired onto
    # the same /tele_cmd_vel topic MovementManager subscribes to.
    teleop = KeyboardTeleopTUI(
        linear_speed=0.3,
        angular_speed=0.6,
        on_sit=go2.liedown,
        on_stand=stand_and_ready,
    )
    teleop.tele_cmd_vel.transport = LCMTransport("/tele_cmd_vel", Twist)
    teleop.start()
    try:
        coordinator.loop()
    finally:
        teleop.stop()
