#!/usr/bin/env python3
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

"""Drive-and-record blueprint for the Go2 + Mid-360 rig.

Pygame WASD teleop drives the dog while Point-LIO odom+lidar, the Go2's lidar/odom,
and the front camera are recorded into a memory2 db. The Go2/Mid-360 mount frames are
published continuously onto tf so they're captured in the recording. Raw Livox capture
is opt-in: set ``RECORD_PCAP=1`` to also record a .pcap of the Mid-360 UDP stream.

The lidar IPs come from each module's own config (``DIMOS_MID360_LIDAR_IP`` for the
Mid-360 / pcap capture, ``DIMOS_POINTLIO_LIDAR_IP`` for Point-LIO). Run it for a
timestamped ``recordings/`` folder::

    export DIMOS_MID360_LIDAR_IP=192.168.1.171 DIMOS_POINTLIO_LIDAR_IP=192.168.1.171
    uv run python dimos/robot/unitree/go2/blueprints/basic/unitree_go2_mid360_record.py
"""

from datetime import datetime
import os
from pathlib import Path

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.virtual_mid360.recorder import Mid360PcapRecorder
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.go2_mid360_recorder import Go2Mid360Recorder
from dimos.robot.unitree.go2.go2_mid360_static_transforms import Go2Mid360StaticTf
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Opt-in raw-Livox pcap capture (default off). Set RECORD_PCAP=1 to include it.
_RECORD_PCAP = os.getenv("RECORD_PCAP", "").lower() in ("1", "true", "yes", "on")

_TELEOP_LINEAR_SPEED = 0.3
_TELEOP_ANGULAR_SPEED = 0.6


def _default_recording_dir() -> Path:
    # Local time, with the machine's actual zone abbreviation (not a hardcoded PST).
    now = datetime.now().astimezone()
    stamp = (
        now.strftime("%Y-%m-%d") + "_" + now.strftime("%I-%M%p").lower() + "-" + now.strftime("%Z")
    )
    return RECORDINGS_DIR / stamp


_RECORDING_DIR = _default_recording_dir()


unitree_go2_mid360_record = autoconnect(
    MovementManager.blueprint(),
    GO2Connection.blueprint().remappings(
        [
            (GO2Connection, "lidar", "go2_lidar"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    ),
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
    Go2Mid360Recorder.blueprint(db_path=str(_RECORDING_DIR / "mem2.db")),
    # Continuously republishes the rig's mount frames onto tf (no latched static tf).
    Go2Mid360StaticTf.blueprint(),
    # Pygame keyboard teleop (WASD drive + Q/E strafe). Its cmd_vel feeds
    # MovementManager's tele_cmd_vel.
    KeyboardTeleop.blueprint(
        linear_speed=_TELEOP_LINEAR_SPEED, angular_speed=_TELEOP_ANGULAR_SPEED
    ).remappings(
        [
            (KeyboardTeleop, "cmd_vel", "tele_cmd_vel"),
        ]
    ),
).global_config(n_workers=12, robot_model="unitree_go2")

if _RECORD_PCAP:
    unitree_go2_mid360_record = autoconnect(
        unitree_go2_mid360_record,
        Mid360PcapRecorder.blueprint(pcap_path=str(_RECORDING_DIR / "mid360.pcap")),
    )


if __name__ == "__main__":
    _RECORDING_DIR.mkdir(parents=True, exist_ok=True)
    global_config.obstacle_avoidance = False
    coordinator = ModuleCoordinator.build(unitree_go2_mid360_record)
    coordinator.loop()
