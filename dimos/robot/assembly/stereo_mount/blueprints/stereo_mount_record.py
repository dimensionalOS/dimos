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

"""Record blueprint for the stereo_mount rig (SDK-free ZED + Mid-360 via Point-LIO).

Both ZED eyes (60 fps color, no ZED SDK) and Point-LIO odom+lidar are recorded into
a memory2 db. Point-LIO publishes the moving ``world -> lidar_link`` edge onto tf
and the rig's urdf mount frames are republished continuously, so every recorded
stream resolves against ``world``. The lidar IP comes from Point-LIO's own config
(``DIMOS_POINTLIO_LIDAR_IP``)::

    export DIMOS_POINTLIO_LIDAR_IP=192.168.1.107
    dimos run stereo-mount-record
"""

from datetime import datetime
from pathlib import Path

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.sensors.camera.zed.uvc import ZedUvcCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.robot.assembly.stereo_mount.assembly import StereoMountStaticTf
from dimos.robot.assembly.stereo_mount.record import StereoMountRecorder


def _default_recording_dir() -> Path:
    # Local time, with the machine's actual zone abbreviation (not a hardcoded PST).
    now = datetime.now().astimezone()
    stamp = (
        now.strftime("%Y-%m-%d") + "_" + now.strftime("%I-%M%p").lower() + "-" + now.strftime("%Z")
    )
    return RECORDINGS_DIR / stamp


_RECORDING_DIR = _default_recording_dir()


stereo_mount_record = autoconnect(
    ZedUvcCamera.blueprint(),
    # world -> lidar_link is the moving odometry edge; lidar_link is the mid360
    # point-cloud origin in stereo_mount.urdf, tying odometry into the rig tree.
    PointLio.blueprint(frame_id="world", sensor_frame_id="lidar_link").remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    StereoMountRecorder.blueprint(db_path=str(_RECORDING_DIR / "mem2.db")),
    # Continuously republishes the rig's urdf mount frames onto tf (no latched static tf).
    StereoMountStaticTf.blueprint(),
).global_config(n_workers=4)


if __name__ == "__main__":
    _RECORDING_DIR.mkdir(parents=True, exist_ok=True)
    coordinator = ModuleCoordinator.build(stereo_mount_record)
    coordinator.loop()
