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

"""Add the Go2's static front-camera CameraInfo (K + distortion, camera_optical
frame) to a recording's .db so AprilTag post-processing can find intrinsics. Adds
the `camera_info` stream only if it's missing.

Usage:
  python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/add_camera_info.py PATH.db
"""

from pathlib import Path
import sys
from typing import Any

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.unitree.go2.connection import GO2Connection

STREAM = "camera_info"


def earliest_ts(store: SqliteStore) -> float:
    """First observation timestamp across existing streams, so the CameraInfo sits
    at the head of the recording timeline (0.0 if the db has no data yet)."""
    timestamps = []
    for stream_name in store.list_streams():
        observation: Any = next(iter(store.stream(stream_name)), None)
        if observation is not None:
            timestamps.append(float(observation.ts))
    return min(timestamps) if timestamps else 0.0


def main() -> None:
    db_path = Path(sys.argv[1]).expanduser()
    store = SqliteStore(path=db_path, must_exist=True)
    store.start()
    try:
        if STREAM in store.list_streams():
            print(f"{STREAM!r} already present in {db_path} — nothing to do")
            return
        camera_info: CameraInfo = GO2Connection.camera_info_static
        store.stream(STREAM, CameraInfo).append(camera_info, ts=earliest_ts(store))
        print(f"added {STREAM!r} (frame {camera_info.frame_id!r}) to {db_path}")
    finally:
        store.stop()


if __name__ == "__main__":
    main()
