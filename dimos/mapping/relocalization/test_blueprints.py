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

from pathlib import Path
import threading

import numpy as np
import pytest

from dimos.mapping.relocalization.blueprints import RecordingPlayer
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


@pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning")
def test_recording_player_stops_mid_replay(tmp_path: Path) -> None:
    path = tmp_path / "recording.db"
    with SqliteStore(path=str(path)) as store:
        lidar = store.stream("pointlio_lidar", PointCloud2)
        tf = store.stream("tf", TFMessage)
        for i in range(3):
            ts = 1.0 + 10.0 * i
            lidar.append(PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32)), ts=ts)
            tf.append(TFMessage(), ts=ts)

    player = RecordingPlayer(dataset=str(path))
    started = threading.Event()
    player.lidar.subscribe(lambda _: started.set())
    player.start()
    try:
        assert started.wait(5)
    finally:
        # Both streams still hold an open cursor here.
        player.stop()
