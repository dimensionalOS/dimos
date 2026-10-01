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

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.memory.store.sqlite import SqliteStore


@pytest.fixture
def dataset(tmp_path: Path) -> str:
    """A tiny on-disk memory dataset: 5 odom poses walking 4m in +x over 4s."""
    path = tmp_path / "tiny.db"
    with SqliteStore(path=str(path)) as store:
        stream = store.stream("odom", PoseStamped)
        for i in range(5):
            pose = PoseStamped(
                header=Header(frame_id="world"),
                pose=Pose(position=Point(x=float(i)), orientation=Quaternion(w=1.0)),
            )
            stream.append(pose, ts=1000.0 + i)
    return str(path)
