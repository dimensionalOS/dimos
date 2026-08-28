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
import time

import pytest

from dimos.core.stream import Out
from dimos.memory.replay_module import replay_module
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


@pytest.fixture
def recording(tmp_path: Path) -> str:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    odom = store.stream("odom", PoseStamped)
    goal = store.stream("goal", PoseStamped)
    for ts in (1.0, 1.1, 1.2):
        odom.append(PoseStamped(ts=ts), ts=ts)
    goal.append(PoseStamped(ts=1.05), ts=1.05)
    store.stop()
    return str(path)


def test_ports_come_from_the_recording(recording: str) -> None:
    cls = replay_module(recording)
    assert cls.__annotations__ == {"goal": Out[PoseStamped], "odom": Out[PoseStamped]}
    assert replay_module(recording, topics="odom").__annotations__ == {"odom": Out[PoseStamped]}
    assert cls.stream_types == {"goal": PoseStamped, "odom": PoseStamped}
    assert replay_module("").__annotations__ == {}
    with pytest.raises(ValueError, match="matched none of: goal, odom"):
        replay_module(recording, topics="nope")


def test_module_publishes_recorded_messages(recording: str) -> None:
    cls = replay_module(recording)
    module = cls(dataset=recording)
    got: dict[str, list[float]] = {"odom": [], "goal": []}
    module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
    module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
    module.start()
    deadline = time.time() + 5
    while time.time() < deadline and len(got["odom"]) < 3:
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]
    assert [round(t, 3) for t in got["goal"]] == [1.05]


def test_portless_class_gets_ports_from_dataset(recording: str) -> None:
    """A worker imports the class without a recording; the instance still has the ports."""
    module = replay_module("")(dataset=recording)
    assert sorted(module.outputs) == ["goal", "odom"]
    module.stop()
