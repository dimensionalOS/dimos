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
from typing import Any

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


def test_topic_filter_reaches_the_instance(recording: str) -> None:
    """Workers rebuild ports from config, so the filter must travel in the blueprint kwargs."""
    cls = replay_module(recording, topics="odom")
    assert cls.blueprint(dataset=recording).blueprints[0].kwargs["topics"] == "odom"
    module = replay_module("")(dataset=recording, topics="odom")
    assert sorted(module.outputs) == ["odom"]
    module.stop()


def test_stream_named_like_a_module_attribute_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    store.stream("start", PoseStamped).append(PoseStamped(ts=1.0), ts=1.0)
    store.stop()
    with pytest.raises(ValueError, match="'start' clashes"):
        replay_module(str(path))
    with pytest.raises(ValueError, match="'start' clashes"):
        replay_module("")(dataset=str(path))


def test_slow_first_decode_does_not_skip_other_streams(
    recording: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Anchor is pinned after setup; a stream that is slow to prime must not make later streams late."""
    from dimos.memory import replay as replay_mod

    real = replay_mod.ReplayStream._decode

    def slow(self: Any, obs: Any) -> Any:
        if obs.ts == 1.05:  # the goal frame, subscribed first (sorted order)
            time.sleep(0.2)
        return real(self, obs)

    monkeypatch.setattr(replay_mod.ReplayStream, "_decode", slow)
    module = replay_module(recording)(dataset=recording)
    got: dict[str, list[float]] = {"odom": [], "goal": []}
    module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
    module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
    module.start()
    deadline = time.time() + 5
    while time.time() < deadline and (len(got["odom"]) < 3 or not got["goal"]):
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]


def test_recorded_rerun_config_from_run_dir(tmp_path: Path) -> None:
    from dimos.memory.replay_module import recorded_rerun_config

    db = tmp_path / "20260910-135235-unitree-go2" / "memory.db"
    cfg = recorded_rerun_config(str(db))
    assert "world/robot_body" in cfg["static"]
    assert callable(cfg["blueprint"])  # the live layout, so replay looks like the live run
    assert recorded_rerun_config(str(tmp_path / "downloads" / "x.db")) == {}
    assert recorded_rerun_config(str(tmp_path / "20260910-135235-no-such-bp" / "m.db")) == {}


def test_dataset_path_resolves_names_and_skips_the_default(recording: str) -> None:
    from dimos.memory.replay_module import dataset_path

    assert dataset_path(recording) == recording
    assert dataset_path(recording, explicit=False) == recording  # a real file always counts
    assert dataset_path("go2_short", explicit=False) == ""  # registry import: no LFS
    assert dataset_path("no-such-dataset-for-this-test") == ""
