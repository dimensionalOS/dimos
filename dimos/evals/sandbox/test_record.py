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
import time
import uuid

import pytest

from dimos.core.global_config import global_config
from dimos.evals.sandbox.record import SETTLE_S, blueprint_transports, record
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Twist import Twist

BRIDGE = ["run", "raw-robot-bridge"]


@pytest.fixture
def restored_global_config(monkeypatch: pytest.MonkeyPatch) -> None:
    """The recorder sets the global config as `dimos run` would; put it back afterwards."""
    for field, value in global_config.model_dump().items():
        monkeypatch.setattr(global_config, field, value)
    monkeypatch.setenv("DIMOS_TRANSPORT", "zenoh")
    monkeypatch.setenv("ZENOH_NAMESPACE", f"test/{uuid.uuid4().hex[:8]}")


def test_the_transports_are_the_blueprints_under_its_namespace(
    restored_global_config: None,
) -> None:
    transports = blueprint_transports(BRIDGE)
    assert ("cmd_vel", Twist) in transports
    assert str(transports["cmd_vel", Twist].topic.topic) == "dimos/cmd_vel"
    assert global_config.zenoh_namespace.startswith("test/")  # every session it opens adds it


def test_the_recorder_keeps_what_a_stream_carries_until_stopped(
    restored_global_config: None, tmp_path: Path
) -> None:
    db = tmp_path / "trial-1" / "memory.db"
    stop = threading.Event()
    recorder = threading.Thread(target=record, args=(db, BRIDGE, stop))
    recorder.start()
    try:
        time.sleep(SETTLE_S + 1.0)
        cmd_vel = blueprint_transports(BRIDGE)["cmd_vel", Twist]
        for _ in range(5):
            cmd_vel.publish(Twist(linear=[0.2, 0.0, 0.0]))
            time.sleep(0.1)
    finally:
        stop.set()
        recorder.join(timeout=30)
    assert not recorder.is_alive()
    with SqliteStore(path=str(db)) as store:
        twists = [obs.data for obs in store.stream("cmd_vel", Twist).to_list()]
    assert len(twists) == 5 and twists[0].linear.x == pytest.approx(0.2)
