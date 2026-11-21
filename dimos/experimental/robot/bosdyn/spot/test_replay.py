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

import pytest

from dimos.experimental.robot.bosdyn.spot.replay import SpotReplay
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


@pytest.mark.parametrize(("name", "error"), [("temperature", ValueError), ("odometry", TypeError)])
def test_rejects_unusable_robot_output(tmp_path: Path, name: str, error: type[Exception]) -> None:
    path = tmp_path / "input.db"
    with SqliteStore(path=path) as store:
        store.stream(name, int).append(1, ts=1.0)
        if name == "odometry":
            store.stream("tf", TFMessage).append(TFMessage(), ts=1.0)
    module = SpotReplay(db_path=str(path))
    try:
        with pytest.raises(error, match=name):
            module.start()
        assert module.rpc is not None  # The coordinator still needs the startup error reply.
        assert module._loop_thread is None
    finally:
        module.stop()
