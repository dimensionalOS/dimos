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

from collections.abc import Iterator

import pytest

pytest.importorskip("tars_sdk")

from dimos.hardware.drive_trains.registry import twist_base_adapter_registry
from dimos.hardware.drive_trains.spec import TwistBaseAdapter
from dimos.hardware.drive_trains.tars.adapter import TarsTwistAdapter

pytestmark = pytest.mark.mujoco


@pytest.fixture
def adapter() -> Iterator[TarsTwistAdapter]:
    a = TarsTwistAdapter(realtime=False, cmd_timeout=1.0)
    assert a.connect()
    yield a
    a.disconnect()


def _drive(a: TarsTwistAdapter, cmd: list[float], seconds: float) -> None:
    for _ in range(round(seconds / 0.1)):
        a.write_velocities(cmd)
        a.client.step(0.1)


def test_registry_creates_tars_adapter() -> None:
    a = twist_base_adapter_registry.create("tars", dof=3, realtime=False)
    assert isinstance(a, TarsTwistAdapter)
    assert isinstance(a, TwistBaseAdapter)


def test_commands_are_refused_until_enabled(adapter: TarsTwistAdapter) -> None:
    assert not adapter.write_velocities([0.1, 0.0, 0.0])
    assert adapter.write_enable(True)
    assert adapter.wait_until_ready()
    assert adapter.write_velocities([0.1, 0.0, 0.0])


def test_walks_forward_and_odometry_tracks_ground_truth(adapter: TarsTwistAdapter) -> None:
    adapter.write_enable(True)
    adapter.wait_until_ready()
    _drive(adapter, [0.15, 0.0, 0.0], seconds=15.0)

    x, y, _ = adapter.read_odometry()
    gt = adapter.client.get_odometry(ground_truth=True)
    assert x > 1.0, "TARS should have walked at least 1 m forward"
    assert abs(x - gt.x) < 0.1
    assert abs(y) < 0.1


def test_turns_in_place(adapter: TarsTwistAdapter) -> None:
    adapter.write_enable(True)
    adapter.wait_until_ready()
    _drive(adapter, [0.0, 0.0, 0.2], seconds=8.0)

    x, y, yaw = adapter.read_odometry()
    assert yaw > 1.0
    assert abs(x) < 0.1 and abs(y) < 0.1


def test_disable_sits_down(adapter: TarsTwistAdapter) -> None:
    adapter.write_enable(True)
    adapter.wait_until_ready()
    adapter.write_enable(False)
    assert not adapter.read_enabled()
    assert adapter.client.wait_for_mode("sit", timeout=10.0)
