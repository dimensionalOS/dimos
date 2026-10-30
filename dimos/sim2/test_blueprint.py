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

"""Real workers, Zenoh, native physics/rendering and URDF TF; separate discovery bus."""

import time
import uuid

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.sim2_lab.blueprints import sim2_lab
from dimos.sim2.module import SimModule
from dimos.sim2.sensor_module import SimSensorModule

pytestmark = pytest.mark.native_e2e


@pytest.fixture
def lab(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = sim2_lab.global_config(
        zenoh_domain="sim2-test-" + uuid.uuid4().hex, zenoh_scout_addr="224.0.0.226:7446"
    )
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        yield coordinator
    finally:
        coordinator.stop()


def test_blueprint_publishes_sensors_moves_then_resets(lab):
    sim = lab.get_instance(SimModule)
    sensors = lab.get_instance(SimSensorModule)
    left = lab.get_instance("left")
    start = time.monotonic()
    deadline = start + 12
    stats = {}
    while time.monotonic() < deadline:
        elapsed = time.monotonic() - start
        left.move(min(0.65, elapsed * 0.18), 0.0)
        stats = sensors.statistics()
        assert not stats["error"]
        state = sim.state()
        if elapsed > 4 and stats["images"] >= 8 and stats["scans"] >= 15:
            break
        time.sleep(0.05)  # bounded polling for motion plus sensor evidence
    else:
        pytest.fail(f"sensor/physics deadline: {stats}")
    layout = sim.describe()
    assert state.poses[layout.bodies.index("box"), 0] > 0.15
    assert state.position[2:4] == pytest.approx([0.0, 0.0], abs=1e-6)
    assert not sim.timing()["error"]
    reset = sim.reset()
    assert reset.epoch != state.epoch
    assert reset.position[0] == pytest.approx(0.0)
