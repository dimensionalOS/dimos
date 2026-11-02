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

import uuid

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.sim2.module import SimModule
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.sensor_module import SimSensorModule
from dimos.sim2.sensors import MujocoSensors
from dimos.sim2.types import Wrench
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.native_e2e


@pytest.fixture
def deployed_world(world, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = autoconnect(
        SimModule.blueprint(world=world, backend=MujocoBackend),
        SimSensorModule.blueprint(
            world=world,
            reader=MujocoSensors,
            camera="left/camera",
            lidar="left/lidar",
            imu="left/imu",
        ),
    ).global_config(
        viewer="none",
        transport="zenoh",
        n_workers=2,
        zenoh_domain="sim2-test-" + uuid.uuid4().hex,
        zenoh_scout_addr="224.0.0.227:7446",
    )
    deployment = ModuleCoordinator.build(blueprint)
    try:
        yield deployment
    finally:
        deployment.stop()


def test_world_steps_with_sensors_accepts_force_and_resets(deployed_world):
    sim = deployed_world.get_instance(SimModule)
    sensors = deployed_world.get_instance(SimSensorModule)
    wait_until(lambda: sim.state().sim_time > 0.2, timeout=10)
    wait_until(lambda: sensors.statistics()["images"] >= 2, timeout=10)
    wait_until(lambda: sensors.statistics()["scans"] >= 2, timeout=10)

    initial = sim.state()
    box = sim.describe().bodies.index("box")
    sim.apply_wrench(Wrench(initial.epoch, "box", (5.0, 0.0, 0.0), duration=0.2))
    wait_until(lambda: sim.state().poses[box, 0] > initial.poses[box, 0] + 0.1, timeout=5)
    assert sim.state().poses[box, 0] > initial.poses[box, 0] + 0.1

    reset = sim.reset()
    assert reset.epoch != initial.epoch
    assert reset.poses[box, :3] == pytest.approx([0, 0, 0.055])
    images_before = sensors.statistics()["images"]
    wait_until(lambda: sim.state().epoch == reset.epoch and sim.state().sim_time > 0.2, timeout=5)
    wait_until(lambda: sensors.statistics()["images"] > images_before, timeout=5)
    assert sim.state().epoch == reset.epoch
    assert sim.timing()["error"] == ""
    assert sim.timing()["rejected_commands"] == 0
    assert sensors.statistics()["error"] == ""
