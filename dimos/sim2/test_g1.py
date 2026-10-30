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

"""Actual G1 URDF, shipped ONNX policy and normal module deployment, not mocks."""

from dataclasses import replace
import time
import uuid

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2 import G1_DESK_WORLD, unitree_g1_sim2
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.sim_device import G1SimDevice
from dimos.sim2.module import SimModule
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.runtime import Runtime
from dimos.sim2.sensor_module import SimSensorModule
from dimos.sim2.types import JointTargets

pytestmark = pytest.mark.mujoco


@pytest.fixture
def walking_robot():
    world = replace(G1_DESK_WORLD, instances=G1_DESK_WORLD.instances[:1])
    backend = MujocoBackend(world)
    runtime = Runtime(backend, backend.layout.drives)
    device = G1SimDevice("g1", runtime.submit)
    device.bind(runtime.backend.layout)
    return runtime, device


def cycle(runtime, device, count, velocity=None):
    minimum_height = 10.0
    for _ in range(count):
        if velocity is not None:
            device.move(velocity)
        device.observe(runtime.capture())
        runtime.advance(10)
        minimum_height = min(minimum_height, runtime.snapshot().poses[0, 2])
    assert minimum_height > 0.68
    assert runtime.rejected == 0
    return runtime.snapshot()


def test_g1_balances_walks_turns_and_resets(walking_robot):
    runtime, device = walking_robot
    initial = runtime.snapshot()
    idle = cycle(runtime, device, 3000)  # a minute, well beyond the previous jitter onset
    assert np.linalg.norm(idle.poses[0, :2] - initial.poses[0, :2]) < 0.3
    before = idle.poses[0, :3].copy()
    heading = Rotation.from_quat(idle.poses[0, 3:].copy()).as_matrix()[:, 0]
    walking = cycle(runtime, device, 250, Twist(linear=Vector3(0.2, 0, 0)))
    assert np.dot(walking.poses[0, :3] - before, heading) > 0.6
    yaw_before = Rotation.from_quat(walking.poses[0, 3:].copy()).as_euler("xyz")[2]
    turned = cycle(runtime, device, 200, Twist(angular=Vector3(0, 0, 0.3)))
    yaw_after = Rotation.from_quat(turned.poses[0, 3:].copy()).as_euler("xyz")[2]
    assert (yaw_after - yaw_before + np.pi) % (2 * np.pi) - np.pi > 0.4
    reset = runtime.reset()
    assert reset.epoch != initial.epoch
    assert reset.poses == pytest.approx(initial.poses)
    cycle(runtime, device, 500)


def test_control_barrier_requires_current_epoch_tick_and_valid_targets(walking_robot):
    runtime, device = walking_robot
    assert not runtime.controlled(device.names)
    runtime.submit(JointTargets(runtime.epoch, device.names, (0.0,) * 29))
    runtime.prepare()
    assert runtime.last_error == "paced joints require a feedback tick"
    device.observe(runtime.capture())
    runtime.prepare()
    assert runtime.controlled(device.names)
    runtime.advance(10)
    assert not runtime.controlled(device.names)
    runtime.submit(JointTargets(runtime.epoch, device.names, (0.0,) * 29, tick=0))
    runtime.prepare()
    assert not runtime.controlled(device.names)
    assert runtime.last_error == "stale control tick"
    runtime.submit(JointTargets(runtime.epoch, device.names, (100.0,) * 29, tick=runtime.tick))
    runtime.prepare()
    assert not runtime.controlled(device.names)
    assert runtime.last_error.startswith("target outside joint limits")
    before = runtime.snapshot()
    for _ in range(10):
        runtime.prepare()  # waiting for inference must not advance or reset physics
    assert runtime.snapshot().poses == pytest.approx(before.poses)
    assert runtime.tick == before.tick
    runtime.reset()
    runtime.submit(JointTargets(before.epoch, device.names, (0.0,) * 29, tick=0))
    runtime.prepare()
    assert runtime.last_error == "stale simulation epoch"


@pytest.fixture
def deployed_g1(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = unitree_g1_sim2.global_config(
        zenoh_domain="g1-test-" + uuid.uuid4().hex, zenoh_scout_addr="224.0.0.226:7446"
    )
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        yield coordinator
    finally:
        coordinator.stop()


def test_g1_blueprint_drives_with_sensors_then_resets(deployed_g1):
    sim = deployed_g1.get_instance(SimModule)
    connection = deployed_g1.get_instance(G1Connection)
    sensors = deployed_g1.get_instance(SimSensorModule)
    initial = sim.state()
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        state = sim.state()
        stats = sensors.statistics()
        assert not stats["error"]
        assert state.poses[0, 2] > 0.68
        if state.sim_time < 3:
            connection.move(Twist(linear=Vector3(0.12, 0, 0)))
        else:
            connection.move(Twist())
        if state.sim_time > 5 and stats["images"] >= 10 and stats["scans"] >= 20:
            break
        time.sleep(0.05)  # poll multiple independent workers with a bounded deadline
    else:
        pytest.fail(
            f"G1 did not progress: {sim.timing()}, sensors={stats}, control={connection.control_status()}"
        )
    assert state.poses[0, 0] - initial.poses[0, 0] > 0.25
    assert not sim.timing()["error"]
    assert sim.timing()["rejected_commands"] == 0
    reset = sim.reset()
    assert reset.epoch != state.epoch
    assert reset.poses[0, :3] == pytest.approx([0, 0, 0.755])
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        state = sim.state()
        if state.epoch == reset.epoch and state.sim_time > 1:
            break
        time.sleep(0.05)
    assert state.epoch == reset.epoch and state.sim_time > 1
    assert state.poses[0, 2] > 0.68
