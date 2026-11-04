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

import threading

import pytest

from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.physics.simulation import ControllerConfig, Simulation, SimulationConfig
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def controlled(world):
    config = SimulationConfig(
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        joint_ids=("left/slide_x",),
        motion_frames=("left/imu",),
        startup_timeout_s=0.2,
        control_timeout_s=0.1,
    )
    sim = Simulation(MujocoBackend(), world, config)
    thread = threading.Thread(target=sim.run)
    thread.start()
    sim.ready.result(timeout=10)
    yield sim
    sim.request_stop()
    thread.join(timeout=5)
    assert not thread.is_alive()


def test_missing_controller_pauses_without_stepping_or_resetting(controlled):
    wait_until(lambda: controlled.status()["status"] == "fault", timeout=3)
    assert controlled.time_ns == 0
    assert controlled.epoch == 0
    assert "deadline" in controlled.status()["error"]


def test_control_uses_exact_boundary_then_waits_for_next(controlled, command):
    controlled.submit(command(controlled))
    wait_until(lambda: controlled.status()["time_ns"] == 20_000_000, timeout=3)
    assert controlled.time_ns == 20_000_000
    assert controlled.epoch == 0


def test_nondivisible_export_rate_has_no_rounding_drift(world):
    config = SimulationConfig(pose_hz=200)
    sim = Simulation(MujocoBackend(), world, config)
    sim.load()
    try:
        stamps = []
        for _ in range(11):
            sim.publish_truth()
            while not sim.body_poses.empty():
                stamps.append(sim.body_poses.get_nowait().header.time_ns)
            sim.advance()
        assert stamps == [0, 6_000_000, 10_000_000, 16_000_000, 20_000_000]
    finally:
        sim.backend.close()
