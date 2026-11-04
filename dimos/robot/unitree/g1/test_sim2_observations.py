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

"""Real G1 geometry and GR00T weights through ordinary worker/transport lifecycle."""

import uuid

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2 import unitree_g1_sim2
from dimos.robot.unitree.g1.sim2_policy import G1PolicyModule
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.native_e2e


@pytest.fixture
def deployed_g1(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = unitree_g1_sim2.global_config(zenoh_domain="sim2-g1-" + uuid.uuid4().hex)
    deployment = ModuleCoordinator.build(blueprint)
    try:
        yield deployment
    finally:
        deployment.stop()


def test_g1_walks_moves_an_arm_and_resets_with_independent_sensors(deployed_g1):
    sim = deployed_g1.get_instance(SimModule)
    policy = deployed_g1.get_instance(G1PolicyModule)
    camera = deployed_g1.get_instance(SimCameraModule)
    lidar = deployed_g1.get_instance(SimLidarModule)
    wait_until(lambda: sim.status()["time_ns"] > 2_000_000_000, timeout=40)
    wait_until(lambda: camera.statistics()["acquisitions"] >= 3, timeout=20)

    def scans_ready():
        stats = lidar.statistics()
        assert not stats["error"], stats
        return stats["acquisitions"] >= 3

    wait_until(scans_ready, timeout=20, message=str(lidar.statistics()))
    before = sim.inspect()[0]
    assert poses_by_id(before)["g1/pelvis"].position.z > 0.68
    policy.arm_joint("g1/left_elbow_joint", 1.4)
    policy.move(Twist(linear=Vector3(0.2, 0, 0)), duration=20)
    wait_until(lambda: sim.status()["time_ns"] > before.header.time_ns + 3_000_000_000, timeout=40)
    after, joints = sim.inspect(("g1/left_elbow_joint",))
    policy.move(Twist(), duration=1)
    assert poses_by_id(after)["g1/pelvis"].position.z > 0.68
    assert (
        poses_by_id(after)["g1/pelvis"].position.x - poses_by_id(before)["g1/pelvis"].position.x
        > 0.2
    )
    assert joints.position[0] == pytest.approx(1.4, abs=0.12)
    reset = sim.reset()[0]
    wait_until(
        lambda: sim.status()["epoch"] == reset.header.epoch
        and sim.status()["time_ns"] > 1_000_000_000,
        timeout=30,
    )
    assert poses_by_id(sim.inspect()[0])["g1/pelvis"].position.z > 0.68
    assert (
        sim.status()["error"] == camera.statistics()["error"] == lidar.statistics()["error"] == ""
    )
    assert policy.status()["inferences"] > 250
