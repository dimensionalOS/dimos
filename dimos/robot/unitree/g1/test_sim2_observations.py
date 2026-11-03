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

from contextlib import closing
import uuid

from dimos_lcm.sim_msgs import Observations
import numpy as np
import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2 import unitree_g1_sim2
from dimos.robot.unitree.g1.demo_sim2_control import G1PolicyDemo
from dimos.sim2.physics.module import SimModule
from dimos.sim2.sensors.module import SimSensorModule
from dimos.sim2.sensors.mujoco import MujocoSensors
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


def test_g1_walks_moves_an_arm_and_resets_with_pose_only_sensors(deployed_g1):
    sim = deployed_g1.get_instance(SimModule)
    policy = deployed_g1.get_instance(G1PolicyDemo)
    sensors = deployed_g1.get_instance(SimSensorModule)
    wait_until(lambda: sim.state().sim_time > 2, timeout=30)
    wait_until(
        lambda: sensors.statistics()["images"] >= 3 and sensors.statistics()["scans"] >= 3,
        timeout=20,
    )
    layout = sim.describe()
    root = layout.bodies.index("g1/pelvis")
    joint = layout.joints.index("g1/left_elbow_joint")
    before = sim.state()
    policy.arm_joint("g1/left_elbow_joint", 1.4)
    policy.move(Twist(linear=Vector3(0.2, 0, 0)), duration=10)
    wait_until(lambda: sim.state().sim_time > before.sim_time + 3, timeout=30)
    after = sim.state()
    policy.move(Twist(), duration=1)
    assert after.poses[root, 2] > 0.68
    assert after.poses[root, 0] - before.poses[root, 0] > 0.2
    assert after.position[joint] == pytest.approx(1.4, abs=0.12)

    # A new consumer has never seen the robot's spawn, qpos, joint stream or old poses.
    with closing(MujocoSensors(sim.describe_world(), layout)) as reader:
        message = Observations.lcm_decode(sim.sample().lcm_encode())
        sample = reader.observe(message)
        assert reader.model.nq == reader.model.nu == 0
        rgb, depth = reader.camera("g1/camera", sample)
        assert rgb.std() > 10 and np.isfinite(depth).all()
        old_epoch = message.epoch
        reset = sim.reset()
        wait_until(
            lambda: sim.state().epoch == reset.epoch and sim.state().sim_time > 1, timeout=20
        )
        reader.observe(sim.sample())
        assert reader.history[-1].epoch > old_epoch
        assert reader.observe(message) is None
    assert sim.state().poses[root, 2] > 0.68
    assert sim.timing()["error"] == sensors.statistics()["error"] == ""
    assert sim.timing()["rejected_commands"] == 0
