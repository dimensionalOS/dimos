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

"""Exercise the shipped K1 blueprint through its real worker transports."""

import math
import uuid

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import UsdModel
from dimos.robot.booster.k1.blueprints.booster_k1_sim2 import booster_k1_sim2
from dimos.robot.booster.k1.control_config import EFFORT_LIMITS, JOINTS
from dimos.robot.booster.k1.policy_module import K1PolicyModule
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.module import SimColorCameraModule, SimCompressedCameraModule
from dimos.utils.data import LfsPath
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.self_hosted


def test_native_asset_preserves_colliders_friction_and_motor_limits():
    backend = MujocoBackend()
    try:
        backend.load(UsdModel.from_file(LfsPath("booster_k1/world.usda")), 2_000_000)
        assert backend.joint_ids == JOINTS
        assert np.bincount(backend.model.geom_contype).tolist() == [24, 21]
        assert backend.model.geom_friction[0, 0] == pytest.approx(0.4)
        np.testing.assert_allclose(backend.model.actuator_forcerange[:, 1], EFFORT_LIMITS)
    finally:
        backend.close()


@pytest.fixture
def k1(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    coordinator = ModuleCoordinator.build(
        booster_k1_sim2.global_config(
            zenoh_domain="k1-test-" + uuid.uuid4().hex,
        )
    )
    try:
        yield (
            coordinator.get_instance(SimModule),
            coordinator.get_instance(K1PolicyModule),
            coordinator.get_instance(SimCompressedCameraModule),
            coordinator.get_instance(SimColorCameraModule),
        )
    finally:
        coordinator.stop()


def test_balances_walks_and_stops_in_sim2(k1):
    sim, policy, camera, stereo = k1
    wait_until(lambda: sim.status()["time_ns"] >= 2_000_000_000, timeout=20)
    standing = poses_by_id(sim.inspect())["k1/trunk"]
    assert 0.48 < standing.position.z < 0.62
    assert abs(standing.position.x) < 0.2
    policy.move(Twist(linear=Vector3(0.3, 0, 0)), duration=4)
    start = sim.inspect().header.time_ns
    wait_until(lambda: sim.status()["time_ns"] >= start + 3_000_000_000, timeout=20)
    walking = poses_by_id(sim.inspect())["k1/trunk"]
    assert walking.position.x - standing.position.x > 0.25
    assert 0.48 < walking.position.z < 0.62
    policy.move(Twist(angular=Vector3(0, 0, 0.4)), duration=3)
    start = sim.inspect().header.time_ns
    wait_until(lambda: sim.status()["time_ns"] >= start + 2_000_000_000, timeout=20)
    turned = poses_by_id(sim.inspect())["k1/trunk"]
    q = turned.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    assert 0.2 < yaw < 1.5
    policy.move(Twist(), duration=0.1)
    start = sim.inspect().header.time_ns
    wait_until(lambda: sim.status()["time_ns"] >= start + 2_000_000_000, timeout=20)
    stopped = poses_by_id(sim.inspect())["k1/trunk"]
    start = sim.inspect().header.time_ns
    wait_until(lambda: sim.status()["time_ns"] >= start + 1_000_000_000, timeout=20)
    settled = poses_by_id(sim.inspect())["k1/trunk"]
    assert abs(settled.position.x - stopped.position.x) < 0.12
    assert 0.48 < settled.position.z < 0.62
    assert sim.status()["rejected_commands"] == 0
    assert policy.status()["inferences"] > 200

    poses = poses_by_id(sim.inspect())
    head = poses["k1/aahead_pitch_link"]
    turn = Rotation.from_quat(
        [head.orientation.x, head.orientation.y, head.orientation.z, head.orientation.w]
    )
    for frame, offset in (
        ("k1/realsense_optical", (0.053617, -0.0115, 0.10231)),
        ("k1/stereo_optical", (0.060138, 0.0351, 0.092774)),
    ):
        position = poses[frame].position
        relative = turn.inv().apply(
            [
                position.x - head.position.x,
                position.y - head.position.y,
                position.z - head.position.z,
            ]
        )
        np.testing.assert_allclose(relative, offset, atol=1e-6)

    for device, frame in ((camera, "k1/realsense_optical"), (stereo, "k1/stereo_optical")):
        color = device.peek_stream("color_image", timeout=10).decode()
        assert color.frame_id == frame
        assert color.data.shape == (240, 320, 3)
        assert color.data.std() > 10
    depth = camera.peek_stream("depth_image", timeout=10)
    assert depth.frame_id == "k1/realsense_optical"
    assert np.count_nonzero(depth.data > 0.5) > depth.data.size // 4
