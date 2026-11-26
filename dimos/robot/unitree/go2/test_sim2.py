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

from __future__ import annotations

import asyncio
from collections.abc import Iterator
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.sim2.world_observations import WorldMirror

if TYPE_CHECKING:
    from dimos.robot.unitree.go2.sim2 import Go2Sim

pytest.importorskip("pxr")
# Go2Sim loads its policy and meshes from Git LFS data, which GitHub-hosted runners do not fetch.
pytestmark = pytest.mark.self_hosted


@pytest.fixture
def go2_sim() -> Iterator[Go2Sim]:
    from dimos.robot.unitree.go2.sim2 import Go2Sim

    module = Go2Sim()
    module._world = WorldMirror()
    try:
        yield module
    finally:
        module.stop()


def test_the_world_holds_and_drives_a_selected_policy_with_its_own_gains_and_pose(
    tmp_path: Path,
) -> None:
    from dimos.robot.unitree.go2.sim2 import Go2Sim
    from dimos.sim2.mujoco.drives import usd_drives
    from dimos.simulation.go2_legged.test_policy import _blob

    path = tmp_path / "policy.bin"
    path.write_bytes(_blob())
    module = Go2Sim(policy=path)
    try:
        policy = module._policy
        gains = zip(policy.joint_names, policy.kp, policy.kd, policy.default_pose, strict=True)
        expected = {joint: pytest.approx((kp, kd, q)) for joint, kp, kd, q in gains}
        assert usd_drives(module.simulated_robot().model.usd) == expected
        assert usd_drives(module.get_usd()) == expected
    finally:
        module.stop()


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_a_non_finite_cmd_vel_leaves_the_last_good_command(go2_sim: Go2Sim, bad: float) -> None:
    asyncio.run(go2_sim.handle_cmd_vel(Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))))
    asyncio.run(go2_sim.handle_cmd_vel(Twist(Vector3(bad, 0.0, 0.0), Vector3(0.0, 0.0, bad))))

    assert np.array_equal(go2_sim._desired, [0.5, 0.0, 0.0])


def test_odom_keeps_the_go2_standing_above_its_mount(go2_sim: Go2Sim) -> None:
    from dimos.robot.assets.usd_utils.frames import usd_frame_pose

    usd = go2_sim.config.usd
    assert usd.source_path is not None
    standing = usd_frame_pose(usd.usd, usd.root_link, usd.source_path.parent)[2, 3]

    odom = go2_sim.odometry(
        np.array([2.0, -1.0, standing, 0.0, 0.0, 0.0, 1.0]), 1.0, go2_sim.config.odom_frame
    )

    assert standing > 0
    assert np.allclose([odom.position.x, odom.position.y, odom.position.z], [0.0, 0.0, standing])


def test_odom_is_in_the_frame_a_real_go2_reports_it_in(go2_sim: Go2Sim) -> None:
    from dimos.robot.unitree.go2.connection import ConnectionConfig

    odom = go2_sim.odometry(
        np.array([0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]), 1.0, go2_sim.config.odom_frame
    )

    assert odom.frame_id == ConnectionConfig.model_fields["odom_frame_id"].default


def test_the_simulated_body_is_the_root_link_of_a_real_go2() -> None:
    from dimos.robot.unitree.go2.connection import GO2_MODEL
    from dimos.robot.unitree.go2.sim2 import GO2_BODY, GO2_ODOM_TF, GO2_USD

    assert GO2_USD.root_link == GO2_BODY == GO2_ODOM_TF["tf_frames_local"]["body"]
    assert GO2_BODY.rpartition("/")[2] == GO2_MODEL.load().root_link


def test_a_go2_publishes_its_odometry_and_only_the_joints_that_move(go2_sim: Go2Sim) -> None:
    go2_sim._tf = go2_sim._create_tfbuffer(None)
    published: list[set[tuple[str, str]]] = []
    odometry: list[PoseStamped] = []
    publish = go2_sim._tf.publish

    def recorded(*transforms: Transform) -> None:
        published.append({(t.frame_id, t.child_frame_id) for t in transforms})
        publish(*transforms)

    go2_sim._tf.publish = recorded  # type: ignore[method-assign]
    go2_sim.odom.publish = odometry.append  # type: ignore[method-assign,assignment]
    go2_sim._publish_pose(np.array([0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]), 1.0, {})

    moving = {(t.frame_id, t.child_frame_id) for t in go2_sim.moving_transforms()}
    assert moving
    # Odometry is an estimate, so it goes out on odom for an OdomTf, never on the hardware's tf.
    assert published == [moving]
    assert [reading.frame_id for reading in odometry] == [go2_sim.config.odom_frame]


def test_a_mounted_go2_s_mount_holds_its_odom_and_nothing_static_holds_its_body() -> None:
    from dimos.robot.unitree.go2.sim2 import Go2Sim

    go2 = Go2Sim(instance_name="logistics/go2/go2sim", attached_to="logistics/go2")
    try:
        body = go2.tf_frames[go2.config.usd.root_link]
        static = {(t.frame_id, t.child_frame_id) for t in go2.static_transforms()}
    finally:
        go2.stop()

    assert ("logistics/go2", go2.config.odom_frame) in static
    assert not any(f"{body}/".startswith(f"{child}/") for _, child in static), sorted(static)
    assert any(parent == body for parent, _ in static), "the body's own static links still go out"
