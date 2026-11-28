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

import asyncio
import math
from typing import Any

import mujoco
import numpy as np
import pytest

pytest.importorskip("pxr")
# Its USD names its Git LFS meshes, over the hosted runner's pull cap.
pytestmark = pytest.mark.self_hosted

from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
from dimos.robot.unitree.go2.body_sim import GO2_BODY, GO2_JOINT_NAMES, Go2BodySim
from dimos.sim2.body import is_site
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.scene import LOGISTICS, scene_usd
from dimos.sim2.sim import compose_usds, read_actions

_SPOT = "logistics/spawns/go2"


@pytest.fixture
def go2() -> Any:
    body = Go2BodySim(instance_name=f"{_SPOT}/go2bodysim", attached_to="go2")
    yield body
    body.stop()


def _world(go2: Go2BodySim) -> MujocoBackend:
    usd = go2.config.usd
    served = usd_under_namespace(resolve_usd_assets(usd.usd, str(usd.asset_root)), _SPOT)
    room = resolve_usd_assets(scene_usd(LOGISTICS).usd, str(LOGISTICS.parent))
    world = compose_usds([(room, None), (served, _SPOT)])
    return MujocoBackend(world, [go2.simulated_robot()])


def _step(backend: MujocoBackend, go2: Go2BodySim, seconds: float) -> None:
    """Step ``seconds`` of the world, its body's frames reaching ``go2`` as world_observations would."""
    for _ in range(round(seconds / backend.model.opt.timestep)):
        backend._apply_pending()
        mujoco.mj_step(backend.model, backend.data)
    go2._world.apply(backend.frame())


def test_its_mounts_and_imu_are_isaac_sim_sites_and_its_odometry_holds_its_body(
    go2: Go2BodySim,
) -> None:
    usd = go2.config.usd
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd.usd, "a robot", usd.asset_root))
    for frame in [*Go2BodySim.mounts, go2.config.imu_site]:
        assert is_site(stage.GetPrimAtPath(f"/{frame}")), frame
    assert go2.frame.lidar == f"{_SPOT}/lidar"
    assert go2.frame.camera == f"{_SPOT}/camera"
    # Nothing static places its body: its odometry does.
    static = {(t.frame_id, t.child_frame_id) for t in go2.static_transforms()}
    assert go2.body_frame == go2.tf_frames[GO2_BODY] == f"{_SPOT}/base_link"
    assert (_SPOT, go2.attach_frame) in static
    assert not any(child == go2.body_frame for _, child in static)
    assert (go2.body_frame, f"{_SPOT}/camera") in static


def test_held_at_its_standing_pose_by_world_actions_it_stands_and_reads_its_joints_and_gravity(
    go2: Go2BodySim,
) -> None:
    backend = _world(go2)
    sent: list[Any] = []
    go2.world_actions.publish = sent.append  # type: ignore[method-assign,assignment]
    command = JointState(name=["go2/FL_calf"], position=[-1.2])
    asyncio.run(go2.handle_position_command(command))
    assert sent == [], "it commands nothing before the world shows it"
    go2._world.apply(backend.frame())
    asyncio.run(go2.handle_position_command(command))
    # A position command is a world action on its joints with its USD's gains.
    [action] = read_actions(sent[0], backend.world)
    assert action.entity == _SPOT
    assert action.names == ("FL_calf_joint",)
    assert action.position == (-1.2,)
    assert (action.kp, action.kd) == ((20.0,), (0.5,))

    standing = JointState(
        name=list(GO2_JOINT_NAMES), position=[0.0, 0.8, -1.5] * 4, velocity=[0.0] * 12
    )
    go2.world_actions.publish = lambda message: backend.apply(  # type: ignore[method-assign,assignment,func-returns-value]
        read_actions(message, backend.world)
    )
    base = backend.model.body(f"{_SPOT}/base_link").id
    for _ in range(20):
        asyncio.run(go2.handle_position_command(standing))
        _step(backend, go2, 0.1)

    states: list[JointState] = []
    imus: list[Imu] = []
    go2.joint_state.publish = states.append  # type: ignore[method-assign,assignment]
    go2.imu.publish = imus.append  # type: ignore[method-assign,assignment]
    go2._publish_reading(go2._imu_in_body)

    # Dropped from its spawn point onto the floor, it stands on its legs, upright.
    assert 0.2 < float(backend.data.xpos[base][2]) < 0.45, "it fell"
    assert backend.data.xmat[base].reshape(3, 3)[2, 2] > 0.95, "it tipped over"
    [state] = states
    assert state.name == list(GO2_JOINT_NAMES)
    # Each joint as the world has it, in its stream's order.
    model, data = backend.model, backend.data
    world = [
        data.qpos[model.jnt_qposadr[model.joint(f"{_SPOT}/{Go2BodySim.usd_joint(name)}").id]]
        for name in GO2_JOINT_NAMES
    ]
    assert np.allclose(state.position, world, atol=1e-6)
    [imu] = imus
    assert imu.linear_acceleration.z == pytest.approx(9.81, abs=0.5)
    assert abs(imu.angular_velocity.z) < 0.1


def test_its_odometry_starts_on_its_mount_and_its_twist_is_in_its_body_s_frame(
    go2: Go2BodySim,
) -> None:
    standing = go2._body_on_mount[:3, 3]
    # Where it stood when the world first showed it: its odom's origin is its mount.
    start = np.array([5.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    start[:3] += standing
    first = go2.odometry(start, np.zeros(6), 1.0)
    assert [first.x, first.y, first.z] == pytest.approx(list(standing))
    # Turned a quarter left and moving along the world's +y, it moves along its own +x.
    turned = start.copy()
    turned[3:] = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    odom = go2.odometry(turned, np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.3]), 2.0)

    assert odom.frame_id == go2.attach_frame
    assert odom.child_frame_id == go2.body_frame
    assert [odom.twist.linear.x, odom.twist.linear.y] == pytest.approx([0.5, 0.0], abs=1e-9)
    assert odom.twist.angular.z == pytest.approx(0.3)
