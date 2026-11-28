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

import math

import mujoco
import pytest

from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.robot.unitree.go2.webrtc_sim import BODY, SERVOS, Go2WebRtcSim, go2_usd
from dimos.sim2.actions import JointCommand
from dimos.sim2.assets.logistics import LOGISTICS
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.mujoco.drives import usd_drives
from dimos.sim2.scene import scene_usd
from dimos.sim2.sim import compose_usds
from dimos.sim2.spec import SimulatedRobot

pytest.importorskip("pxr")

_SPOT = "logistics/spawns/go2"


def _world() -> MujocoBackend:
    go2 = go2_usd()
    served = usd_under_namespace(resolve_usd_assets(go2.usd, str(go2.asset_root)), _SPOT)
    world = compose_usds(
        [
            (resolve_usd_assets(scene_usd(LOGISTICS).usd, str(LOGISTICS.parent)), None),
            (served, _SPOT),
        ]
    )
    return MujocoBackend(world, [SimulatedRobot(_SPOT, go2, free=False)])


def _position(backend: MujocoBackend, servo: str) -> float:
    joint = backend.model.joint(f"{_SPOT}/{servo}")
    return float(backend.data.qpos[backend.model.jnt_qposadr[joint.id]])


def _velocity(backend: MujocoBackend, servo: str) -> float:
    joint = backend.model.joint(f"{_SPOT}/{servo}")
    return float(backend.data.qvel[backend.model.jnt_dofadr[joint.id]])


def test_its_body_moves_at_the_velocity_its_servos_are_commanded_and_stays_up() -> None:
    backend = _world()
    gains = tuple(usd_drives(go2_usd().usd, go2_usd().asset_root)[name][1] for name in SERVOS)
    base = backend.model.body(f"{_SPOT}/base_link").id
    mujoco.mj_forward(backend.model, backend.data)
    height = backend.data.xpos[base][2]
    seconds = 1.0
    for _ in range(round(seconds / backend.model.opt.timestep)):
        backend.apply(
            [JointCommand(_SPOT, SERVOS, velocity=(0.4, 0.0, 0.5), kp=(0.0,) * 3, kd=gains)]
        )
        backend._apply_pending()
        mujoco.mj_step(backend.model, backend.data)

    assert _velocity(backend, "x") == pytest.approx(0.4, abs=0.01)
    assert _velocity(backend, "yaw") == pytest.approx(0.5, abs=0.01)
    assert backend.data.xpos[base][2] == pytest.approx(height, abs=0.01), "its body fell or rose"


def test_a_wall_stops_it() -> None:
    """Driven at the logistics room's north wall, 5 m off, its body stops against the wall instead of passing through."""
    backend = _world()
    gains = tuple(usd_drives(go2_usd().usd, go2_usd().asset_root)[name][1] for name in SERVOS)
    wall = backend.model.geom("/logistics/Geometry/north_wall")
    face = backend.model.geom_pos[wall.id][1] - backend.model.geom_size[wall.id][1]
    for _ in range(round(8.0 / backend.model.opt.timestep)):
        backend.apply(
            [JointCommand(_SPOT, SERVOS, velocity=(0.0, 1.0, 0.0), kp=(0.0,) * 3, kd=gains)]
        )
        backend._apply_pending()
        mujoco.mj_step(backend.model, backend.data)

    reached = _position(backend, "y")
    assert face - 1.0 < reached < face, (
        f"it stopped at y={reached:.2f}, the wall's face is at {face:.2f}"
    )


def test_its_frames_are_the_names_a_real_go2_gives_them_under_its_namespace() -> None:
    go2 = Go2WebRtcSim(instance_name="main/go2webrtcsim", attached_to="main")
    try:
        assert go2.tf_frames[BODY] == go2.frame.base_link == "main/base_link"
        assert go2.frame.camera_optical == "main/camera_optical"
        assert go2.attach_frame == "main/odom"
        # Odometry holds its body: no edge places it, nor the frames its servos move.
        children = {child for _, child in go2.tf_edge_ids}
        assert "main/base_link" not in children
        assert ("main", "main/odom") in go2.tf_edge_ids
        assert ("main/base_link", "main/camera_link") in go2.tf_edge_ids
    finally:
        go2.stop()


def test_its_odometry_turns_its_servos_into_a_pose_in_its_odom_frame() -> None:
    go2 = Go2WebRtcSim(instance_name="main/go2webrtcsim", attached_to="main")
    try:
        odom = go2._odometry(1.0, -2.0, math.pi / 2, 5.0)
    finally:
        go2.stop()
    assert odom.frame_id == "main/odom"
    assert (odom.position.x, odom.position.y) == (1.0, -2.0)
    assert odom.position.z == pytest.approx(0.35, abs=0.01)
    assert odom.orientation.z == pytest.approx(math.sin(math.pi / 4))
