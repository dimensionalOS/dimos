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
from collections.abc import AsyncIterator, Iterator
import math
import threading
from types import SimpleNamespace
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest

pytest.importorskip("pxr")

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.robot.unitree.go2 import webrtc_sim
from dimos.robot.unitree.go2.webrtc_sim import BODY, SERVOS, Go2WebRtcSim, go2_usd
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.mujoco.drives import usd_drives
from dimos.sim2.scene import LOGISTICS, scene_usd
from dimos.sim2.sim import JointCommand, SimulatedRobot, compose_usds

_SPOT = "logistics/spawns/go2"


def _world() -> MujocoBackend:
    go2 = go2_usd()
    served = usd_under_namespace(resolve_usd_assets(go2.usd, str(go2.asset_root)), _SPOT)
    scene = resolve_usd_assets(scene_usd(LOGISTICS).usd, str(LOGISTICS.parent))
    world = compose_usds([(scene, None), (served, _SPOT)])
    return MujocoBackend(world, [SimulatedRobot(_SPOT, go2, free=False)])


def _drive(backend: MujocoBackend, velocity: tuple[float, ...], seconds: float) -> Iterator[None]:
    """Steps the world with its servos commanded at `velocity`, yielding after every step."""
    gains = tuple(usd_drives(go2_usd().usd, go2_usd().asset_root)[name][1] for name in SERVOS)
    for _ in range(round(seconds / backend.model.opt.timestep)):
        backend.apply([JointCommand(_SPOT, SERVOS, velocity=velocity, kp=(0.0,) * 3, kd=gains)])
        backend._apply_pending()
        mujoco.mj_step(backend.model, backend.data)
        yield


def test_driven_its_body_tracks_its_servos_stays_up_and_stops_nose_first_at_a_wall() -> None:
    """Its body moves at its commanded velocities without falling or rising; then, driven nose first into the logistics room's north wall 5 m off, it stops there and its camera stays beyond its near plane."""
    backend = _world()
    servo = {name: backend.data.joint(f"{_SPOT}/{name}") for name in SERVOS}
    base, camera = (backend.data.body(f"{_SPOT}/{name}") for name in ("base_link", "camera_link"))
    mujoco.mj_forward(backend.model, backend.data)
    height = float(base.xpos[2])
    spawn_qpos, spawn_qvel = backend.data.qpos.copy(), backend.data.qvel.copy()
    for _ in _drive(backend, (0.4, 0.0, 0.5), seconds=1.0):
        pass
    assert (servo["x"].qvel[0], servo["yaw"].qvel[0]) == pytest.approx((0.4, 0.5), abs=0.01)
    assert base.xpos[2] == pytest.approx(height, abs=0.01), "its body fell or rose"

    # Back at its spawn, turned to face the wall.
    backend.data.qpos[:], backend.data.qvel[:] = spawn_qpos, spawn_qvel
    servo["yaw"].qpos[0] = math.pi / 2
    wall = backend.model.geom("/logistics/Geometry/north_wall")
    face = wall.pos[1] - wall.size[1]
    near = webrtc_sim.Go2WebRtcSimConfig().render.near
    closest = min(face - float(camera.xpos[1]) for _ in _drive(backend, (0.0, 1.0, 0.0), 8.0))
    reached = float(servo["y"].qpos[0])
    assert face - 1.0 < reached < face, f"it stopped at y={reached:.2f}, the wall is at {face:.2f}"
    assert closest > near, f"its camera got {closest:.3f} m from the wall, nearer than {near} m"


@pytest.fixture
def go2() -> Iterator[Go2WebRtcSim]:
    go2 = Go2WebRtcSim(instance_name="main/go2webrtcsim", attached_to="main")
    yield go2
    go2.stop()


class _DrivenWorld:
    """A world whose servos move a little further each of its 5 ticks, as a world being driven does."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.time = 0.0

    async def ticks(self, rate_hz: float) -> AsyncIterator[float]:
        for _ in range(5):
            self.time += 1 / rate_hz
            yield self.time

    def joints(self, robot: str) -> tuple[list[str], NDArray[np.float64]]:
        return list(SERVOS), np.array(
            [[self.time, 0.4, 0.0], [self.time, 0.0, 0.0], [self.time, 0.5, 0.0]]
        )


def test_driven_it_sends_odometry_in_its_namespaced_frames_and_only_static_tf(
    go2: Go2WebRtcSim, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Driven, it sends odometry and puts nothing on tf; its static edges never place its body or what its servos move."""
    assert go2.tf_frames[BODY] == go2.frame.base_link == "main/base_link"
    assert (go2.frame.camera_optical, go2.attach_frame) == ("main/camera_optical", "main/odom")
    # Facing +y, sliding +y along its mount at 0.5 m/s and turning at 0.2 rad/s.
    odom = go2._odometry((1.0, -2.0, math.pi / 2), (0.0, 0.5, 0.2), 5.0)
    assert (odom.frame_id, odom.child_frame_id, odom.ts) == ("main/odom", "main/base_link", 5.0)
    assert (odom.position.x, odom.position.y) == (1.0, -2.0)
    assert odom.position.z == pytest.approx(0.35, abs=0.01)
    assert odom.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert (odom.vx, odom.vy, odom.wz) == pytest.approx((0.5, 0.0, 0.2))

    on_tf: list[Any] = []
    odometry: list[Odometry] = []
    # Whatever it would put on tf, through its tfbuffer or its tf port.
    tfbuffer = SimpleNamespace(publish=lambda *edges: on_tf.extend(edges), dispose=lambda: None)
    monkeypatch.setattr(go2, "_tf", tfbuffer)
    monkeypatch.setattr(go2.tf, "publish", on_tf.append)
    monkeypatch.setattr(go2.odom, "publish", odometry.append)
    monkeypatch.setattr(go2.world_actions, "publish", lambda action: None)
    monkeypatch.setattr(webrtc_sim, "world_action", lambda world, command: None)
    go2._world = _DrivenWorld()
    asyncio.run(go2._drive())
    assert on_tf == []
    assert len(odometry) == 5
    static = go2.static_transforms(ts=0.0)
    moving_children = {edge.child_frame_id for edge in go2.moving_transforms()}
    assert moving_children, "its USD's servos move frames, which tf would have to carry"
    assert not {edge.child_frame_id for edge in static} & (moving_children | {"main/base_link"})
    edges = {(edge.frame_id, edge.child_frame_id) for edge in static}
    assert edges == go2.tf_edge_ids
    assert {("main", "main/odom"), ("main/base_link", "main/camera_link")} <= edges


@pytest.mark.skipif_no_turbojpeg
def test_its_camera_sends_jpegs_with_its_lens_stamped_as_the_frame_was_drawn(
    go2: Go2WebRtcSim, monkeypatch: pytest.MonkeyPatch
) -> None:
    images: list[Any] = []
    infos: list[Any] = []
    monkeypatch.setattr(go2.color_image, "publish", images.append)
    monkeypatch.setattr(go2.camera_info, "publish", infos.append)
    info = go2._camera_info()
    go2._publish_frame(info, np.full((info.height, info.width, 3), (200, 40, 10), np.uint8), 3.0)
    [jpeg] = images
    assert jpeg.format == "jpeg" and jpeg.data[:2] == b"\xff\xd8"
    assert (jpeg.ts, jpeg.frame_id) == (3.0, "main/camera_optical")
    assert (jpeg.decode().width, jpeg.decode().height) == (info.width, info.height)
    assert [i.ts for i in infos] == [3.0]
