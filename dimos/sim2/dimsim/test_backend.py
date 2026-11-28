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

from collections.abc import Iterator
import math
from pathlib import Path
import shutil
from typing import Any

import numpy as np
import pytest

pytest.importorskip("pxr")

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.actions import JointCommand
from dimos.sim2.dimsim.backend import WORLD_DIR, DimSimBackend
from dimos.sim2.dimsim.room import ROOM
from dimos.sim2.dimsim.sim import DimSimSim
from dimos.sim2.scene import scene_usd
from dimos.sim2.sim import compose_usds
from dimos.sim2.spec import ODOM_FRAME, SimulatedRobot
from dimos.sim2.world_observations import REFUSED, WorldMirror

TIMESTEP = 0.02
# The room's spawn point the bot mounts on, and so the bot's namespace and robot id.
BOT = "dimsim_room/spawns/go2"
SERVOS = ("x", "y", "yaw")
BOT_USD = Path(__file__).parent / "assets" / "planar_bot.usda"


def _needs_dimsim() -> None:
    if shutil.which("deno") is None:
        pytest.skip("the dimsim world runs on Deno, which is not installed")
    if not (WORLD_DIR / "sim2" / "world.ts").is_file():
        pytest.skip("the dimsim world ships with a dimos checkout, not the package")


class _Stepped:
    def __init__(self, backend: DimSimBackend) -> None:
        self.backend, self.world = backend, WorldMirror()
        self.received: list[Any] = []
        self.step(0)

    def drive(self, vx: float, vy: float, wz: float) -> None:
        self.backend.apply(
            [JointCommand(BOT, SERVOS, velocity=(vx, vy, wz), kp=(0.0,) * 3, kd=(400.0,) * 3)]
        )

    def step(self, steps: int = 1) -> WorldMirror:
        _at, frame = self.backend.advance(steps) if steps else (0.0, self.backend.frame())
        self.world.apply(frame)
        self.received += self.world.received
        return self.world

    def pose(self, name: str) -> list[float]:
        entity = self.world.entity(name)
        assert entity is not None and entity.pose is not None
        return list(entity.pose.tolist())

    def servos(self) -> dict[str, tuple[float, float]]:
        names, rows = self.world.joints(BOT)
        return {name: (float(row[0]), float(row[1])) for name, row in zip(names, rows, strict=True)}


def _world() -> tuple[UsdModel, SimulatedRobot]:
    bot = UsdModel.from_file(BOT_USD)
    served = usd_under_namespace(resolve_usd_assets(bot.usd, str(bot.asset_root)), BOT)
    world = compose_usds(
        [(resolve_usd_assets(scene_usd(ROOM).usd, str(ROOM.parent)), None), (served, BOT)]
    )
    return world, SimulatedRobot(BOT, bot, odom_frame=f"{BOT}/odom", free=False)


@pytest.fixture
def backend() -> Iterator[_Stepped]:
    _needs_dimsim()
    world, robot = _world()
    engine = DimSimBackend(world, [robot], timestep=TIMESTEP)
    yield _Stepped(engine)
    engine.close()


def test_the_world_says_what_is_in_it_and_holds_the_robot_on_its_mount(backend: _Stepped) -> None:
    world = backend.step(5)
    assert world.entity("world") is not None
    assert world.entity("world").cbor["backend"] == "dimsim"  # type: ignore[union-attr]
    assert world.entity(BOT).cbor[ODOM_FRAME] == f"{BOT}/odom"  # type: ignore[union-attr]
    # Its root is the anchor its servos move off, which stays on the mount.
    assert backend.pose(BOT)[:3] == pytest.approx([0.0, 0.0, 0.35], abs=1e-6)
    assert backend.pose("crate")[:3] == pytest.approx([1.5, 0.0, 0.2], abs=0.02)
    assert sorted(backend.servos()) == sorted(SERVOS)


def test_its_servos_move_at_the_velocity_commanded_and_push_the_crate(backend: _Stepped) -> None:
    backend.drive(1.0, 0.0, 0.0)
    backend.step(100)
    servos = backend.servos()
    assert servos["x"][0] > 0.6
    # Slowed by the crate it pushes, never faster than commanded.
    assert 0.0 < servos["x"][1] <= 1.0 + 1e-6
    assert backend.pose("crate")[0] > 1.6, "the crate was not pushed"


def test_a_turn_reads_on_its_yaw_servo_and_its_slides_stay_on_the_mount_axes(
    backend: _Stepped,
) -> None:
    backend.drive(0.0, 0.0, 0.5)
    backend.step(50)
    yaw, rate = backend.servos()["yaw"]
    assert yaw == pytest.approx(0.5 * 50 * TIMESTEP, abs=1e-3)
    assert rate == pytest.approx(0.5, abs=1e-3)
    # Turned, its y servo still slides along the mount's y.
    backend.drive(0.0, -0.5, 0.0)
    backend.step(50)
    servos = backend.servos()
    assert servos["y"][0] == pytest.approx(-0.5, abs=0.05)
    assert servos["x"][0] == pytest.approx(0.0, abs=0.01)


def test_a_wall_stops_it(backend: _Stepped) -> None:
    """Driven at the room's north wall, whose face is 3 m off, it stops against the wall instead of passing through."""
    backend.drive(0.0, 1.0, 0.0)
    backend.step(250)
    # The wall's face, less the box's half width and the character controller's offset.
    assert backend.servos()["y"][0] == pytest.approx(3.0 - 0.2 - 0.05, abs=0.06)


def test_the_last_state_sent_of_a_body_that_stopped_is_still(backend: _Stepped) -> None:
    backend.drive(1.0, 0.0, 0.0)
    backend.step(70)
    crate = backend.world.entity("crate")
    assert crate is not None and crate.velocity is not None
    assert crate.velocity[0] > 0.1, "the pushed crate sent no velocity"
    backend.drive(0.0, 0.0, 0.0)
    backend.step(800)
    crate = backend.world.entity("crate")
    assert crate is not None and crate.velocity is not None
    assert crate.velocity.tolist() == pytest.approx([0.0] * 6, abs=1e-3)
    assert backend.servos()["x"][1] == pytest.approx(0.0, abs=1e-3)


def test_what_its_servos_cannot_do_is_refused(backend: _Stepped) -> None:
    backend.backend.apply([JointCommand(BOT, ("neck",), velocity=(1.0,))])
    backend.backend.apply([JointCommand(BOT, ("x",), position=(1.0,), kp=(10.0,))])
    backend.step()
    assert sum(key == REFUSED for _, key, _ in backend.received) >= 1


def test_a_robot_without_planar_servos_is_refused() -> None:
    _needs_dimsim()
    toy = UsdModel.from_layer(
        '#usda 1.0\n(\n    defaultPrim = "bot"\n)\ndef Xform "bot"\n{\n'
        '    def Cube "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]\n'
        "    )\n    {\n        float physics:mass = 5\n        double size = 0.4\n    }\n}\n"
    )
    composed = compose_usds(
        [
            (resolve_usd_assets(scene_usd(ROOM).usd, str(ROOM.parent)), None),
            (usd_under_namespace(toy.usd, BOT), BOT),
        ]
    )
    with pytest.raises(ValueError, match="three servos"):
        DimSimBackend(composed, [SimulatedRobot(BOT, toy)], timestep=TIMESTEP)


def test_stopping_before_the_modules_report_ends_the_wait() -> None:
    """_run waits on this in a thread, and cancelling the task cannot reach a blocked wait."""
    sim = DimSimSim()
    sim.stop()
    assert sim._built.is_set(), "a thread in _built.wait would never return"


def test_its_yaw_servo_counts_whole_turns(backend: _Stepped) -> None:
    backend.drive(0.0, 0.0, 2.0)
    backend.step(round(2 * math.pi / 2.0 / TIMESTEP) + 10)
    assert backend.servos()["yaw"][0] > 2 * math.pi
    assert np.isfinite(backend.pose(BOT)).all()
