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
from collections.abc import Sequence
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

# Before importing the sim, whose spawn points register a USD schema on import.
pytest.importorskip("pxr")

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.hardware_module import HardwareSpec, Mount
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.scene import LOGISTICS, Logistics, toy_robot
from dimos.sim2.sim import (
    SimulatedRobot,
    SimulatedRobotSpec,
    SimWorld,
    WorldMirror,
    WorldWriter,
    _stream_names,
    compose_usds,
    meeting,
    on_mount,
    sim,
    validate,
)

# The logistics room's frame a dog mounts on, and so the dog's namespace.
_SPOT = "logistics/dog"
_ROOM = UsdModel.from_file(LOGISTICS).with_frames({"dog": ("logistics", (0, 0, 0), (0, 0, 0))}).usd


def _dog(namespace: str = _SPOT) -> str:
    return usd_under_namespace(toy_robot().usd, namespace)


def test_a_world_needs_exactly_one_unmounted_usd_and_mounts_something_offers() -> None:
    refused = {
        "exactly one unmounted": [[(_dog(), _SPOT)], [(_ROOM, None), (_ROOM, None)]],
        r"mount frames \['cat'\]": [[(_ROOM, None), (_dog(), "cat")]],
        # A mounted USD holds its mount's path as its namespace, which is no frame anything else offers.
        r"mount frames \['logistics/cat'\]": [
            [(_ROOM, None), (_dog("logistics/cat"), "logistics/cat")]
        ],
    }
    for refusal, worlds in refused.items():
        for usds in worlds:
            with pytest.raises(ValueError, match=refusal):
                compose_usds(usds)


def test_the_sim_finds_parts_by_their_rpcs_names_robots_once_and_imports_no_engine() -> None:
    room = SimpleNamespace(rpcs={"get_usd", "attached_to", "get_hardware_transform_ids", "start"})
    dog = SimpleNamespace(
        rpcs={"get_usd", "attached_to", "get_hardware_transform_ids", "simulated_robot"}
    )
    other = SimpleNamespace(rpcs={"get_usd", "describe"})
    assert meeting(HardwareSpec, [room, dog, other]) == [room, dog]
    assert meeting(SimulatedRobotSpec, [room, dog, other]) == [dog]
    with pytest.raises(ValueError, match="duplicate"):
        validate([SimulatedRobot("dog", toy_robot()), SimulatedRobot("dog", toy_robot())])
    with pytest.raises(ValueError, match="namespace"):
        validate([SimulatedRobot("", toy_robot())])

    choice = sim({"mujoco": SimWorld.blueprint(), "other": SimWorld.blueprint()}, default="mujoco")
    assert (choice.name, choice.default, set(choice.choices)) == (
        "sim",
        "mujoco",
        {"mujoco", "other"},
    )
    assert all(isinstance(option, Blueprint) for option in choice.choices.values())
    # A mounted blueprint shares the sim streams and namespaces the rest.
    mounted = on_mount(autoconnect(Logistics.blueprint()), Mount("room/spot", "spot")).solidify()
    assert _stream_names(mounted) == {
        "spot/hardware_file_request",
        "spot/hardware_file_response",
        "tf",
        "tf_static",
        "world_observations",
    }
    code = (
        "import sys, dimos.sim2.sim; assert not [m for m in sys.modules if m.startswith('mujoco')]"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


PERIOD = 0.05
# How late the event loop may wake a sleeping tick.
SLACK = 0.01


def _started_world() -> WorldMirror:
    mirror, writer = WorldMirror(), WorldWriter()
    writer.add("world")
    mirror.apply(writer.end_frame(0.0))
    return mirror


async def _tick_times(work: Sequence[float]) -> list[float]:
    """When a sensor ticks whose work on each tick takes ``work[i]`` wall seconds."""
    ticks: list[float] = []
    async for tick in _started_world().ticks(1 / PERIOD):
        ticks.append(tick)
        if len(ticks) > len(work):
            break
        await asyncio.sleep(work[len(ticks) - 1])
    return ticks


@pytest.mark.parametrize(
    "work",
    [
        [0.0] * 8,
        [1.5 * PERIOD] + [0.0] * 7,
        [1.9 * PERIOD] + [0.0] * 7,
        [3.0 * PERIOD] + [0.0] * 7,
        [1.4 * PERIOD] * 6,
    ],
    ids=["idle", "one_slow_tick", "one_nearly_two_periods", "one_very_slow_tick", "always_slow"],
)
def test_a_sensor_ticks_a_period_apart_or_as_soon_as_its_work_allows(work: list[float]) -> None:
    gaps = np.diff(asyncio.run(_tick_times(work)))
    # Never within a period of its last tick, and never a further period after its work is done.
    assert np.all(gaps >= np.maximum(PERIOD - SLACK, work)), gaps
    assert np.all(gaps < np.maximum(PERIOD, work) + PERIOD), gaps
    if work[-1] == 0.0:
        # Once caught up it is back on its rate, not drifting slower.
        assert gaps[-1] == pytest.approx(PERIOD, abs=SLACK), gaps


def test_a_sensor_always_behind_its_rate_still_lets_the_loop_s_other_tasks_run() -> None:
    async def behind() -> int:
        handled = 0

        async def handler() -> None:
            nonlocal handled
            while True:
                await asyncio.sleep(0)
                handled += 1

        handling = asyncio.ensure_future(handler())
        ticks = 0
        async for _ in _started_world().ticks(10_000):
            # Work that never yields, at a rate it can never keep up with.
            time.sleep(2 * PERIOD / 10)
            ticks += 1
            if ticks == 10:
                break
        handling.cancel()
        return handled

    assert asyncio.run(behind()) >= 5


def test_ticks_wait_for_the_first_frame_and_frames_are_stamped_in_wall_time() -> None:
    mirror, writer = WorldMirror(), WorldWriter()
    writer.add("world")

    async def first_tick() -> float:
        ticking = asyncio.ensure_future(anext(aiter(mirror.ticks(1 / PERIOD))))
        await asyncio.sleep(2 * PERIOD)
        assert not ticking.done(), "it ticked before the world had a frame"
        mirror.apply(writer.end_frame(0.0))
        return await ticking

    before = time.time()
    assert before <= asyncio.run(first_tick()) <= time.time()

    # Rebased after falling behind, the next frame is stamped with wall time.
    writer.start_ns = time.time_ns() - round(60 * 1e9)
    writer.rebase(1.0)
    frame = writer.end_frame(1.0 + PERIOD)
    assert frame.ts / 1e9 == pytest.approx(time.time() + PERIOD, abs=SLACK)
