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
import time

import numpy as np
import pytest

from dimos.sim2.world_observations import WorldMirror, WorldWriter

PERIOD = 0.05
# How late the event loop may wake a sleeping tick.
SLACK = 0.01


async def _tick_times(work: Sequence[float]) -> list[float]:
    """When a sensor ticks whose work on each tick takes ``work[i]`` wall seconds, once the world has a frame."""
    mirror, writer = WorldMirror(), WorldWriter()
    writer.add("world")
    mirror.apply(writer.end_frame(0.0))
    ticks: list[float] = []
    async for tick in mirror.ticks(1 / PERIOD):
        ticks.append(tick)
        if len(ticks) > len(work):
            break
        await asyncio.sleep(work[len(ticks) - 1])
    return ticks


@pytest.mark.parametrize(
    "work",
    [[0.0] * 8, [1.5 * PERIOD] + [0.0] * 7, [1.9 * PERIOD] + [0.0] * 7, [3.0 * PERIOD] + [0.0] * 7],
    ids=["idle", "one_slow_tick", "one_nearly_two_periods", "one_very_slow_tick"],
)
def test_a_sensor_never_ticks_within_a_period_of_its_last_tick(work: list[float]) -> None:
    gaps = np.diff(asyncio.run(_tick_times(work)))
    assert gaps.min() >= PERIOD - SLACK, gaps
    # Once caught up it is back on its rate, not drifting slower.
    assert gaps[-1] == pytest.approx(PERIOD, abs=SLACK), gaps


def test_a_sensor_slower_than_its_rate_ticks_as_often_as_its_work_allows() -> None:
    work = 1.4 * PERIOD
    gaps = np.diff(asyncio.run(_tick_times([work] * 6)))
    # The next tick comes once the work is done, not a further period after it.
    assert np.all((gaps >= work) & (gaps < work + PERIOD)), gaps


def test_ticks_wait_for_the_world_s_first_frame_and_are_wall_times() -> None:
    mirror, writer = WorldMirror(), WorldWriter()
    writer.add("world")

    async def first_tick() -> float:
        ticking = asyncio.ensure_future(anext(aiter(mirror.ticks(1 / PERIOD))))
        await asyncio.sleep(2 * PERIOD)
        assert not ticking.done(), "it ticked before the world had a frame"
        mirror.apply(writer.end_frame(0.0))
        return await ticking

    before = time.time()
    tick = asyncio.run(first_tick())
    assert before <= tick <= time.time()


def test_a_world_rebased_after_falling_behind_stamps_its_next_frame_with_wall_time() -> None:
    writer = WorldWriter()
    writer.add("world")
    writer.start_ns = time.time_ns() - round(60 * 1e9)
    writer.rebase(1.0)
    frame = writer.end_frame(1.0 + PERIOD)
    assert frame.ts / 1e9 == pytest.approx(time.time() + PERIOD, abs=SLACK)
