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

import numpy as np
import pytest

from dimos.sim2.world_observations import WorldMirror, WorldWriter

PERIOD = 0.13
FRAME = 0.005
EPOCH = 1_700_000_000.0


async def _tick_times(work: Sequence[float]) -> list[float]:
    """When a sensor ticks whose work on each tick takes ``work[i]`` sim seconds, in a world on epoch time."""
    mirror, writer = WorldMirror(), WorldWriter()
    writer.start_ns = round(EPOCH * 1e9)
    writer.add("world")

    async def run_the_world() -> None:
        frame = 0
        while True:
            mirror.apply(writer.end_frame(frame * FRAME))
            frame += 1
            # Lets the sensor run between frames; no time passes.
            await asyncio.sleep(0)

    world = asyncio.create_task(run_the_world())
    ticks: list[float] = []
    async for tick in mirror.ticks(1 / PERIOD):
        ticks.append(tick)
        if len(ticks) > len(work):
            break
        await mirror.reached(tick + work[len(ticks) - 1])
    world.cancel()
    return ticks


@pytest.mark.parametrize(
    "work",
    [[0.0] * 8, [1.5 * PERIOD] + [0.0] * 7, [1.9 * PERIOD] + [0.0] * 7, [3.0 * PERIOD] + [0.0] * 7],
    ids=["idle", "one_slow_tick", "one_nearly_two_periods", "one_very_slow_tick"],
)
def test_a_sensor_never_ticks_within_a_period_of_its_last_tick(work: list[float]) -> None:
    gaps = np.diff(asyncio.run(_tick_times(work)))
    assert gaps.min() >= PERIOD - FRAME, gaps
    # Once caught up it is back on its rate, not drifting slower.
    assert gaps[-1] == pytest.approx(PERIOD, abs=FRAME), gaps


def test_a_sensor_slower_than_its_rate_ticks_as_often_as_its_work_allows() -> None:
    work = 1.4 * PERIOD
    gaps = np.diff(asyncio.run(_tick_times([work] * 6)))
    # The next tick comes once the work is done, not a further period after it.
    assert np.all((gaps >= work) & (gaps < work + PERIOD)), gaps
