# Copyright 2025 Dimensional Inc.
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
from collections.abc import Generator
import datetime
import time

import pytest

from dimos.core import Module, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.skill.coordinator import SkillCoordinator
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Output, Reducer, Stream
from dimos.utils.data import get_data


class SkillContainerTest(Module):
    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill()
    def add(self, x: int, y: int) -> int:
        """adds x and y."""
        time.sleep(2)
        return x + y

    @skill()
    def delayadd(self, x: int, y: int) -> int:
        """waits 0.3 seconds before adding x and y."""
        time.sleep(0.3)
        return x + y

    @skill(stream=Stream.call_agent, reducer=Reducer.all)  # type: ignore[arg-type]
    def counter(self, count_to: int, delay: float | None = 0.05) -> Generator[int, None, None]:
        """Counts from 1 to count_to, with an optional delay between counts."""
        for i in range(1, count_to + 1):
            if delay is not None and delay > 0:
                time.sleep(delay)
            yield i

    @skill(stream=Stream.passive, reducer=Reducer.sum)  # type: ignore[arg-type]
    def counter_passive_sum(
        self, count_to: int, delay: float | None = 0.05
    ) -> Generator[int, None, None]:
        """Counts from 1 to count_to, with an optional delay between counts."""
        for i in range(1, count_to + 1):
            if delay is not None and delay > 0:
                time.sleep(delay)
            yield i

    @skill(stream=Stream.passive, reducer=Reducer.latest)  # type: ignore[arg-type]
    def current_time(self, frequency: float | None = 10) -> Generator[str, None, None]:
        """Provides current time."""
        while True:
            yield str(datetime.datetime.now())
            if frequency is not None:
                time.sleep(1 / frequency)

    @skill(stream=Stream.passive, reducer=Reducer.latest)  # type: ignore[arg-type]
    def uptime_seconds(self, frequency: float | None = 10) -> Generator[float, None, None]:
        """Provides current uptime."""
        start_time = datetime.datetime.now()
        while True:
            yield (datetime.datetime.now() - start_time).total_seconds()
            if frequency is not None:
                time.sleep(1 / frequency)

    @skill()
    def current_date(self, frequency: float | None = 10) -> str:
        """Provides current date."""
        return str(datetime.datetime.now())

    @skill(output=Output.image)
    def take_photo(self) -> Image:  # type: ignore[type-arg]
        """Takes a camera photo"""
        print("Taking photo...")
        img = Image.from_file(str(get_data("cafe-smol.jpg")))  # type: ignore[arg-type]
        print("Photo taken.")
        return img  # type: ignore[return-value]


@pytest.mark.asyncio
async def test_coordinator_parallel_calls() -> None:
    container = SkillContainerTest()
    skillCoordinator = SkillCoordinator()
    skillCoordinator.register_skills(container)

    skillCoordinator.start()
    skillCoordinator.call_skill("test-call-0", "add", {"args": [0, 2]})

    time.sleep(0.1)

    cnt = 0
    while await skillCoordinator.wait_for_updates(1):
        print(skillCoordinator)

        skillstates = skillCoordinator.generate_snapshot()

        skill_id = f"test-call-{cnt}"
        tool_msg = skillstates[skill_id].agent_encode()
        assert tool_msg.content == cnt + 2  # type: ignore[union-attr]

        cnt += 1
        if cnt < 5:
            skillCoordinator.call_skill(
                f"test-call-{cnt}-delay",
                "delayadd",
                {"args": [cnt, 2]},
            )
            skillCoordinator.call_skill(
                f"test-call-{cnt}",
                "add",
                {"args": [cnt, 2]},
            )

        await asyncio.sleep(0.1 * cnt)

    container.stop()
    skillCoordinator.stop()


@pytest.mark.asyncio
async def test_coordinator_generator() -> None:
    container = SkillContainerTest()
    skillCoordinator = SkillCoordinator()
    skillCoordinator.register_skills(container)
    skillCoordinator.start()

    # here we call a skill that generates a sequence of messages
    skillCoordinator.call_skill("test-gen-0", "counter", {"args": [10]})
    skillCoordinator.call_skill("test-gen-1", "counter_passive_sum", {"args": [5]})
    skillCoordinator.call_skill("test-gen-2", "take_photo", {"args": []})

    # periodically agent is stopping it's thinking cycle and asks for updates
    while await skillCoordinator.wait_for_updates(2):
        print(skillCoordinator)
        agent_update = skillCoordinator.generate_snapshot(clear=True)
        print(agent_update)
        await asyncio.sleep(0.125)

    print("coordinator loop finished")
    print(skillCoordinator)
    container.stop()
    skillCoordinator.stop()
