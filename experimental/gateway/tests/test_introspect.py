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
from pathlib import Path
import sys
from typing import Literal

import pytest

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from experimental.gateway.introspect import (
    Cache,
    IntrospectError,
    rpcs,
    run_child,
    spec_topic,
)


def test_spec_topic_reads_lcm_and_zenoh_specs() -> None:
    assert spec_topic(LCMTransport.spec("/cmd_vel", Twist)) == "/cmd_vel"
    assert (
        spec_topic(ZenohTransport.spec(ZenohTopic("dimos/head_left_color", Twist)))
        == "/head_left_color"
    )


class Dog(Module):
    @rpc
    def bark(self, times: int = 2) -> str:
        return "woof" * times

    @skill
    def trick(self, name: Literal["sit", "jump"]) -> str:
        """Do a trick."""
        return name


def test_a_modules_own_rpcs_are_listed_with_a_skill_flag() -> None:
    found = {record["name"]: record for record in rpcs(Dog)}
    assert set(found) == {"bark", "trick"}  # not every Module's start, stop, ...
    assert (
        not found["bark"]["skill"]
        and found["trick"]["skill"]
        and found["trick"]["doc"] == "Do a trick."
    )
    assert found["bark"]["params"] == [
        {"name": "times", "type": "int", "default": "2", "required": False, "choices": None}
    ]
    assert found["trick"]["params"][0]["required"] and found["trick"]["params"][0]["choices"] == [
        "sit",
        "jump",
    ]


def test_the_cache_shares_a_computation_and_drops_expired_ones() -> None:
    cache, computed = Cache(), []

    async def compute() -> int:
        computed.append(1)
        await asyncio.sleep(0.01)
        return len(computed)

    async def scenario() -> None:
        assert await asyncio.gather(*(cache.get("a", 60, compute) for _ in range(3))) == [1, 1, 1]
        await cache.get("b", 0, compute)
        await cache.get("c", 60, compute)

    asyncio.run(scenario())
    assert len(computed) == 3
    assert set(cache.entries) == {"a", "c"}


def test_another_checkout_is_introspected_with_this_file_and_its_own_dimos(tmp_path: Path) -> None:
    # a branch checkout with no gateway in it: its own dimos must be the one imported
    robot = tmp_path / "dimos" / "robot"
    robot.mkdir(parents=True)
    (tmp_path / "dimos" / "__init__.py").write_text("")
    (robot / "__init__.py").write_text("")
    (robot / "get_all_blueprints.py").write_text(
        "def get_by_name(name):\n    raise LookupError(f'{name} from the branch checkout')\n"
    )
    with pytest.raises(IntrospectError, match="r1pro-nav from the branch checkout"):
        asyncio.run(run_child(tmp_path, ["blueprint", "r1pro-nav"], python=sys.executable))
