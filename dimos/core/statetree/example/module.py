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

"""A simulated delivery robot whose behavior is a StateTree run by its Module."""

from collections.abc import AsyncIterator
from dataclasses import dataclass

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.statetree.component import StateTreeComponent
from dimos.core.statetree.runtime import (
    State,
    StateTree,
    Status,
    Task,
    Transition,
    Trigger,
)
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist


class Drive(Task["DeliveryModule"]):
    def enter(self, ctx: "DeliveryModule") -> Status:
        ctx.command_speed(ctx.config.speed)
        ctx.trace.append("drive.enter")
        return Status.RUNNING

    def tick(self, ctx: "DeliveryModule", dt: float) -> Status:
        ctx.distance += ctx.commanded_speed * dt
        return Status.SUCCEEDED if ctx.distance >= ctx.config.route_length else Status.RUNNING

    def exit(self, ctx: "DeliveryModule") -> None:
        ctx.command_speed(0.0)
        ctx.trace.append("drive.exit")


class WaitForClearance(Task["DeliveryModule"]):
    def __init__(self) -> None:
        self.elapsed = 0.0

    def enter(self, ctx: "DeliveryModule") -> Status:
        ctx.trace.append("wait.enter")
        return Status.RUNNING

    def tick(self, ctx: "DeliveryModule", dt: float) -> Status:
        self.elapsed += dt
        return Status.SUCCEEDED if self.elapsed >= ctx.config.clearance_wait else Status.RUNNING

    def exit(self, ctx: "DeliveryModule") -> None:
        ctx.trace.append("wait.exit")


def delivery_tree() -> StateTree["DeliveryModule"]:
    return StateTree(
        State(
            "delivery",
            children=(
                State(
                    "idle",
                    tasks=(Task,),
                    transitions=(Transition(Trigger.EVENT, "drive", event="dispatch"),),
                ),
                State(
                    "drive",
                    tasks=(Drive,),
                    transitions=(
                        Transition(Trigger.EVENT, "wait", event="obstacle"),
                        Transition(Trigger.SUCCESS, Status.SUCCEEDED),
                    ),
                ),
                State(
                    "wait",
                    tasks=(WaitForClearance,),
                    transitions=(Transition(Trigger.SUCCESS, "drive"),),
                ),
            ),
        )
    )


class DeliveryConfig(ModuleConfig):
    route_length: float = 2.0
    """Meters to cover before the delivery succeeds."""

    speed: float = 1.0
    """Forward speed in m/s while driving."""

    clearance_wait: float = 1.0
    """Seconds to stand still after an obstacle is reported."""

    tick_rate: float = 20.0
    """StateTree ticks per second."""


@dataclass(frozen=True)
class DeliveryState:
    status: Status | None
    path: tuple[str, ...]
    distance: float
    commanded_speed: float
    trace: tuple[str, ...]


class DeliveryModule(Module):
    """Waits for dispatch, drives the route, and pauses when an obstacle is reported.

    The tree's tasks receive this module as their context. They run on the module
    loop, as do the async RPCs, so the fields below need no locks.
    """

    config: DeliveryConfig
    cmd_vel: Out[Twist]

    distance: float
    commanded_speed: float
    trace: list[str]
    _behavior: StateTreeComponent["DeliveryModule"]

    async def main(self) -> AsyncIterator[None]:
        self.distance = 0.0
        self.commanded_speed = 0.0
        self.trace = []
        async with StateTreeComponent(
            delivery_tree(), self, tick_rate=self.config.tick_rate
        ) as self._behavior:
            yield

    def command_speed(self, speed: float) -> None:
        self.commanded_speed = speed
        self.cmd_vel.publish(Twist(linear=[speed, 0.0, 0.0]))

    @rpc
    async def dispatch(self) -> None:
        self._behavior.send_event("dispatch")

    @rpc
    async def report_obstacle(self) -> None:
        self._behavior.send_event("obstacle")

    @rpc
    async def get_state(self) -> DeliveryState:
        return DeliveryState(
            status=self._behavior.status,
            path=self._behavior.active_path,
            distance=self.distance,
            commanded_speed=self.commanded_speed,
            trace=tuple(self.trace),
        )
