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

"""Tests for the opt-in arbitrated joint targets output stream."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import threading
from typing import Any

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.sensor_msgs.JointState import JointState


class _StubTask:
    def __init__(self, name: str, output: JointCommandOutput, priority: int = 0) -> None:
        self.name = name
        self._output = output
        self._priority = priority

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=frozenset(self._output.joint_names),
            priority=self._priority,
            mode=self._output.mode,
        )

    def is_active(self) -> bool:
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return self._output

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass


def _tick_once(tasks: dict[str, _StubTask]) -> list[JointState]:
    published: list[JointState] = []
    tick_loop = TickLoop(
        tick_rate=100.0,
        hardware={},
        hardware_lock=threading.Lock(),
        tasks=tasks,
        task_lock=threading.Lock(),
        joint_to_hardware={},
        publish_targets_callback=published.append,
    )
    tick_loop._tick()
    return published


@pytest.fixture
def make_coordinator() -> Iterator[Callable[..., ControlCoordinator]]:
    built: list[ControlCoordinator] = []

    def make(**kwargs: Any) -> ControlCoordinator:
        coordinator = ControlCoordinator(**kwargs)
        built.append(coordinator)
        return coordinator

    try:
        yield make
    finally:
        for coordinator in built:
            coordinator.stop()


class TestFlagOff:
    def test_default_is_off(self, make_coordinator):
        coordinator = make_coordinator()

        assert coordinator.config.publish_joint_targets is False

    def test_port_stays_silent_without_the_flag(self, make_coordinator, wait_until):
        coordinator = make_coordinator()
        merged: list[JointState] = []
        targets: list[JointState] = []
        coordinator.coordinator_joint_state.subscribe(merged.append)
        coordinator.coordinator_joint_targets.subscribe(targets.append)
        coordinator.start()

        wait_until(lambda: len(merged) > 2, timeout=5.0)
        assert not targets


class TestTickLoopMessages:
    def test_message_carries_the_commanded_joints_by_mode(self):
        arm = _StubTask(
            "arm",
            JointCommandOutput(
                joint_names=["arm/joint1", "arm/joint2"],
                positions=[0.5, 0.6],
                mode=ControlMode.SERVO_POSITION,
            ),
        )
        base = _StubTask(
            "base",
            JointCommandOutput(
                joint_names=["base/vx"],
                velocities=[0.25],
                mode=ControlMode.VELOCITY,
            ),
        )

        published = _tick_once({"arm": arm, "base": base})

        assert len(published) == 1
        msg = published[0]
        by_name = dict(zip(msg.name, zip(msg.position, msg.velocity, msg.effort, strict=False), strict=False))
        assert set(by_name) == {"arm/joint1", "arm/joint2", "base/vx"}
        assert by_name["arm/joint1"] == (0.5, 0.0, 0.0)
        assert by_name["arm/joint2"] == (0.6, 0.0, 0.0)
        assert by_name["base/vx"] == (0.0, 0.25, 0.0)

    def test_arbitration_winner_is_what_gets_published(self):
        low = _StubTask(
            "low",
            JointCommandOutput(
                joint_names=["arm/joint1"], positions=[1.0], mode=ControlMode.SERVO_POSITION
            ),
            priority=0,
        )
        high = _StubTask(
            "high",
            JointCommandOutput(
                joint_names=["arm/joint1"], positions=[2.0], mode=ControlMode.SERVO_POSITION
            ),
            priority=10,
        )

        published = _tick_once({"low": low, "high": high})

        msg = published[0]
        assert list(msg.name) == ["arm/joint1"]
        assert list(msg.position) == [2.0]

    def test_nothing_is_published_when_no_task_commands(self):
        assert _tick_once({}) == []
