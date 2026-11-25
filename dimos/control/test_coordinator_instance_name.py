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

"""A coordinator subclass is told to serve its RPCs under the name clients look for."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from typing import Any

import pytest

import dimos.control.coordinator as coord_mod
from dimos.control.coordinator import ControlCoordinator
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState


class Subclassed(ControlCoordinator):
    extra_joints: Out[JointState]


@pytest.fixture
def make_coordinator() -> Iterator[Callable[..., ControlCoordinator]]:
    built: list[ControlCoordinator] = []

    def make(
        coordinator_cls: type[ControlCoordinator] = ControlCoordinator, **kwargs: Any
    ) -> ControlCoordinator:
        coordinator = coordinator_cls(**kwargs)
        built.append(coordinator)
        return coordinator

    try:
        yield make
    finally:
        for coordinator in built:
            coordinator.stop()


def test_subclass_without_instance_name_warns(make_coordinator, mocker):
    warn = mocker.patch.object(coord_mod.logger, "warning")
    make_coordinator(coordinator_cls=Subclassed).start()

    logged = " ".join(str(call) for call in warn.call_args_list)
    assert "instance_name" in logged
    assert "Subclassed" in logged


def test_subclass_with_instance_name_does_not_warn(make_coordinator, mocker):
    warn = mocker.patch.object(coord_mod.logger, "warning")
    make_coordinator(coordinator_cls=Subclassed, instance_name="ControlCoordinator").start()

    assert not any("instance_name" in str(call) for call in warn.call_args_list)


def test_plain_coordinator_does_not_warn(make_coordinator, mocker):
    warn = mocker.patch.object(coord_mod.logger, "warning")
    make_coordinator().start()

    assert not any("instance_name" in str(call) for call in warn.call_args_list)
