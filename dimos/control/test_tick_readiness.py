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

import threading

from pytest_mock import MockerFixture

from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.spec import ControlMode


def _tick_loop(hardware: dict) -> TickLoop:  # type: ignore[type-arg]
    return TickLoop(
        tick_rate=100.0,
        hardware=hardware,
        hardware_lock=threading.Lock(),
        tasks={},
        task_lock=threading.Lock(),
        joint_to_hardware={},
    )


def test_unready_hardware_is_not_read_or_published(mocker: MockerFixture) -> None:
    hardware = mocker.Mock()
    hardware.is_ready.return_value = False
    published = mocker.Mock()
    tick_loop = TickLoop(
        tick_rate=100.0,
        hardware={"robot": hardware},
        hardware_lock=threading.Lock(),
        tasks={},
        task_lock=threading.Lock(),
        joint_to_hardware={},
        publish_robot_callback=published,
    )

    tick_loop._tick()

    hardware.read_state.assert_not_called()
    published.assert_not_called()


def test_unready_hardware_is_not_commanded(mocker: MockerFixture) -> None:
    hardware = mocker.Mock()
    hardware.is_ready.return_value = False
    tick_loop = _tick_loop({"robot": hardware})

    tick_loop._write_all_hardware({"robot": ({"robot/joint1": 0.5}, ControlMode.POSITION)})

    hardware.write_command.assert_not_called()
