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

import curses

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.terminal import TerminalTeleopState


def test_movement_is_ignored_until_operator_engages() -> None:
    state = TerminalTeleopState()

    state.handle_key(ord("w"), now=1.0)

    assert state.next_command(now=1.0) is None
    assert state.engaged is False


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        (ord("w"), Twist(linear=Vector3(0.2, 0.0, 0.0), angular=Vector3())),
        (curses.KEY_DOWN, Twist(linear=Vector3(-0.2, 0.0, 0.0), angular=Vector3())),
        (ord("a"), Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, 0.4))),
        (curses.KEY_RIGHT, Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, -0.4))),
        (ord("q"), Twist(linear=Vector3(0.0, 0.2, 0.0), angular=Vector3())),
        (ord("e"), Twist(linear=Vector3(0.0, -0.2, 0.0), angular=Vector3())),
    ],
)
def test_engaged_movement_keys_publish_expected_robot_frame_twist(
    key: int, expected: Twist
) -> None:
    state = TerminalTeleopState(linear_speed=0.2, angular_speed=0.4)
    state.handle_key(10, now=1.0)

    state.handle_key(key, now=1.1)

    assert state.next_command(now=1.1) == expected


def test_command_lease_expires_to_one_stop_message() -> None:
    state = TerminalTeleopState(deadman_timeout=0.2)
    state.handle_key(10, now=1.0)
    state.handle_key(ord("w"), now=1.0)

    assert state.next_command(now=1.19) is not None
    assert state.next_command(now=1.21) == Twist()
    assert state.next_command(now=1.22) is None


def test_space_stops_and_requires_reengagement() -> None:
    state = TerminalTeleopState()
    state.handle_key(10, now=1.0)
    state.handle_key(ord("w"), now=1.0)

    state.handle_key(ord(" "), now=1.1)

    assert state.next_command(now=1.1) == Twist()
    assert state.engaged is False
    state.handle_key(ord("w"), now=1.2)
    assert state.next_command(now=1.2) is None


def test_escape_requests_quit_and_stop() -> None:
    state = TerminalTeleopState()
    state.handle_key(10, now=1.0)
    state.handle_key(ord("w"), now=1.0)

    state.handle_key(27, now=1.1)

    assert state.should_quit is True
    assert state.next_command(now=1.1) == Twist()
