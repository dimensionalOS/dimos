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

"""Behavioral test for the uniform ``(msg, t_now)`` teleop_buttons handler."""

from __future__ import annotations

from dimos.control.tasks.teleop_task.teleop_task import TeleopIKTask


def test_on_teleop_buttons_delegates_to_on_buttons(mocker) -> None:
    mock_self = mocker.Mock()
    mock_self.on_buttons.return_value = True
    sentinel = object()
    assert TeleopIKTask.on_teleop_buttons(mock_self, sentinel, 0.0) is True
    mock_self.on_buttons.assert_called_once_with(sentinel)
