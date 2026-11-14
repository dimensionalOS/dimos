# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any

from dimos.msgs.std_msgs.Bool import Bool
from dimos.robot.manipulators.openarm.mini_connection import OpenArmMiniArmConnection


def test_follow_off_holds_the_follower_until_follow_is_on_again(mocker: Any) -> None:
    leaders = OpenArmMiniArmConnection(mock_positions={})
    joints = mocker.patch.object(leaders.joint_state, "publish")
    reading = {"right": {f"joint_{index}": 0.0 for index in range(1, 8)}}
    try:
        leaders._on_follow(Bool(data=False))
        leaders.publish_reading(reading)
        sent_while_off = [call.args[0] for call in joints.call_args_list]
        leaders._on_follow(Bool(data=True))
        leaders.publish_reading(reading)
    finally:
        leaders.stop()

    # While off: one hold for all 14 arm joints (zero speed, no positions), no targets.
    [hold] = sent_while_off
    assert len(hold.name) == 14
    assert hold.position == []
    assert hold.velocity == [0.0] * 14
    # On again: the right leader's targets flow.
    assert len(joints.call_args.args[0].position) == 7
