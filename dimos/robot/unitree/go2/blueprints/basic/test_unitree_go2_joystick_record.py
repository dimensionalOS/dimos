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

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_joystick_record import (
    record_rerun_config,
)


def test_keys_text_logs_each_change_once() -> None:
    keys = record_rerun_config["visual_override"]["world/joystick"]
    w_shift = Joy(buttons=[1, 0, 0, 0, 0, 0, 1, 0, 0])
    lines = [keys(w_shift), keys(w_shift), keys(Joy(buttons=[0] * 9))]
    assert [
        [entry.text.as_arrow_array().to_pylist()[0] for _, entry in line] for line in lines
    ] == [
        ["W + SHIFT"],
        [],
        ["(released)"],
    ]
