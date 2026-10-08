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

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_joystick_record import (
    record_rerun_config,
    unitree_go2_joystick_record,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule


def test_only_raw_inputs_and_display() -> None:
    modules = sorted(a.module.__name__ for a in unitree_go2_joystick_record.active_blueprints)
    assert modules == ["GO2Connection", "KeyboardTeleop", "RerunBridgeModule"]


def test_viewer_uses_the_record_window_with_a_fixed_memory_limit() -> None:
    bridges = [
        a for a in unitree_go2_joystick_record.active_blueprints if a.module is RerunBridgeModule
    ]
    assert len(bridges) == 1
    assert bridges[0].kwargs["blueprint"] is record_rerun_config["blueprint"]
    assert bridges[0].kwargs["memory_limit"] == "2GB"


def test_the_window_has_a_plot_per_command_stream() -> None:
    column = record_rerun_config["blueprint"]().root_container.contents[0]
    assert [(v.name, str(v.origin)) for v in column.contents] == [
        ("Camera", "world/color_image"),
        ("odom", "plots/odom"),
        ("cmd_vel", "plots/cmd_vel"),
    ]


def test_converters_plot_forward_strafe_and_turn() -> None:
    overrides = record_rerun_config["visual_override"]
    cmd_vel = overrides["world/cmd_vel"](Twist(linear=[0.5, 0.1, 0.0], angular=[0.0, 0.0, 0.8]))
    assert [path for path, _ in cmd_vel] == ["plots/cmd_vel"]
    for _, scalars in cmd_vel:
        assert scalars.scalars.as_arrow_array().to_pylist() == pytest.approx([0.5, 0.1, 0.8])
