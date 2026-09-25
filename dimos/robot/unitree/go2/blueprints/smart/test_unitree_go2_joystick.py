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

from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_joystick import (
    Replay,
    joystick_rerun_config,
    unitree_go2_joystick,
    unitree_go2_joystick_replay,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _bridge_kwargs(blueprint: Blueprint) -> dict[str, object]:
    bridges = [a for a in blueprint.active_blueprints if a.module is RerunBridgeModule]
    assert len(bridges) == 1
    return dict(bridges[0].kwargs)


@pytest.mark.parametrize("blueprint", [unitree_go2_joystick, unitree_go2_joystick_replay])
def test_blueprint_uses_the_joystick_viewer_window(blueprint: Blueprint) -> None:
    kwargs = _bridge_kwargs(blueprint)
    assert kwargs["blueprint"] is joystick_rerun_config["blueprint"]
    assert kwargs["visual_override"] is joystick_rerun_config["visual_override"]


def test_the_default_go2_viewer_window_is_unchanged() -> None:
    assert _bridge_kwargs(unitree_go2)["blueprint"] is rerun_config["blueprint"]
    assert "world/joystick" not in rerun_config["visual_override"]


def test_the_window_has_a_plot_per_command_stream() -> None:
    column = joystick_rerun_config["blueprint"]().root_container.contents[0]
    assert [(v.name, str(v.origin)) for v in column.contents] == [
        ("Camera", "world/color_image"),
        ("odom", "plots/odom"),
        ("joystick", "plots/joystick"),
        ("tele_cmd_vel", "plots/tele_cmd_vel"),
        ("cmd_vel", "plots/cmd_vel"),
    ]


def test_converters_plot_forward_strafe_and_turn() -> None:
    overrides = joystick_rerun_config["visual_override"]
    joystick = overrides["world/joystick"](Joy(axes=[0.5, 0.1, 0.0, 0.0, 0.0, 0.8]))
    cmd_vel = overrides["world/cmd_vel"](Twist(linear=[0.5, 0.1, 0.0], angular=[0.0, 0.0, 0.8]))
    assert [path for path, _ in joystick + cmd_vel] == ["plots/joystick", "plots/cmd_vel"]
    for _, scalars in joystick + cmd_vel:
        assert scalars.scalars.as_arrow_array().to_pylist() == pytest.approx([0.5, 0.1, 0.8])


def test_importing_the_blueprints_opens_no_recording() -> None:
    assert Replay.__annotations__ == {}
    assert Replay.stream_types == {}
