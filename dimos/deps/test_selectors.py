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

from dimos.deps.selectors import SelectorInput, collect_selector_inputs, identifiers

FIELDS = frozenset({"hardware", "tasks"})


def test_cli_tokens_in_every_form() -> None:
    tokens = [
        "--controlcoordinator.hardware",
        '[{"adapter_type": "xarm"}]',
        "--modules.ControlCoordinator.tasks=[]",
        "--hardware",
        "--camera.fps",
        "30",
        "--hardware.nested.deep",
        "x",
        "--tick-rate=5",
        "--tasks",
    ]
    assert collect_selector_inputs(tokens, {}, {}, FIELDS) == (
        SelectorInput(
            "--controlcoordinator.hardware",
            "controlcoordinator",
            "hardware",
            '[{"adapter_type": "xarm"}]',
        ),
        SelectorInput("--modules.ControlCoordinator.tasks", "controlcoordinator", "tasks", "[]"),
        SelectorInput("--hardware", None, "hardware", None),
        SelectorInput("--tasks", None, "tasks", None),
    )


def test_environment_and_config_file_sections() -> None:
    environ = {
        "CONTROLCOORDINATOR__HARDWARE": "[]",
        "G__HARDWARE": "x",
        "TRANSPORTS__A__HARDWARE": "x",
        "CONTROLCOORDINATOR__TICK_RATE": "5",
        "HARDWARE": "x",
    }
    sections = {
        "g": {"simulation": "mujoco"},
        "transports": {"hardware": "x"},
        "ControlCoordinator": {"tasks": [{"type": "trajectory"}], "tick_rate": 5},
    }
    assert collect_selector_inputs([], sections, environ, FIELDS) == (
        SelectorInput(
            "config file section 'ControlCoordinator'",
            "controlcoordinator",
            "tasks",
            [{"type": "trajectory"}],
        ),
        SelectorInput("CONTROLCOORDINATOR__HARDWARE", "controlcoordinator", "hardware", "[]"),
    )


def test_identifiers() -> None:
    assert identifiers("adapter", '[{"adapter_type": "XArm"}, {}]') == ["xarm", "mock"]
    assert identifiers("task", [{"type": "trajectory"}]) == ["trajectory"]
    assert identifiers("connection", "MuJoCo") == ["mujoco"]
    with pytest.raises(ValueError, match="expected a JSON list"):
        identifiers("adapter", "xarm")
    with pytest.raises(ValueError, match="is not valid JSON"):
        identifiers("adapter", "[oops")
    with pytest.raises(ValueError, match="expected a list of objects"):
        identifiers("adapter", '["xarm"]')
    with pytest.raises(ValueError, match="an item has no 'type'"):
        identifiers("task", [{}])
    with pytest.raises(ValueError, match="expected a connection name"):
        identifiers("connection", "")
