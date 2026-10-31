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

from __future__ import annotations

import subprocess
import sys

import pytest

from dimos.core.coordination.blueprints import blueprint_factory
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.robot.get_all_blueprints import (
    OptionalDependencyError,
    get_by_name,
    get_by_name_or_exit,
    load_blueprint,
)
from dimos.robot.manipulators.xarm.blueprints import connection_teleop


def test_factory_discovery_does_not_import_connection_or_sdk():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import importlib.abc
import sys

class RejectConnection(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'xarm' or fullname.startswith('xarm.') or fullname in {
            'dimos.robot.manipulators.xarm.connection',
            'dimos.robot.manipulators.xarm.blueprints.teleop',
        }:
            raise AssertionError(f'Discovery imported {fullname}')

sys.meta_path.insert(0, RejectConnection())
from dimos.robot.manipulators.xarm.blueprints import connection_teleop
from dimos.core.coordination.blueprints import BlueprintFactory
from dimos.cli.commands.info import list_blueprints

assert isinstance(connection_teleop.xarm7_keyboard_teleop, BlueprintFactory)
assert isinstance(connection_teleop.xarm6_keyboard_teleop, BlueprintFactory)
list_blueprints()
""",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "  xarm7-keyboard-teleop\n" in result.stdout


def test_resolve_string_blueprint():
    bp = get_by_name("demo-mcp-stress-test")
    assert bp is not None


def test_resolve_string_module():
    bp = get_by_name("camera-module")
    assert bp is not None


def test_resolve_string_blueprint_or_exit():
    bp = get_by_name_or_exit("demo-mcp-stress-test")
    assert bp is not None


def test_resolve_string_module_or_exit():
    bp = get_by_name_or_exit("camera-module")
    assert bp is not None


def test_resolve_unknown_name():
    with pytest.raises(ValueError, match="Unknown blueprint or module"):
        get_by_name("nonexistent-blueprint-xyz")


def test_factory_resolves_only_selected_arm(monkeypatch):
    calls = []

    def compose(arm):
        calls.append(arm)
        return Module.blueprint(arm=arm)

    monkeypatch.setattr(connection_teleop, "_keyboard_teleop", compose)

    blueprint = get_by_name("xarm7-keyboard-teleop")

    assert calls == ["xarm7"]
    assert blueprint.blueprints[0].kwargs == {"arm": "xarm7"}


def test_factory_reads_current_configuration_on_each_resolution(monkeypatch):
    @blueprint_factory
    def selected():
        return Module.blueprint(simulation=global_config.simulation)

    monkeypatch.setattr(connection_teleop, "xarm7_keyboard_teleop", selected)
    monkeypatch.setattr(global_config, "simulation", "")
    first = get_by_name("xarm7-keyboard-teleop")
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    second = get_by_name("xarm7-keyboard-teleop")

    assert first.blueprints[0].kwargs == {"simulation": ""}
    assert second.blueprints[0].kwargs == {"simulation": "mujoco"}
    assert first is not second


def test_static_blueprint_target_is_preserved(monkeypatch):
    blueprint = Module.blueprint()
    monkeypatch.setattr(connection_teleop, "xarm7_keyboard_teleop", blueprint)

    assert get_by_name("xarm7-keyboard-teleop") is blueprint


def test_undecorated_function_is_not_executed(monkeypatch):
    def selected():
        pytest.fail("Undecorated functions must not execute")

    monkeypatch.setattr(connection_teleop, "xarm7_keyboard_teleop", selected)

    with pytest.raises(TypeError, match="decorated blueprint factory"):
        get_by_name("xarm7-keyboard-teleop")


def test_factory_must_return_blueprint(monkeypatch):
    @blueprint_factory
    def selected():
        return "invalid"

    monkeypatch.setattr(connection_teleop, "xarm7_keyboard_teleop", selected)

    with pytest.raises(TypeError, match="must return a Blueprint"):
        get_by_name("xarm7-keyboard-teleop")


def test_factory_reports_missing_optional_sdk(monkeypatch):
    def missing_sdk(arm):
        raise ModuleNotFoundError("No module named 'xarm'", name="xarm")

    monkeypatch.setattr(connection_teleop, "_keyboard_teleop", missing_sdk)

    with pytest.raises(OptionalDependencyError, match="xarm"):
        load_blueprint("xarm7-keyboard-teleop")
