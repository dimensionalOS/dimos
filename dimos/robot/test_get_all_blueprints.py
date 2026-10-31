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

from dimos.robot.get_all_blueprints import get_by_name, get_by_name_or_exit


def test_list_and_legacy_teleop_do_not_import_optional_connection():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import importlib.abc
import sys

class RejectXArm(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'xarm' or fullname.startswith('xarm.') or fullname in {
            'dimos.robot.manipulators.xarm.connection',
            'dimos.robot.manipulators.xarm.blueprints.connection_teleop',
        }:
            raise AssertionError(f'Imported optional connection dependency {fullname}')

sys.meta_path.insert(0, RejectXArm())
from dimos.cli.commands.info import list_blueprints
from dimos.robot.get_all_blueprints import get_by_name

list_blueprints()
assert 'dimos.robot.manipulators.xarm.blueprints.teleop' not in sys.modules
blueprint = get_by_name('keyboard-teleop-xarm7')
assert [atom.module.__name__ for atom in blueprint.blueprints] == [
    'KeyboardTeleopModule', 'ArmTwistCoordinator', 'ManipulationModule',
]
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
