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

import sys

import pytest

from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, MetaBlueprint, autoconnect
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.get_all_blueprints import get_by_name, get_by_name_or_exit


def test_resolve_string_blueprint():
    bp = get_by_name("demo-mcp-stress-test")
    assert bp is not None


def test_resolve_string_module():
    bp = get_by_name("webcam-module")
    assert bp is not None


def test_resolve_string_blueprint_or_exit():
    bp = get_by_name_or_exit("demo-mcp-stress-test")
    assert bp is not None


def test_resolve_string_module_or_exit():
    bp = get_by_name_or_exit("webcam-module")
    assert bp is not None


@pytest.mark.parametrize("lookup", [get_by_name, get_by_name_or_exit])
def test_exported_composition_stays_unresolved(lookup) -> None:
    blueprint = lookup("demo-capabilities")

    assert isinstance(blueprint, MetaBlueprint)
    assert {atom.module.__name__ for atom in blueprint.solidify().blueprints} == {
        "DemoSensors",
        "DemoRobotActions",
        "DemoMonitoring",
        "McpServer",
        "Agent",
    }


def test_resolve_unknown_name():
    with pytest.raises(ValueError, match="Unknown blueprint or module"):
        get_by_name("nonexistent-blueprint-xyz")


@pytest.mark.parametrize("lookup", [get_by_name, get_by_name_or_exit])
def test_registry_preserves_unresolved_choices(monkeypatch: pytest.MonkeyPatch, lookup) -> None:
    mapping = autoconnect(ChoiceAtom(name="Required", choices={"empty": Blueprint(blueprints=())}))
    monkeypatch.setattr(sys.modules[__name__], "mapping", mapping, raising=False)
    monkeypatch.setitem(all_blueprints, "test-mapping", f"{__name__}:mapping")

    assert lookup("test-mapping") is mapping
