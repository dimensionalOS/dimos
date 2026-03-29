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

"""Tests for the dimos Python API (dimos.connect / coordinator.module)."""

from __future__ import annotations

import pytest

from dimos.api import connect, list_blueprints


@pytest.fixture
def robot():
    """Launch stress-test blueprint and yield coordinator, stop on teardown."""
    coordinator = connect("demo-mcp-stress-test")
    yield coordinator
    coordinator.stop()


def test_list_blueprints():
    names = list_blueprints()
    assert "unitree-go2" in names
    assert "demo-mcp-stress-test" in names
    assert len(names) > 10


def test_connect_returns_coordinator(robot):
    assert robot.n_modules > 0
    assert robot.n_modules > 0


def test_module_by_name(robot):
    assert "StressTestModule" in robot.module_names
    stress = robot.module("StressTestModule")
    assert stress is not None


def test_module_not_found(robot):
    with pytest.raises(KeyError, match="No module"):
        robot.module("NonExistentModule")


def test_module_rpc_echo(robot):
    stress = robot.module("StressTestModule")
    assert stress.echo("hello") == "hello"
    assert stress.echo("world") == "world"


def test_module_rpc_ping(robot):
    stress = robot.module("StressTestModule")
    assert stress.ping() == "pong"


def test_skill_discovery(robot):
    stress = robot.module("StressTestModule")
    skills = stress.get_skills()
    names = [s.func_name for s in skills]
    assert "echo" in names
    assert "ping" in names
    assert "slow" in names
    assert "info" in names


def test_module_names_lists_all(robot):
    names = robot.module_names
    assert "StressTestModule" in names
    assert isinstance(names, list)
