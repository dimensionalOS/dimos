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

from dimos.deps.predicates import evaluate, fields, holds, simplify

CONFIG = {"simulation": "mujoco", "viewer": "rerun", "local_relay": False, "relay_url": None}


@pytest.mark.parametrize(
    ("predicate", "expected"),
    [
        (["eq", "simulation", "mujoco"], True),
        (["eq", "simulation", "dimsim"], False),
        (["ne", "viewer", "none"], True),
        (["in", "simulation", ["mujoco", "true"]], True),
        (["in", "simulation", ["dimsim"]], False),
        (["truthy", "local_relay"], False),
        (["truthy", "simulation"], True),
        (["not", ["truthy", "local_relay"]], True),
        (["any", ["truthy", "local_relay"], ["truthy", "relay_url"]], False),
        (["any", ["truthy", "local_relay"], ["eq", "viewer", "rerun"]], True),
        (["all", ["eq", "simulation", "mujoco"], ["eq", "viewer", "rerun"]], True),
        (["all", ["eq", "simulation", "mujoco"], ["truthy", "local_relay"]], False),
    ],
)
def test_evaluate(predicate: list[object], expected: bool) -> None:
    assert evaluate(predicate, CONFIG) is expected


def test_missing_field_is_unknown() -> None:
    assert evaluate(["eq", "robot_ip", "mujoco"], CONFIG) is None
    assert evaluate(["not", ["eq", "robot_ip", "mujoco"]], CONFIG) is None
    # any: a known True wins over unknown; all: a known False wins over unknown.
    assert evaluate(["any", ["eq", "robot_ip", "mujoco"], ["truthy", "simulation"]], CONFIG) is True
    assert (
        evaluate(["any", ["eq", "robot_ip", "mujoco"], ["truthy", "local_relay"]], CONFIG) is None
    )
    assert (
        evaluate(["all", ["eq", "robot_ip", "mujoco"], ["truthy", "local_relay"]], CONFIG) is False
    )
    assert evaluate(["all", ["eq", "robot_ip", "mujoco"], ["truthy", "simulation"]], CONFIG) is None


def test_unknown_operator_is_unknown() -> None:
    assert evaluate(["regex", "simulation", ".*"], CONFIG) is None
    assert evaluate([], CONFIG) is None


def test_holds_treats_unknown_as_true() -> None:
    assert holds(None, CONFIG)
    assert holds(["eq", "robot_ip", "mujoco"], CONFIG)
    assert holds(["truthy", "simulation"], CONFIG)
    assert not holds(["truthy", "local_relay"], CONFIG)


def test_simplify() -> None:
    nested = ["all", ["all", ["eq", "a", 1], ["eq", "b", 2]], ["not", ["not", ["eq", "c", 3]]]]
    assert simplify(nested) == ["all", ["eq", "a", 1], ["eq", "b", 2], ["eq", "c", 3]]
    assert simplify(["any", ["eq", "a", 1]]) == ["eq", "a", 1]
    assert simplify(simplify(nested)) == simplify(nested)


def test_fields() -> None:
    predicate = ["any", ["truthy", "local_relay"], ["all", ["eq", "a", 1], ["not", ["eq", "b", 2]]]]
    assert fields(predicate) == {"local_relay", "a", "b"}
