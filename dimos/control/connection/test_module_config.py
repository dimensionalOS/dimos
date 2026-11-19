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

"""The config contract's types cross processes and need no robot code to decode."""

from __future__ import annotations

import subprocess
import sys
from typing import Any

from pydantic import BaseModel, ValidationError
import pytest

from dimos.control.connection.module_config import (
    AppliedFile,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
    ConfigState,
)
from dimos.protocol.encode.cbor import encode
from dimos.utils.testing.cbor import roundtrip

PATCH = ConfigPatch(
    target="rig/left",
    values={
        "gains": {"joint1": {"kp": 60.0, "kd": 3.0}, "joint2": {}},
        "offsets": [0.1, -0.2],
        "notes": "bench",
    },
)

VALUES: list[BaseModel] = [
    PATCH,
    ConfigApplied(
        target="rig/left",
        scope="persist",
        applied={"gains": {"joint1": {"kp": 50.0, "kd": 3.0}}},
        clamped=["gains.joint1.kp"],
        unsupported=["notes"],
        why={"notes": "not saved"},
        path="/arms/rig/left/calibration.json",
        applied_now=False,
        refused="too far",
        details={"step": {"joint1": 2.5}},
    ),
    ConfigState(
        target="rig/left",
        values={"gains": {"joint1": {"kp": 60.0, "kd": 3.0}}},
        file=AppliedFile(
            path="/arms/rig/left/calibration.json",
            created="2026-11-18T09:30:00+00:00",
            sha256="a" * 64,
        ),
        session_id="s" * 32,
    ),
    ConfigKeys(
        source="arm",
        sessions=True,
        targets={"rig/left": ["joint1", "joint2"]},
        readable=["gains"],
        session=["gains"],
        persist=["gains", "notes"],
        persist_paths={"rig/left": "/arms/rig/left/calibration.json"},
        features={"holds_itself_up": True},
    ),
]


@pytest.mark.parametrize("value", VALUES, ids=lambda value: type(value).__name__)
def test_every_type_round_trips_as_an_rpc_carries_it(value: BaseModel) -> None:
    back = roundtrip(value)
    assert type(back) is type(value) and back == value


# Decode stdin where no robot code, motor SDK or pinocchio can be imported.
DECODE_WITHOUT_ROBOT_CODE = """
import sys
sys.modules["can_motor_control"] = None
sys.modules["pinocchio"] = None
from dimos.protocol.encode.cbor import decode
state = decode(sys.stdin.buffer.read())
print(type(state).__module__, type(state).__name__)
print(state.values["gains"]["joint1"]["kp"], state.file.sha256[:4])
robot_code = ("dimos.robot", "dimos.control.calibration", "dimos.control.connection.connection_")
print(sorted(name for name in sys.modules if name.startswith(robot_code)))
"""


def test_the_types_decode_without_any_robot_code() -> None:
    result = subprocess.run(
        [sys.executable, "-c", DECODE_WITHOUT_ROBOT_CODE],
        input=encode(VALUES[2]),
        capture_output=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stderr.decode()
    assert result.stdout.decode().splitlines() == [
        "dimos.control.connection.module_config ConfigState",
        "60.0 aaaa",
        "[]",
    ]


def test_a_patch_names_every_value_it_gives() -> None:
    assert PATCH.keys() == [
        "gains.joint1.kp",
        "gains.joint1.kd",
        "gains.joint2",
        "offsets",
        "notes",
    ]
    assert ConfigPatch(target="rig/left").keys() == []
    assert ConfigPatch(target="rig/left", values={"gains": {}}).keys() == ["gains"]


@pytest.mark.parametrize(
    "data",
    [
        {"values": {}},
        {"target": 1, "values": {}},
        {"target": "rig/left", "values": [("gains", {})]},
        {"target": "rig/left", "values": {}, "scope": "session"},
    ],
)
def test_a_patch_checks_its_shape_and_leaves_the_values_to_the_driver(data: Any) -> None:
    with pytest.raises(ValidationError):
        ConfigPatch.model_validate(data, strict=True)
    anything = {"gains": {"joint1": {"kp": "80"}}, "links": [None]}
    assert ConfigPatch.model_validate({"target": "rig/left", "values": anything}).values == anything
