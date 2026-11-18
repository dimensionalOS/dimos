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

"""The arm config contract's types cross processes and need no robot code to decode."""

from __future__ import annotations

import subprocess
import sys

from pydantic import BaseModel, ValidationError
import pytest

from dimos.control.connection.arm_config import (
    AppliedFile,
    ArmConfigState,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
    JointGains,
    LinkGravity,
)
from dimos.protocol.encode.cbor import encode
from dimos.utils.testing.cbor import roundtrip

PATCH = ConfigPatch(
    arm_id="rig/left",
    gains={"joint1": JointGains(kp=60.0, kd=3.0), "joint2": JointGains(kp=0.0, kd=0.5)},
    torque_offset_nm={"joint1": -0.2},
    links={"link2": LinkGravity(mass_kg=1.5, com_m=(0.1, 0.0, 0.0), merged_links=["tool"])},
    fit={"ridge_nm": 0.1, "holdout_poses": ["pose_05"]},
    friction_nm={"joint1": 0.3},
    checks={"float": {"passed": True}},
    notes="bench",
)

VALUES: list[BaseModel] = [
    PATCH,
    ConfigApplied(
        arm_id="rig/left",
        scope="persist",
        applied=PATCH,
        clamped=["gains.joint1.kp"],
        unsupported=["torque_offset_nm.joint1"],
        why={"torque_offset_nm.joint1": "session only"},
        path="/arms/rig/left/calibration.json",
        applied_now=False,
        refused="too far",
        gravity_step_nm={"joint1": 2.5},
    ),
    ArmConfigState(
        arm_id="rig/left",
        gains={"joint1": JointGains(kp=60.0, kd=3.0)},
        floating={"joint2": 0.5},
        torque_offset_nm={"joint1": -0.2},
        links={"link2": LinkGravity(mass_kg=1.5, com_m=(0.1, 0.0, 0.0))},
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
        arms={"rig/left": ["joint1", "joint2"]},
        readable=["gains", "links", "torque_offset_nm"],
        session=["gains", "float", "torque_offset_nm"],
        persist=["gains", "links"],
        persist_paths={"rig/left": "/arms/rig/left/calibration.json"},
        gravity_compensation=True,
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
print(state.gains["joint1"].kp, state.file.sha256[:4])
print(sorted(name for name in sys.modules if name.startswith("dimos.robot")))
"""


def test_the_types_decode_without_any_robot_code() -> None:
    state = VALUES[2]
    result = subprocess.run(
        [sys.executable, "-c", DECODE_WITHOUT_ROBOT_CODE],
        input=encode(state),
        capture_output=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stderr.decode()
    assert result.stdout.decode().splitlines() == [
        "dimos.control.connection.arm_config ArmConfigState",
        "60.0 aaaa",
        "[]",
    ]


def test_a_patch_names_every_value_it_gives() -> None:
    assert PATCH.keys() == [
        "gains.joint1.kp",
        "gains.joint1.kd",
        "gains.joint2.kp",
        "gains.joint2.kd",
        "torque_offset_nm.joint1",
        "links.link2",
        "friction_nm.joint1",
        "checks.float",
        "fit",
        "notes",
    ]
    assert ConfigPatch(arm_id="rig/left").keys() == []
    assert ConfigPatch(arm_id="rig/left", links={}, gains={}).keys() == ["gains", "links"]


@pytest.mark.parametrize(
    "fields",
    [
        {"gains": {"joint1": {"kp": -1.0, "kd": 0.0}}},
        {"gains": {"joint1": {"kp": 1.0}}},
        {"torque_offset_nm": {"joint1": float("nan")}},
        {"links": {"link2": {"mass_kg": 0.0, "com_m": (0.0, 0.0, 0.0)}}},
        {"friction_nm": {"joint1": -0.1}},
        {"stiffness": {}},
    ],
)
def test_a_patch_refuses_values_no_arm_could_take(fields: dict[str, object]) -> None:
    with pytest.raises(ValidationError):
        ConfigPatch.model_validate({"arm_id": "rig/left", **fields})
