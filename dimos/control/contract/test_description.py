# Copyright 2025-2026 Dimensional Inc.
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

"""The description's five fields, its derived views, and that it survives RPC."""

from __future__ import annotations

import dataclasses
import pickle

import pytest

from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import Interface, Unit


def test_a_description_is_exactly_five_fields() -> None:
    """Name, parts, limits, report rate and deadman timeout, and nothing else."""
    assert [f.name for f in dataclasses.fields(ControlDescription)] == [
        "source",
        "resources",
        "limits",
        "state_rate_hz",
        "deadman_timeout_s",
    ]


def test_limits_refuse_unless_told_to_clamp() -> None:
    """Clamping is something a description asks for, never what it gets by default."""
    assert Limits(-1.0, 1.0).clamp is False


def test_state_and_command_keys_follow_declaration_order(xarm: ControlDescription) -> None:
    """Order is resources in order, then interfaces in order -- not sorted."""
    assert xarm.state_keys()[:4] == (
        "arm/joint1/position",
        "arm/joint1/effort",
        "arm/joint2/position",
        "arm/joint2/effort",
    )
    assert xarm.command_keys()[:2] == ("arm/joint1/position", "arm/joint1/velocity")


def test_state_and_command_sets_may_differ(xarm: ControlDescription) -> None:
    """The xArm commands velocity but never reports it, and that is declarable."""
    assert "arm/joint1/velocity" in xarm.command_keys()
    assert "arm/joint1/velocity" not in xarm.state_keys()
    assert "arm/joint1/effort" in xarm.state_keys()
    assert "arm/joint1/effort" not in xarm.command_keys()


def test_joint_names_cover_joints_and_grippers(xarm: ControlDescription) -> None:
    """A gripper is claimed like a joint; it has a position a task drives."""
    names = xarm.joint_names()

    assert names[0] == "arm/joint1"
    assert "arm/gripper" in names
    assert len(names) == 8


def test_joint_names_exclude_bases(chassis: ControlDescription) -> None:
    """A base is claimed as a resource with vx/vy/wz, never as a virtual joint."""
    assert chassis.joint_names() == ()


def test_an_imu_reports_but_is_never_commanded(g1: ControlDescription) -> None:
    """A sensor adds state keys, no command keys, and is not a joint."""
    assert "g1/imu/qw" in g1.state_keys()
    assert not any(key.startswith("g1/imu/") for key in g1.command_keys())
    assert "g1/imu" not in g1.joint_names()
    assert g1.unit_of("g1/imu/gz") is Unit.RAD_PER_S
    assert g1.unit_of("g1/imu/az") is Unit.M_PER_S2


def test_unit_of(xarm: ControlDescription) -> None:
    """Units come from the owning resource, and are None when undeclared."""
    assert xarm.unit_of("arm/joint1/position") is Unit.RAD
    assert xarm.unit_of("arm/gripper/position") is Unit.M
    assert xarm.unit_of("arm/joint1/nonsense") is None
    assert xarm.unit_of("elsewhere/joint1/position") is None


def test_gripper_units_are_per_gripper(xarm: ControlDescription) -> None:
    """Metres here, but a gripper measured 0 to 1 is just as declarable."""
    normalized = dataclasses.replace(
        xarm.resource("gripper"),  # type: ignore[arg-type]
        units={Interface.POSITION: Unit.NORMALIZED},
    )
    swapped = dataclasses.replace(xarm, resources=(*xarm.resources[:-1], normalized))

    assert swapped.unit_of("arm/gripper/position") is Unit.NORMALIZED


def test_resource_lookup(xarm: ControlDescription) -> None:
    """Resource lookup is by declared name, or None."""
    assert xarm.resource("gripper") is not None
    assert xarm.resource("missing") is None


def test_descriptions_are_frozen(xarm: ControlDescription) -> None:
    """A change means building a new description, not editing this one."""
    with pytest.raises(dataclasses.FrozenInstanceError):
        xarm.source = "other"  # type: ignore[misc]


def test_every_dataclass_pickles() -> None:
    """Descriptions travel over RPC, so every part must survive the round trip."""
    parts = (
        Resource(
            name="joint1",
            kind=ResourceKind.JOINT,
            state_interfaces=(Interface.POSITION,),
            command_interfaces=(Interface.POSITION,),
            units={Interface.POSITION: Unit.RAD},
        ),
        Limits(-1.0, 1.0, clamp=True),
    )
    for original in parts:
        assert pickle.loads(pickle.dumps(original)) == original


def test_whole_descriptions_pickle(
    xarm: ControlDescription, g1: ControlDescription, chassis: ControlDescription
) -> None:
    """A robot hands its description over from its own process, so it must come back equal."""
    for desc in (xarm, g1, chassis):
        restored = pickle.loads(pickle.dumps(desc))

        assert restored == desc
        assert restored.state_keys() == desc.state_keys()
        assert restored.command_keys() == desc.command_keys()


def test_resource_kinds_pickle_by_identity() -> None:
    """Enum members compare by identity after a round trip, not just by value."""
    for kind in ResourceKind:
        assert pickle.loads(pickle.dumps(kind)) is kind
