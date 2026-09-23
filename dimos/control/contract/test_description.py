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

"""The description's derived views, and that all of it survives RPC."""

from __future__ import annotations

import dataclasses
import pickle

import pytest

from dimos.control.contract.description import (
    WRITE_ON_RECEIPT,
    ActivationPolicy,
    AvailableAfter,
    ControlDescription,
    Estop,
    EstopKind,
    EstopRecovery,
    LimitPolicy,
    Limits,
    ModeGroup,
    Omission,
    ProcessLoss,
    Resource,
    ResourceKind,
    SafeStop,
    SafeStopKind,
    ShutdownMotion,
    Timing,
)
from dimos.control.contract.keys import EFFORT, KD, KP, POSITION, VELOCITY, VX, Unit


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


def test_unit_of(xarm: ControlDescription) -> None:
    """Units come from the owning resource, and are None when undeclared."""
    assert xarm.unit_of("arm/joint1/position") is Unit.RAD
    assert xarm.unit_of("arm/gripper/position") is Unit.M
    assert xarm.unit_of("arm/joint1/nonsense") is None
    assert xarm.unit_of("elsewhere/joint1/position") is None


def test_gripper_units_are_per_gripper(xarm: ControlDescription) -> None:
    """Metres here, but a normalized gripper is equally declarable (D19)."""
    normalized = dataclasses.replace(
        xarm.resource("gripper"),  # type: ignore[arg-type]
        units={POSITION: Unit.NORMALIZED},
    )
    swapped = dataclasses.replace(xarm, resources=(*xarm.resources[:-1], normalized))

    assert swapped.unit_of("arm/gripper/position") is Unit.NORMALIZED


def test_omission_defaults(xarm: ControlDescription) -> None:
    """Position and gains hold their last value; everything else falls to zero."""
    assert xarm.omission_of("arm/joint1/position") is Omission.RETAIN_LAST
    assert xarm.omission_of("arm/joint1/velocity") is Omission.ZERO


def test_omission_defaults_cover_gains(g1: ControlDescription) -> None:
    """kp and kd retain, which is what lets a position-only task win a PD joint."""
    assert g1.omission_of("g1/joint1/kp") is Omission.RETAIN_LAST
    assert g1.omission_of("g1/joint1/kd") is Omission.RETAIN_LAST
    assert g1.omission_of("g1/joint1/effort") is Omission.ZERO


def test_explicit_omission_overrides_the_default(g1: ControlDescription) -> None:
    """UNSET is opt-in, and is the only way None reaches a vendor's write()."""
    assert g1.omission_of("g1/joint1/velocity") is Omission.UNSET


def test_groups_for(xarm: ControlDescription) -> None:
    """An arm joint sits in both exclusive groups; the gripper in its own."""
    assert [g.name for g in xarm.groups_for("joint1")] == ["position", "velocity"]
    assert [g.name for g in xarm.groups_for("gripper")] == ["gripper"]
    assert xarm.groups_for("nonexistent") == ()


def test_is_command_and_state_key(xarm: ControlDescription) -> None:
    """Membership answers for both directions, and for foreign sources."""
    assert xarm.is_command_key("arm/joint1/position")
    assert not xarm.is_command_key("arm/joint1/effort")
    assert xarm.is_state_key("arm/joint1/effort")
    assert not xarm.is_state_key("g1/joint1/position")


def test_resource_lookup(xarm: ControlDescription) -> None:
    """Resource lookup is by declared name, or None."""
    assert xarm.resource("gripper") is not None
    assert xarm.resource("missing") is None


def test_write_rate_defaults_to_the_state_rate(g1: ControlDescription) -> None:
    """None means 'the state rate', so a description need not repeat itself."""
    assert g1.timing.write_rate_hz is None
    assert g1.timing.effective_write_rate_hz() == 500.0


def test_explicit_write_rate_is_used(chassis: ControlDescription) -> None:
    """A declared rate wins over the state rate."""
    assert chassis.timing.effective_write_rate_hz() == 50.0


def test_write_on_receipt_is_distinct_from_none() -> None:
    """The D16 escape hatch is its own sentinel, so None never means two things."""
    timing = Timing(
        state_rate_hz=100.0,
        stale_timeout_s=0.05,
        watchdog_timeout_s=0.1,
        write_rate_hz=WRITE_ON_RECEIPT,
    )

    assert timing.effective_write_rate_hz() is None
    assert WRITE_ON_RECEIPT is not None


def test_process_loss_scalar_and_per_group(g1: ControlDescription) -> None:
    """A source may answer once, or per mode group."""
    assert g1.process_loss_of() is ProcessLoss.UNPROTECTED
    assert g1.process_loss_of("pd") is ProcessLoss.UNPROTECTED

    per_group = dataclasses.replace(g1, process_loss={"pd": ProcessLoss.NATIVE_WATCHDOG})

    assert per_group.process_loss_of("pd") is ProcessLoss.NATIVE_WATCHDOG
    assert per_group.process_loss_of("absent") is ProcessLoss.UNKNOWN


def test_descriptions_are_frozen(xarm: ControlDescription) -> None:
    """Immutable per epoch: a re-describe builds a new one and bumps the epoch."""
    with pytest.raises(dataclasses.FrozenInstanceError):
        xarm.source = "other"  # type: ignore[misc]


@pytest.fixture
def every_dataclass() -> tuple[object, ...]:
    """One instance of every frozen dataclass in the package."""
    return (
        Resource(
            name="joint1",
            kind=ResourceKind.JOINT,
            state_interfaces=(POSITION,),
            command_interfaces=(POSITION, KP, KD, VELOCITY, EFFORT),
            units={POSITION: Unit.RAD},
        ),
        Limits(-1.0, 1.0, LimitPolicy.CLAMP),
        ModeGroup(name="pd", resources=("joint1",), interfaces=frozenset({POSITION, VX})),
        SafeStop(kind=SafeStopKind.DAMP, kd={"g1/joint1/kd": 5.0}, stable_state="floor"),
        Estop(kind=EstopKind.DAMP, recovery=EstopRecovery.PREPARE_ARM_REQUIRED),
        Timing(state_rate_hz=100.0, stale_timeout_s=0.05, watchdog_timeout_s=0.1),
        ShutdownMotion(pose={"arm/joint1/position": 0.0}, tolerance=0.01, timeout_s=5.0),
    )


def test_every_dataclass_pickles(every_dataclass: tuple[object, ...]) -> None:
    """Descriptions travel over RPC, so every part must survive the round trip."""
    for original in every_dataclass:
        assert pickle.loads(pickle.dumps(original)) == original


def test_whole_descriptions_pickle(
    xarm: ControlDescription, g1: ControlDescription, chassis: ControlDescription
) -> None:
    """describe_control() returns these across a process boundary."""
    for desc in (xarm, g1, chassis):
        restored = pickle.loads(pickle.dumps(desc))

        assert restored == desc
        assert restored.state_keys() == desc.state_keys()
        assert restored.command_keys() == desc.command_keys()


@pytest.mark.parametrize(
    "enum_member",
    [
        ResourceKind.SENSOR,
        LimitPolicy.CLAMP,
        Omission.UNSET,
        SafeStopKind.ZERO_RAMP,
        EstopKind.VENDOR,
        EstopRecovery.PREPARE_ARM_REQUIRED,
        ActivationPolicy.OPERATOR_CONFIRMED,
        ProcessLoss.UNKNOWN,
        AvailableAfter.PREPARE_ARM,
        WRITE_ON_RECEIPT,
    ],
)
def test_enums_pickle_by_identity(enum_member: object) -> None:
    """Enum members compare by identity after a round trip, not just by value."""
    assert pickle.loads(pickle.dumps(enum_member)) is enum_member
