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

"""Every description rule, and what a frame has to look like to be applied.

Each numbered rule gets a passing case (the fixtures, which are all valid) and
a failing case built by breaking exactly one thing with ``dataclasses.replace``.
"""

from __future__ import annotations

import dataclasses
import pickle
from types import MappingProxyType

import pytest

from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import (
    EFFORT,
    PITCH,
    POSITION,
    ROLL,
    VX,
    VY,
    VZ,
    WX,
    WY,
    WZ,
    YAW,
    Unit,
    X,
    Y,
    Z,
)
from dimos.control.contract.validate import (
    CommandBatch,
    DescriptionError,
    FrameRejectedError,
    Rejected,
    validate_command,
    validate_description,
    validate_state,
)
from dimos.msgs.control_msgs.ControlValues import ControlValues


def errors_of(desc: ControlDescription) -> list[str]:
    """Every problem in ``desc``, or an empty list when it is valid."""
    try:
        validate_description(desc)
    except DescriptionError as exc:
        return exc.errors
    return []


def command(source: str, values: dict[str, float], *, sequence: int = 1) -> ControlValues:
    """A command frame carrying ``values``."""
    return ControlValues(
        source=source,
        source_ts=0.0,
        epoch=0,
        sequence=sequence,
        interface_names=list(values),
        values=list(values.values()),
    )


def full_state(desc: ControlDescription) -> ControlValues:
    """A complete, well-formed state frame for ``desc``."""
    keys = list(desc.state_keys())
    return ControlValues(
        source=desc.source,
        source_ts=0.0,
        epoch=0,
        sequence=1,
        interface_names=keys,
        values=[0.0] * len(keys),
    )


# validate_description, one section per numbered rule


def test_the_fixtures_are_valid(
    xarm: ControlDescription, g1: ControlDescription, chassis: ControlDescription
) -> None:
    """All three shapes pass every rule; the failing cases below break one each."""
    for desc in (xarm, g1, chassis):
        validate_description(desc)


def test_the_five_fields_alone_make_a_valid_description() -> None:
    """One joint, no limits, a rate and a timeout is everything a robot must say."""
    desc = ControlDescription(
        source="bot",
        resources=(
            Resource(
                name="joint1",
                kind=ResourceKind.JOINT,
                state_interfaces=(POSITION,),
                command_interfaces=(POSITION,),
                units={POSITION: Unit.RAD},
            ),
        ),
        state_rate_hz=100.0,
        deadman_timeout_s=0.1,
    )

    validate_description(desc)


def test_every_problem_is_reported_at_once(xarm: ControlDescription) -> None:
    """A vendor fixing one typo per run would be a bad afternoon."""
    broken = dataclasses.replace(
        xarm,
        source="not a segment",
        state_rate_hz=0.0,
        limits={**xarm.limits, "arm/nonexistent/position": Limits(0.0, 1.0)},
    )

    errors = errors_of(broken)

    assert len(errors) >= 3
    assert any("source" in e for e in errors)
    assert any("state_rate_hz" in e for e in errors)
    assert any("nonexistent" in e for e in errors)


# Rule 1: names are segments, resources unique, interfaces have units.


def test_rule1_invalid_source(xarm: ControlDescription) -> None:
    assert any("source" in e for e in errors_of(dataclasses.replace(xarm, source="arm/extra")))


def test_rule1_duplicate_resource_names(xarm: ControlDescription) -> None:
    doubled = dataclasses.replace(xarm, resources=(*xarm.resources, xarm.resources[0]))

    assert any("duplicate resource" in e for e in errors_of(doubled))


def test_rule1_interface_without_a_unit(xarm: ControlDescription) -> None:
    """An interface with no unit is unusable: a task cannot know what it sent."""
    unitless = dataclasses.replace(xarm.resources[0], units={POSITION: Unit.RAD})
    broken = dataclasses.replace(xarm, resources=(unitless, *xarm.resources[1:]))

    assert any("no declared unit" in e for e in errors_of(broken))


def test_rule1_duplicate_interface_on_one_resource(xarm: ControlDescription) -> None:
    """A repeated interface yields a repeated key, and then no state frame is valid.

    validate_state demands each declared key exactly once, so a description that
    let this through would produce a source that can never report at all.
    """
    doubled = dataclasses.replace(xarm.resources[0], state_interfaces=(POSITION, POSITION, EFFORT))
    broken = dataclasses.replace(xarm, resources=(doubled, *xarm.resources[1:]))

    assert any("more than once" in e for e in errors_of(broken))


# Rule 2: limits sit on command keys, describe a real range, and only bases clamp.


def test_rule2_limit_on_an_undeclared_key(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, limits={"arm/joint1/nonsense": Limits(0.0, 1.0)})

    assert any("not a declared command key" in e for e in errors_of(broken))


def test_rule2_limit_on_a_state_only_key(xarm: ControlDescription) -> None:
    """Limits are only checked on commands, so one on a reading would do nothing."""
    broken = dataclasses.replace(xarm, limits={"arm/joint1/effort": Limits(-1.0, 1.0)})

    assert any("not a declared command key" in e for e in errors_of(broken))


def test_rule2_inverted_bounds(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, limits={"arm/joint1/position": Limits(1.0, -1.0)})

    assert any("above hi" in e for e in errors_of(broken))


def test_rule2_non_finite_bound(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, limits={"arm/joint1/position": Limits(float("nan"), 1.0)})

    assert any("non-finite" in e for e in errors_of(broken))


def test_rule2_clamp_needs_both_bounds(chassis: ControlDescription) -> None:
    """Clamping to an open side is meaningless, so it is refused up front."""
    broken = dataclasses.replace(
        chassis, limits={"chassis/base/vx": Limits(-1.0, None, clamp=True)}
    )

    assert any("bounded on both sides" in e for e in errors_of(broken))


def test_rule2_a_joint_may_not_clamp(xarm: ControlDescription) -> None:
    """A joint past its limit is refused, never trimmed."""
    broken = dataclasses.replace(
        xarm, limits={"arm/joint1/position": Limits(-1.0, 1.0, clamp=True)}
    )

    assert any("only a base's commands may clamp" in e for e in errors_of(broken))


# Rule 3: rates are positive and finite.


@pytest.mark.parametrize("field", ["state_rate_hz", "deadman_timeout_s"])
@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_rule3_rates_must_be_positive(xarm: ControlDescription, field: str, bad: float) -> None:
    broken = dataclasses.replace(xarm, **{field: bad})

    assert any(field in e for e in errors_of(broken))


def test_mapping_fields_accept_any_mapping(xarm: ControlDescription) -> None:
    """Any mapping is accepted, and stored as a plain dict so it pickles."""
    resource = dataclasses.replace(
        xarm.resources[0], units=MappingProxyType(dict(xarm.resources[0].units))
    )
    fine = dataclasses.replace(
        xarm,
        resources=(resource, *xarm.resources[1:]),
        limits=MappingProxyType(dict(xarm.limits)),
    )

    validate_description(fine)

    # MappingProxyType cannot be pickled, so storing one would break the RPC hop.
    assert isinstance(fine.limits, dict)
    assert isinstance(fine.resources[0].units, dict)
    assert pickle.loads(pickle.dumps(fine)) == fine


# Errors


def test_description_error_pickles_and_reads_well() -> None:
    """It travels back over RPC, so it must survive pickling with its list."""
    original = DescriptionError(["first problem", "second problem"])

    restored = pickle.loads(pickle.dumps(original))

    assert restored.errors == ["first problem", "second problem"]
    assert str(restored) == "2 problem(s): first problem; second problem"


def test_frame_rejected_error_pickles_and_reads_well() -> None:
    """Same for the state-frame rejection, which carries a reason and a detail."""
    original = FrameRejectedError("missing_key", "state frame omits ['arm/j1/position']")

    restored = pickle.loads(pickle.dumps(original))

    assert restored.reason == "missing_key"
    assert restored.detail == "state frame omits ['arm/j1/position']"
    assert str(restored) == "missing_key: state frame omits ['arm/j1/position']"


def test_mismatched_array_lengths_are_caught_by_both_validators(
    xarm: ControlDescription,
) -> None:
    """ControlValues forbids this at construction, so only a mutated frame gets here.

    Both paths still check it, because validate_command is specified never to
    raise and a ragged zip would break that promise.
    """
    frame = ControlValues("arm", 0.0, 0, 1, ["arm/joint1/position"], [0.1])
    frame.values = [0.1, 0.2]

    with pytest.raises(FrameRejectedError, match="shape"):
        validate_state(xarm, frame)

    result = validate_command(xarm, frame, last_sequence=None)

    assert isinstance(result, Rejected)
    assert result.reason == "shape"


# validate_state


def test_state_round_trip(xarm: ControlDescription) -> None:
    """A complete frame becomes a mapping of every declared state key."""
    values = validate_state(xarm, full_state(xarm))

    assert set(values) == set(xarm.state_keys())


def test_state_includes_the_imu(g1: ControlDescription) -> None:
    """A sensor's readings are part of the full report like any joint's."""
    values = validate_state(g1, full_state(g1))

    assert "g1/imu/qw" in values


def test_state_from_the_wrong_source(xarm: ControlDescription) -> None:
    keys = list(xarm.state_keys())
    frame = ControlValues("somebody_else", 0.0, 0, 1, keys, [0.0] * len(keys))

    with pytest.raises(FrameRejectedError, match="source"):
        validate_state(xarm, frame)


def test_state_missing_a_key(xarm: ControlDescription) -> None:
    """A state frame is a full report: a partial one is a bug, not a sparse update."""
    keys = list(xarm.state_keys())[:-1]
    frame = ControlValues("arm", 0.0, 0, 1, keys, [0.0] * len(keys))

    with pytest.raises(FrameRejectedError, match="missing_key"):
        validate_state(xarm, frame)


def test_state_with_an_unknown_key(xarm: ControlDescription) -> None:
    keys = [*xarm.state_keys(), "arm/joint1/surprise"]
    frame = ControlValues("arm", 0.0, 0, 1, keys, [0.0] * len(keys))

    with pytest.raises(FrameRejectedError, match="unknown_key"):
        validate_state(xarm, frame)


def test_state_with_a_duplicate_key(xarm: ControlDescription) -> None:
    keys = [*xarm.state_keys(), xarm.state_keys()[0]]
    frame = ControlValues("arm", 0.0, 0, 1, keys, [0.0] * len(keys))

    with pytest.raises(FrameRejectedError, match="duplicate_key"):
        validate_state(xarm, frame)


# validate_command


def test_command_accepts_values_within_limits(xarm: ControlDescription) -> None:
    result = validate_command(
        xarm,
        command("coordinator", {"arm/joint1/position": 0.5, "arm/joint2/position": -0.5}),
        last_sequence=None,
    )

    assert result == CommandBatch(
        values={"arm/joint1/position": 0.5, "arm/joint2/position": -0.5}, clamped=()
    )


def test_command_rejects_a_replayed_sequence(xarm: ControlDescription) -> None:
    result = validate_command(xarm, command("c", {}, sequence=5), last_sequence=5)

    assert isinstance(result, Rejected)
    assert result.reason == "sequence"


def test_command_accepts_the_next_sequence(xarm: ControlDescription) -> None:
    result = validate_command(xarm, command("c", {}, sequence=6), last_sequence=5)

    assert isinstance(result, CommandBatch)


def test_heartbeat_is_accepted(xarm: ControlDescription) -> None:
    """An empty frame commands nothing but still shows the sender is alive."""
    result = validate_command(xarm, command("c", {}), last_sequence=None)

    assert result == CommandBatch(values={}, clamped=())


def test_other_sources_are_ignored_not_rejected(xarm: ControlDescription) -> None:
    """One instruction can carry commands for several robots at once."""
    result = validate_command(
        xarm,
        command(
            "coordinator",
            {
                "arm/joint1/position": 0.5,
                "g1/joint1/position": 1.0,
                "chassis/base/vx": 0.2,
            },
        ),
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.values == {"arm/joint1/position": 0.5}


def test_a_frame_of_only_foreign_keys_is_an_empty_batch(xarm: ControlDescription) -> None:
    result = validate_command(xarm, command("c", {"g1/joint1/position": 1.0}), last_sequence=None)

    assert result == CommandBatch(values={}, clamped=())


def test_command_rejects_an_undeclared_key(xarm: ControlDescription) -> None:
    result = validate_command(xarm, command("c", {"arm/joint1/effort": 1.0}), last_sequence=None)

    assert isinstance(result, Rejected)
    assert result.reason == "unknown_key"


def test_command_rejects_duplicate_keys(xarm: ControlDescription) -> None:
    frame = ControlValues(
        "c", 0.0, 0, 1, ["arm/joint1/position", "arm/joint1/position"], [0.1, 0.2]
    )

    result = validate_command(xarm, frame, last_sequence=None)

    assert isinstance(result, Rejected)
    assert result.reason == "duplicate_key"


@pytest.mark.parametrize("value", [99.0, -99.0])
def test_out_of_limit_is_refused_by_default(xarm: ControlDescription, value: float) -> None:
    """The xArm's limits say nothing about clamping, so a value past one is refused."""
    result = validate_command(
        xarm, command("c", {"arm/joint1/position": value}), last_sequence=None
    )

    assert isinstance(result, Rejected)
    assert result.reason == "limit"


def test_a_declared_clamp_pulls_the_value_back(chassis: ControlDescription) -> None:
    """The chassis asks to clamp, so a stick pushed too far drives at top speed."""
    result = validate_command(
        chassis,
        command("c", {"chassis/base/vx": 9.0, "chassis/base/vy": -9.0, "chassis/base/wz": 0.5}),
        last_sequence=None,
    )

    assert result == CommandBatch(
        values={"chassis/base/vx": 1.5, "chassis/base/vy": -1.0, "chassis/base/wz": 0.5},
        clamped=("chassis/base/vx", "chassis/base/vy"),
    )


def test_a_rejected_batch_applies_nothing(xarm: ControlDescription) -> None:
    """All or nothing: one bad key does not let the good ones through."""
    result = validate_command(
        xarm,
        command("c", {"arm/joint1/position": 0.1, "arm/joint2/position": 99.0}),
        last_sequence=None,
    )

    assert isinstance(result, Rejected)


def test_the_whole_body_drives_five_interfaces_at_once(g1: ControlDescription) -> None:
    """Position, velocity, effort and both gains on one joint is a normal frame."""
    values = {
        "g1/joint1/position": 0.1,
        "g1/joint1/velocity": 0.0,
        "g1/joint1/effort": 0.5,
        "g1/joint1/kp": 60.0,
        "g1/joint1/kd": 1.5,
    }

    result = validate_command(g1, command("c", values), last_sequence=None)

    assert result == CommandBatch(values=values, clamped=())


def test_a_sparse_command_is_accepted(xarm: ControlDescription) -> None:
    """Commanding one of seven joints is sparse, not incomplete."""
    result = validate_command(xarm, command("c", {"arm/joint3/position": 0.1}), last_sequence=None)

    assert result == CommandBatch(values={"arm/joint3/position": 0.1}, clamped=())


def test_a_base_may_be_six_dof() -> None:
    """Something that flies uses all six axes, with the same names.

    Nothing forces a base to stay on the floor. One that drives declares the
    few axes it has; one that flies declares all six.
    """
    linear = (VX, VY, VZ)
    angular = (WX, WY, WZ)
    axes = linear + angular
    drone = ControlDescription(
        source="drone",
        resources=(
            Resource(
                name="body",
                kind=ResourceKind.BASE,
                state_interfaces=(*axes, X, Y, Z, ROLL, PITCH, YAW),
                command_interfaces=axes,
                units=dict.fromkeys(linear, Unit.M_PER_S)
                | dict.fromkeys(angular, Unit.RAD_PER_S)
                | {X: Unit.M, Y: Unit.M, Z: Unit.M}
                | dict.fromkeys((ROLL, PITCH, YAW), Unit.RAD),
            ),
        ),
        state_rate_hz=100.0,
        deadman_timeout_s=0.1,
    )

    validate_description(drone)

    result = validate_command(
        drone, command("c", {f"drone/body/{a}": 0.1 for a in axes}), last_sequence=None
    )

    assert isinstance(result, CommandBatch)
    assert len(result.values) == 6
    # A base is a resource, not a set of virtual joints.
    assert drone.joint_names() == ()
    # Linear axes are m/s and angular axes rad/s. This fixture is a reference
    # someone will copy, so the units have to be physically right.
    assert all(drone.unit_of(f"drone/body/{a}") is Unit.M_PER_S for a in linear)
    assert all(drone.unit_of(f"drone/body/{a}") is Unit.RAD_PER_S for a in angular)
