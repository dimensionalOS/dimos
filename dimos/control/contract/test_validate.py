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
    ActivationPolicy,
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
from dimos.control.contract.keys import (
    EFFORT,
    PITCH,
    POSITION,
    QW,
    ROLL,
    VELOCITY,
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


def command(
    source: str, values: dict[str, float], *, epoch: int = 1, sequence: int = 1
) -> ControlValues:
    """A command frame carrying ``values``."""
    return ControlValues(
        source=source,
        source_ts=0.0,
        epoch=epoch,
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
        epoch=1,
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


def test_every_problem_is_reported_at_once(xarm: ControlDescription) -> None:
    """A vendor fixing one typo per run would be a bad afternoon."""
    broken = dataclasses.replace(
        xarm,
        source="not a segment",
        epoch=-1,
        limits={**xarm.limits, "arm/nonexistent/position": Limits(0.0, 1.0)},
    )

    errors = errors_of(broken)

    assert len(errors) >= 3
    assert any("source" in e for e in errors)
    assert any("epoch" in e for e in errors)
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


# Rule 2: limits address declared keys and describe a real range.


def test_rule2_limit_on_an_undeclared_key(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, limits={"arm/joint1/nonsense": Limits(0.0, 1.0)})

    assert any("not a declared state or command key" in e for e in errors_of(broken))


def test_rule2_inverted_bounds(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, limits={"arm/joint1/position": Limits(1.0, -1.0)})

    assert any("above hi" in e for e in errors_of(broken))


def test_rule2_non_finite_bound(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, limits={"arm/joint1/position": Limits(float("nan"), 1.0)})

    assert any("non-finite" in e for e in errors_of(broken))


def test_rule2_clamp_needs_both_bounds(xarm: ControlDescription) -> None:
    """Clamping to an open side is meaningless, so it is refused up front."""
    broken = dataclasses.replace(
        xarm, limits={"arm/joint1/position": Limits(-1.0, None, LimitPolicy.CLAMP)}
    )

    assert any("bounded on both sides" in e for e in errors_of(broken))


# Rule 3: mode groups.


def test_rule3_duplicate_group_names(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, mode_groups=(*xarm.mode_groups, xarm.mode_groups[0]))

    assert any("duplicate mode group" in e for e in errors_of(broken))


def test_rule3_group_names_an_undeclared_resource(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(
        xarm,
        mode_groups=(
            ModeGroup(name="ghost", resources=("nobody",), interfaces=frozenset({POSITION})),
            *xarm.mode_groups,
        ),
    )

    assert any("undeclared resource" in e for e in errors_of(broken))


def test_rule3_group_interface_not_commandable(xarm: ControlDescription) -> None:
    """A group may only name interfaces its resources actually accept."""
    broken = dataclasses.replace(
        xarm,
        mode_groups=(
            ModeGroup(name="position", resources=("joint1",), interfaces=frozenset({EFFORT})),
            *xarm.mode_groups[1:],
        ),
    )

    assert any("not commandable" in e for e in errors_of(broken))


def test_rule3_a_group_may_span_resources_with_different_interfaces() -> None:
    """A whole-body group over an arm joint and a base is legal.

    The rule is the union of the group's resources' command interfaces, not the
    intersection: a mobile manipulator drives ``position`` on the arm and
    ``vx``/``vy``/``wz`` on the base from one group.
    """
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
            Resource(
                name="base",
                kind=ResourceKind.BASE,
                state_interfaces=(VX,),
                command_interfaces=(VX, VY, WZ),
                units={VX: Unit.M_PER_S, VY: Unit.M_PER_S, WZ: Unit.RAD_PER_S},
            ),
        ),
        mode_groups=(
            ModeGroup(
                name="whole_body",
                resources=("joint1", "base"),
                interfaces=frozenset({POSITION, VX, VY, WZ}),
            ),
        ),
        safe_stop=SafeStop(kind=SafeStopKind.HOLD),
        estop=Estop(kind=EstopKind.HOLD, recovery=EstopRecovery.CLEAR),
        activation_policy=ActivationPolicy.DIRECT,
        timing=Timing(state_rate_hz=100.0, stale_timeout_s=0.05, watchdog_timeout_s=0.1),
        process_loss=ProcessLoss.UNKNOWN,
    )

    validate_description(desc)

    # And the two halves really do drive together, under one group.
    result = validate_command(
        desc,
        command("c", {"bot/joint1/position": 0.1, "bot/base/vx": 0.2}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.active_groups == frozenset({"whole_body"})

    # An interface the resource does not declare is still refused, so the
    # looser group rule opens no hole.
    stray = validate_command(
        desc, command("c", {"bot/joint1/vx": 0.2}), current_epoch=1, last_sequence=None
    )

    assert isinstance(stray, Rejected)
    assert stray.reason == "unknown_key"


def test_rule3_interface_commandable_on_no_resource_is_still_caught() -> None:
    """The union rule still rejects an interface no member resource accepts."""
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
        mode_groups=(
            ModeGroup(name="bad", resources=("joint1",), interfaces=frozenset({POSITION, EFFORT})),
        ),
        safe_stop=SafeStop(kind=SafeStopKind.HOLD),
        estop=Estop(kind=EstopKind.HOLD, recovery=EstopRecovery.CLEAR),
        activation_policy=ActivationPolicy.DIRECT,
        timing=Timing(state_rate_hz=100.0, stale_timeout_s=0.05, watchdog_timeout_s=0.1),
        process_loss=ProcessLoss.UNKNOWN,
    )

    assert any("not commandable on any" in e for e in errors_of(desc))


def test_rule3_member_driving_none_of_its_interfaces(xarm: ControlDescription) -> None:
    """Taking control of a part you cannot then command would leave it stuck.

    Nothing else could use it, and this group could not move it either.
    """
    broken = dataclasses.replace(
        xarm,
        mode_groups=(
            ModeGroup(
                name="position",
                resources=("joint1", "gripper"),
                interfaces=frozenset({VELOCITY}),
            ),
            *xarm.mode_groups[1:],
        ),
    )

    assert any("drives none of its interfaces" in e for e in errors_of(broken))


def test_rule3_group_with_no_interfaces(xarm: ControlDescription) -> None:
    """An empty interface set drives nothing on any member."""
    broken = dataclasses.replace(
        xarm,
        mode_groups=(
            ModeGroup(name="empty", resources=("joint1",), interfaces=frozenset()),
            *xarm.mode_groups,
        ),
    )

    assert any("drives none of its interfaces" in e for e in errors_of(broken))


def test_rule3_sensor_listed_in_a_group(xarm: ControlDescription) -> None:
    """A sensor commands nothing, so a group listing it drives nothing on it."""
    imu = Resource(
        name="imu",
        kind=ResourceKind.SENSOR,
        state_interfaces=(QW,),
        command_interfaces=(),
        units={QW: Unit.UNITLESS},
    )
    broken = dataclasses.replace(
        xarm,
        resources=(*xarm.resources, imu),
        mode_groups=(
            ModeGroup(
                name="position",
                resources=(*xarm.mode_groups[0].resources, "imu"),
                interfaces=frozenset({POSITION}),
            ),
            *xarm.mode_groups[1:],
        ),
    )

    assert any("drives none of its interfaces" in e for e in errors_of(broken))


def test_rule3_uncovered_command_interface(xarm: ControlDescription) -> None:
    """An interface in no group can never be sent, which is a description bug."""
    broken = dataclasses.replace(xarm, mode_groups=xarm.mode_groups[1:])

    assert any("can never be sent" in e for e in errors_of(broken))


def test_rule3_interface_in_two_groups(xarm: ControlDescription) -> None:
    """Two groups offering the same interface make the winner ambiguous."""
    broken = dataclasses.replace(
        xarm,
        mode_groups=(
            *xarm.mode_groups,
            ModeGroup(name="second", resources=("joint1",), interfaces=frozenset({POSITION})),
        ),
    )

    assert any("several mode groups" in e for e in errors_of(broken))


# Rule 4: omission addresses command keys.


def test_rule4_omission_on_a_state_only_key(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, omission={"arm/joint1/effort": Omission.ZERO})

    assert any("omission key" in e for e in errors_of(broken))


# Rule 5: initial values.


def test_rule5_initial_value_on_an_unknown_key(g1: ControlDescription) -> None:
    broken = dataclasses.replace(g1, initial_values={"g1/joint1/nonsense": 1.0})

    assert any("initial value key" in e for e in errors_of(broken))


def test_rule5_initial_position_is_refused(g1: ControlDescription) -> None:
    """Positions seed from measured state, never from a constant in a file."""
    broken = dataclasses.replace(g1, initial_values={"g1/joint1/position": 0.0})

    assert any("positions seed from measured" in e for e in errors_of(broken))


def test_rule5_initial_value_needs_retain_last(g1: ControlDescription) -> None:
    """Seeding a value that gets zeroed every cycle would be a lie."""
    broken = dataclasses.replace(
        g1,
        omission={**g1.omission, "g1/joint1/kp": Omission.ZERO},
        initial_values={"g1/joint1/kp": 60.0},
    )

    assert any("RETAIN_LAST" in e for e in errors_of(broken))


def test_rule5_initial_value_outside_limits(g1: ControlDescription) -> None:
    broken = dataclasses.replace(
        g1,
        limits={**g1.limits, "g1/joint1/kp": Limits(0.0, 10.0)},
        initial_values={"g1/joint1/kp": 60.0},
    )

    assert any("above its limit" in e for e in errors_of(broken))


def test_rule5_non_finite_initial_value(g1: ControlDescription) -> None:
    broken = dataclasses.replace(g1, initial_values={"g1/joint1/kp": float("inf")})

    assert any("not finite" in e for e in errors_of(broken))


# Rule 6: covered_resources.


def test_rule6_self_cover(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, covered_resources={"joint1": ("joint1",)})

    assert any("covers itself" in e for e in errors_of(broken))


def test_rule6_undeclared_resource(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, covered_resources={"ghost": ("joint1",)})

    assert any("covered_resources key" in e for e in errors_of(broken))


def test_rule6_cycle(xarm: ControlDescription) -> None:
    """A covers B covers A would loop the arbiter forever."""
    broken = dataclasses.replace(
        xarm, covered_resources={"joint1": ("joint2",), "joint2": ("joint1",)}
    )

    assert any("cycle" in e for e in errors_of(broken))


def test_rule6_valid_chain_is_accepted(xarm: ControlDescription) -> None:
    """Acyclic coverage is fine, however deep."""
    fine = dataclasses.replace(
        xarm, covered_resources={"joint1": ("joint2",), "joint2": ("joint3",)}
    )

    validate_description(fine)


# Rule 7: available_after.


def test_rule7_available_after_unknown_resource(xarm: ControlDescription) -> None:
    from dimos.control.contract.description import AvailableAfter

    broken = dataclasses.replace(xarm, available_after={"ghost": AvailableAfter.CONNECT})

    assert any("available_after key" in e for e in errors_of(broken))


# Rule 8: timing.


@pytest.mark.parametrize("field", ["state_rate_hz", "watchdog_timeout_s", "hook_timeout_s"])
def test_rule8_non_positive_timing(xarm: ControlDescription, field: str) -> None:
    broken = dataclasses.replace(xarm, timing=dataclasses.replace(xarm.timing, **{field: 0.0}))

    assert any(field in e for e in errors_of(broken))


def test_rule8_stale_timeout_shorter_than_a_state_period(xarm: ControlDescription) -> None:
    """A 100 Hz source with a 1 ms staleness budget would always look dead."""
    broken = dataclasses.replace(
        xarm, timing=dataclasses.replace(xarm.timing, stale_timeout_s=0.001)
    )

    assert any("shorter than one state period" in e for e in errors_of(broken))


def test_rule8_stale_timeout_exactly_one_period_is_allowed(xarm: ControlDescription) -> None:
    """The boundary is inclusive, matching Freshness."""
    fine = dataclasses.replace(xarm, timing=dataclasses.replace(xarm.timing, stale_timeout_s=0.01))

    validate_description(fine)


# Rule 9: stop policies.


def test_rule9_damp_without_a_kd_table(g1: ControlDescription) -> None:
    broken = dataclasses.replace(g1, safe_stop=SafeStop(kind=SafeStopKind.DAMP))

    assert any("needs a kd table" in e for e in errors_of(broken))


def test_rule9_damp_kd_must_name_kd_keys(g1: ControlDescription) -> None:
    broken = dataclasses.replace(
        g1, safe_stop=SafeStop(kind=SafeStopKind.DAMP, kd={"g1/joint1/position": 5.0})
    )

    assert any("is not a declared" in e for e in errors_of(broken))


def test_rule9_zero_ramp_needs_a_positive_ramp(chassis: ControlDescription) -> None:
    broken = dataclasses.replace(
        chassis, safe_stop=SafeStop(kind=SafeStopKind.ZERO_RAMP, ramp_s=0.0)
    )

    assert any("positive ramp_s" in e for e in errors_of(broken))


@pytest.mark.parametrize("kind", list(EstopKind))
@pytest.mark.parametrize("recovery", list(EstopRecovery))
def test_rule9_any_estop_combination_is_allowed(
    xarm: ControlDescription, kind: EstopKind, recovery: EstopRecovery
) -> None:
    """Estop policy is vendor business; the contract does not second-guess it."""
    validate_description(dataclasses.replace(xarm, estop=Estop(kind=kind, recovery=recovery)))


def test_rule1_duplicate_interface_on_one_resource(xarm: ControlDescription) -> None:
    """A repeated interface yields a repeated key, and then no state frame is valid.

    validate_state demands each declared key exactly once, so a description that
    let this through would produce a source that can never report at all.
    """
    doubled = dataclasses.replace(xarm.resources[0], state_interfaces=(POSITION, POSITION, EFFORT))
    broken = dataclasses.replace(xarm, resources=(doubled, *xarm.resources[1:]))

    assert any("more than once" in e for e in errors_of(broken))


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_rule9_non_finite_ramp(chassis: ControlDescription, bad: float) -> None:
    """NaN compares False to everything, so a bare `<= 0` would wave it through."""
    broken = dataclasses.replace(
        chassis, safe_stop=SafeStop(kind=SafeStopKind.ZERO_RAMP, ramp_s=bad)
    )

    assert any("finite ramp_s" in e for e in errors_of(broken))


def test_rule9_non_finite_damp_gain(g1: ControlDescription) -> None:
    """A NaN damping gain would reach the motors on the safe-stop path."""
    broken = dataclasses.replace(
        g1,
        safe_stop=SafeStop(kind=SafeStopKind.DAMP, kd={"g1/joint1/kd": float("nan")}),
    )

    assert any("is not finite" in e for e in errors_of(broken))


def test_shutdown_motion_pose_must_be_declared_positions(xarm: ControlDescription) -> None:
    """Parking a brakeless arm at an undeclared key would fail at de-energize time."""
    broken = dataclasses.replace(
        xarm,
        shutdown_motion=ShutdownMotion(
            pose={"arm/ghost/position": 0.0}, tolerance=0.01, timeout_s=5.0
        ),
    )

    assert any("not a declared command key" in e for e in errors_of(broken))


def test_shutdown_motion_pose_respects_limits(xarm: ControlDescription) -> None:
    """A park pose outside the joint's range is unreachable by construction."""
    broken = dataclasses.replace(
        xarm,
        shutdown_motion=ShutdownMotion(
            pose={"arm/joint1/position": 99.0}, tolerance=0.01, timeout_s=5.0
        ),
    )

    assert any("above its limit" in e for e in errors_of(broken))


@pytest.mark.parametrize(
    ("tolerance", "timeout_s"), [(-1.0, 5.0), (0.01, 0.0), (float("nan"), 5.0)]
)
def test_shutdown_motion_needs_positive_bounds(
    xarm: ControlDescription, tolerance: float, timeout_s: float
) -> None:
    """An unbounded park wait would hang shutdown."""
    broken = dataclasses.replace(
        xarm,
        shutdown_motion=ShutdownMotion(
            pose={"arm/joint1/position": 0.0}, tolerance=tolerance, timeout_s=timeout_s
        ),
    )

    assert any("must be positive" in e for e in errors_of(broken))


def test_a_valid_shutdown_motion_is_accepted(xarm: ControlDescription) -> None:
    """The Piper park-to-zero shape, which is the reason the field exists."""
    fine = dataclasses.replace(
        xarm,
        shutdown_motion=ShutdownMotion(
            pose={"arm/joint1/position": 0.0}, tolerance=0.01, timeout_s=5.0
        ),
    )

    validate_description(fine)


def test_mapping_fields_accept_any_mapping(xarm: ControlDescription) -> None:
    """Any mapping goes in; a plain dict is stored, so the result still pickles."""
    fine = dataclasses.replace(
        xarm,
        covered_resources=MappingProxyType({"joint1": ("joint2",)}),
        limits=MappingProxyType(dict(xarm.limits)),
    )

    validate_description(fine)

    # MappingProxyType cannot be pickled, so storing one would break the RPC hop.
    assert isinstance(fine.covered_resources, dict)
    assert isinstance(fine.limits, dict)
    assert pickle.loads(pickle.dumps(fine)) == fine


# Rule 10: per-group process loss.


def test_rule10_process_loss_names_a_group(xarm: ControlDescription) -> None:
    broken = dataclasses.replace(xarm, process_loss={"not_a_group": ProcessLoss.UNKNOWN})

    assert any("process_loss key" in e for e in errors_of(broken))


def test_rule10_per_group_process_loss_is_accepted(xarm: ControlDescription) -> None:
    """The xArm's two groups genuinely differ: servo holds, velocity may not."""
    fine = dataclasses.replace(
        xarm,
        process_loss={
            "position": ProcessLoss.INTRINSICALLY_SAFE,
            "velocity": ProcessLoss.UNKNOWN,
            "gripper": ProcessLoss.UNKNOWN,
        },
    )

    validate_description(fine)


# Rule 11: epoch.


def test_rule11_negative_epoch(xarm: ControlDescription) -> None:
    assert any("epoch" in e for e in errors_of(dataclasses.replace(xarm, epoch=-1)))


# validate_state


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
    frame = ControlValues("arm", 0.0, 1, 1, ["arm/joint1/position"], [0.1])
    frame.values = [0.1, 0.2]

    with pytest.raises(FrameRejectedError, match="shape"):
        validate_state(xarm, frame)

    result = validate_command(xarm, frame, current_epoch=1, last_sequence=None)

    assert isinstance(result, Rejected)
    assert result.reason == "shape"


def test_state_round_trip(xarm: ControlDescription) -> None:
    """A complete frame becomes a mapping of every declared state key."""
    values = validate_state(xarm, full_state(xarm))

    assert set(values) == set(xarm.state_keys())


def test_state_from_the_wrong_source(xarm: ControlDescription) -> None:
    keys = list(xarm.state_keys())
    frame = ControlValues("somebody_else", 0.0, 1, 1, keys, [0.0] * len(keys))

    with pytest.raises(FrameRejectedError, match="source"):
        validate_state(xarm, frame)


def test_state_missing_a_key(xarm: ControlDescription) -> None:
    """A state frame is a full report: a partial one is a bug, not a sparse update."""
    keys = list(xarm.state_keys())[:-1]
    frame = ControlValues("arm", 0.0, 1, 1, keys, [0.0] * len(keys))

    with pytest.raises(FrameRejectedError, match="missing_key"):
        validate_state(xarm, frame)


def test_state_with_an_unknown_key(xarm: ControlDescription) -> None:
    keys = [*xarm.state_keys(), "arm/joint1/surprise"]
    frame = ControlValues("arm", 0.0, 1, 1, keys, [0.0] * len(keys))

    with pytest.raises(FrameRejectedError, match="unknown_key"):
        validate_state(xarm, frame)


def test_state_with_a_duplicate_key(xarm: ControlDescription) -> None:
    keys = [*xarm.state_keys(), xarm.state_keys()[0]]
    frame = ControlValues("arm", 0.0, 1, 1, keys, [0.0] * len(keys))

    with pytest.raises(FrameRejectedError, match="duplicate_key"):
        validate_state(xarm, frame)


# validate_command


def test_command_accepts_a_single_group(xarm: ControlDescription) -> None:
    result = validate_command(
        xarm,
        command("coordinator", {"arm/joint1/position": 0.5, "arm/joint2/position": -0.5}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.active_groups == frozenset({"position"})
    assert result.values == {"arm/joint1/position": 0.5, "arm/joint2/position": -0.5}
    assert result.clamped == ()


def test_command_rejects_a_stale_epoch(xarm: ControlDescription) -> None:
    result = validate_command(xarm, command("c", {}, epoch=1), current_epoch=2, last_sequence=None)

    assert result == Rejected("epoch", "frame epoch 1, current 2")


def test_command_rejects_a_replayed_sequence(xarm: ControlDescription) -> None:
    result = validate_command(xarm, command("c", {}, sequence=5), current_epoch=1, last_sequence=5)

    assert isinstance(result, Rejected)
    assert result.reason == "sequence"


def test_command_accepts_the_next_sequence(xarm: ControlDescription) -> None:
    result = validate_command(xarm, command("c", {}, sequence=6), current_epoch=1, last_sequence=5)

    assert isinstance(result, CommandBatch)


def test_heartbeat_is_accepted(xarm: ControlDescription) -> None:
    """An empty frame commands nothing but still proves the coordinator is alive."""
    result = validate_command(xarm, command("c", {}), current_epoch=1, last_sequence=None)

    assert result == CommandBatch(values={}, active_groups=frozenset(), clamped=())


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
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.values == {"arm/joint1/position": 0.5}


def test_a_frame_of_only_foreign_keys_is_an_empty_batch(xarm: ControlDescription) -> None:
    result = validate_command(
        xarm, command("c", {"g1/joint1/position": 1.0}), current_epoch=1, last_sequence=None
    )

    assert result == CommandBatch(values={}, active_groups=frozenset(), clamped=())


def test_command_rejects_an_undeclared_key(xarm: ControlDescription) -> None:
    result = validate_command(
        xarm, command("c", {"arm/joint1/effort": 1.0}), current_epoch=1, last_sequence=None
    )

    assert isinstance(result, Rejected)
    assert result.reason == "unknown_key"


def test_command_rejects_duplicate_keys(xarm: ControlDescription) -> None:
    frame = ControlValues(
        "c", 0.0, 1, 1, ["arm/joint1/position", "arm/joint1/position"], [0.1, 0.2]
    )

    result = validate_command(xarm, frame, current_epoch=1, last_sequence=None)

    assert isinstance(result, Rejected)
    assert result.reason == "duplicate_key"


def test_command_rejects_out_of_limit_under_reject_policy(xarm: ControlDescription) -> None:
    result = validate_command(
        xarm, command("c", {"arm/joint1/position": 99.0}), current_epoch=1, last_sequence=None
    )

    assert isinstance(result, Rejected)
    assert result.reason == "limit"


def test_clamp_policy_clamps_and_records_the_key(g1: ControlDescription) -> None:
    """A value a hair past its limit is trimmed and sent, not thrown away."""
    result = validate_command(
        g1, command("c", {"g1/joint1/position": 99.0}), current_epoch=1, last_sequence=None
    )

    assert isinstance(result, CommandBatch)
    assert result.values["g1/joint1/position"] == 2.0
    assert result.clamped == ("g1/joint1/position",)


def test_clamp_applies_to_the_low_side_too(g1: ControlDescription) -> None:
    result = validate_command(
        g1, command("c", {"g1/joint1/position": -99.0}), current_epoch=1, last_sequence=None
    )

    assert isinstance(result, CommandBatch)
    assert result.values["g1/joint1/position"] == -2.0


def test_a_rejected_batch_applies_nothing(xarm: ControlDescription) -> None:
    """All or nothing: one bad key does not let the good ones through."""
    result = validate_command(
        xarm,
        command("c", {"arm/joint1/position": 0.1, "arm/joint2/position": 99.0}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, Rejected)


def test_mode_group_velocity_and_position_on_one_arm_is_rejected(
    xarm: ControlDescription,
) -> None:
    """One joint told where to go while another is told how fast to move."""
    result = validate_command(
        xarm,
        command("c", {"arm/joint1/position": 0.1, "arm/joint5/velocity": 0.2}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, Rejected)
    assert result.reason == "mode_group"


def test_mode_group_mixed_interfaces_on_one_resource_is_rejected(
    xarm: ControlDescription,
) -> None:
    """No single xArm group offers position and velocity together."""
    result = validate_command(
        xarm,
        command("c", {"arm/joint1/position": 0.1, "arm/joint1/velocity": 0.2}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, Rejected)
    assert result.reason == "mode_group"


@pytest.mark.parametrize(
    ("arm_key", "expected_group"),
    [("arm/joint1/position", "position"), ("arm/joint1/velocity", "velocity")],
)
def test_the_gripper_rides_alongside_either_arm_group(
    xarm: ControlDescription, arm_key: str, expected_group: str
) -> None:
    """The gripper's group is non-exclusive, so it never fights the arm."""
    result = validate_command(
        xarm,
        command("c", {arm_key: 0.1, "arm/gripper/position": 0.04}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.active_groups == frozenset({expected_group, "gripper"})


def test_the_whole_body_drives_five_interfaces_at_once(g1: ControlDescription) -> None:
    """One group covering position, velocity, effort and gains is a normal frame."""
    result = validate_command(
        g1,
        command(
            "c",
            {
                "g1/joint1/position": 0.1,
                "g1/joint1/velocity": 0.0,
                "g1/joint1/effort": 0.5,
                "g1/joint1/kp": 60.0,
                "g1/joint1/kd": 1.5,
            },
        ),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.active_groups == frozenset({"pd"})


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
        mode_groups=(ModeGroup(name="twist", resources=("body",), interfaces=frozenset(axes)),),
        safe_stop=SafeStop(kind=SafeStopKind.ZERO, stable_state="holds station"),
        estop=Estop(kind=EstopKind.ZERO, recovery=EstopRecovery.CLEAR),
        activation_policy=ActivationPolicy.OPERATOR_CONFIRMED,
        timing=Timing(state_rate_hz=100.0, stale_timeout_s=0.05, watchdog_timeout_s=0.1),
        process_loss=ProcessLoss.UNKNOWN,
    )

    validate_description(drone)

    result = validate_command(
        drone,
        command("c", {f"drone/body/{a}": 0.1 for a in axes}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.active_groups == frozenset({"twist"})
    # A base is a resource, not a set of virtual joints.
    assert drone.joint_names() == ()
    # Linear axes are m/s and angular axes rad/s. This fixture is a reference
    # someone will copy, so the units have to be physically right.
    assert all(drone.unit_of(f"drone/body/{a}") is Unit.M_PER_S for a in linear)
    assert all(drone.unit_of(f"drone/body/{a}") is Unit.RAD_PER_S for a in angular)


def test_a_base_twist_is_one_group(chassis: ControlDescription) -> None:
    result = validate_command(
        chassis,
        command("c", {"chassis/base/vx": 0.5, "chassis/base/vy": 0.0, "chassis/base/wz": 0.1}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.active_groups == frozenset({"twist"})


def test_partial_group_coverage_is_allowed(xarm: ControlDescription) -> None:
    """Commanding two of seven joints is sparse, not incomplete."""
    result = validate_command(
        xarm,
        command("c", {"arm/joint3/position": 0.1}),
        current_epoch=1,
        last_sequence=None,
    )

    assert isinstance(result, CommandBatch)
    assert result.active_groups == frozenset({"position"})
