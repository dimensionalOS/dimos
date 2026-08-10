# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

"""GripperControlTask unit tests."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_gripper_joints,
    make_joints,
)
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.gripper_task.gripper_task import (
    GripperControlTask,
    GripperControlTaskConfig,
    create_task,
)
from dimos.msgs.std_msgs.Bool import Bool
from dimos.teleop.quest.quest_types import Buttons

# A jaw whose native unit is the xArm's dimensionless SDK scale.
_JAW_LIMITS = [(0.0, 850.0)]


def _task(**overrides: object) -> GripperControlTask:
    limits = overrides.pop("limits", _JAW_LIMITS)
    names = overrides.pop("joint_names", ["arm/gripper"])
    return GripperControlTask(
        "arm_gripper",
        GripperControlTaskConfig(joint_names=list(names), **overrides),  # type: ignore[arg-type]
        limits=list(limits),  # type: ignore[arg-type]
    )


def _state(t_now: float = 0.0, **measured: float) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions=dict(measured)),
        t_now=t_now,
    )


class TestConversionHappensHere:
    def test_normalized_zero_lands_on_the_closed_limit(self) -> None:
        task = _task()

        task.set_normalized([0.0], t_now=0.0)
        assert task.compute(_state()).positions == [0.0]

        task.set_normalized([1.0], t_now=0.0)
        assert task.compute(_state()).positions == [850.0]

        task.set_normalized([0.5], t_now=0.0)
        assert task.compute(_state()).positions == [425.0]

    def test_set_position_passes_native_units_through(self) -> None:
        task = _task()
        task.set_position([612.0], t_now=0.0)
        assert task.compute(_state()).positions == [612.0]

    def test_out_of_range_is_clamped_to_the_declared_travel(self) -> None:
        task = _task()
        task.set_position([9999.0], t_now=0.0)
        assert task.compute(_state()).positions == [850.0]

    def test_emits_servo_position_so_it_never_fights_the_arm(self) -> None:
        task = _task()
        task.set_normalized([1.0], t_now=0.0)
        assert task.compute(_state()).mode is ControlMode.SERVO_POSITION
        assert task.claim().mode is ControlMode.SERVO_POSITION


class TestSweep:
    def test_single_jaw_derives_its_grasp_from_the_closed_limit(self) -> None:
        task = _task()
        task.set_sweep(0.0, t_now=0.0)
        assert task.compute(_state()).positions == [0.0]
        task.set_sweep(1.0, t_now=0.0)
        assert task.compute(_state()).positions == [850.0]

    def test_multi_joint_refuses_without_a_reference_pose(self) -> None:
        task = _task(
            joint_names=make_gripper_joints("hand", 3),
            limits=[(0.0, 1.0)] * 3,
        )

        assert task.set_sweep(0.0, t_now=0.0) is False
        assert task.compute(_state()) is None, "a refused sweep must command nothing"

    def test_multi_joint_sweeps_towards_the_configured_grasp(self) -> None:
        task = _task(
            joint_names=make_gripper_joints("hand", 3),
            limits=[(0.0, 1.0)] * 3,
            reference_pose=[0.8, 0.2, 0.5],
        )

        task.set_sweep(0.0, t_now=0.0)
        assert task.compute(_state()).positions == pytest.approx([0.8, 0.2, 0.5])
        task.set_sweep(1.0, t_now=0.0)
        assert task.compute(_state()).positions == pytest.approx([1.0, 1.0, 1.0])

    def test_a_grasp_planner_can_replace_the_reference_at_runtime(self) -> None:
        task = _task(
            joint_names=make_gripper_joints("hand", 2),
            limits=[(0.0, 1.0)] * 2,
        )
        assert task.set_sweep(0.0, t_now=0.0) is False

        assert task.set_reference_pose([0.3, 0.7])
        assert task.set_sweep(0.0, t_now=0.0)
        assert task.compute(_state()).positions == pytest.approx([0.3, 0.7])


class TestHoldAndActivity:
    def test_always_active_so_get_position_never_goes_stale(self) -> None:
        task = _task()
        assert task.is_active() is True, "must tick even with nothing to command"

        # Idle — never commanded — yet the snapshot still arrives.
        assert task.compute(_state(t_now=1.0, **{"arm/gripper": 400.0})) is None
        assert task.get_position() == [400.0]

        # And it keeps refreshing while idle.
        task.compute(_state(t_now=2.0, **{"arm/gripper": 401.0}))
        assert task.get_position() == [401.0]
        assert task.is_active() is True

    def test_zero_hold_holds_indefinitely(self) -> None:
        task = _task(hold_duration=0.0)
        task.set_normalized([1.0], t_now=0.0)

        for t in (1.0, 100.0, 10_000.0):
            assert task.compute(_state(t_now=t)).positions == [850.0]

    def test_bounded_hold_stops_emitting_when_it_expires(self) -> None:
        task = _task(hold_duration=0.5)
        task.set_normalized([1.0], t_now=10.0)

        assert task.compute(_state(t_now=10.4)).positions == [850.0]
        assert task.compute(_state(t_now=10.6)) is None

    def test_never_stops_because_the_gripper_reached_its_target(self) -> None:
        # A stalled gripper never reaches its target; reaching it must not
        # end the hold either.
        task = _task(hold_duration=0.0)
        task.set_normalized([1.0], t_now=0.0)

        for t in range(1, 6):
            out = task.compute(_state(t_now=float(t), **{"arm/gripper": 850.0}))
            assert out is not None, "reaching the target must not stop the hold"
            assert out.positions == [850.0]

    def test_a_command_without_a_clock_is_stamped_on_the_next_tick(self) -> None:
        task = _task(hold_duration=1.0)
        task.set_normalized([1.0])  # no t_now, as a bare task_invoke would

        assert task.compute(_state(t_now=500.0)).positions == [850.0]
        assert task.compute(_state(t_now=500.9)) is not None
        assert task.compute(_state(t_now=501.1)) is None


class TestCommandsNeverBlock:
    def test_every_declared_command_returns_immediately(self) -> None:
        import time

        task = _task()
        calls = [
            lambda: task.set_position([100.0]),
            lambda: task.set_normalized([0.5]),
            lambda: task.set_sweep(0.5),
            lambda: task.set_reference_pose([10.0]),
            task.get_position,
            task.get_state,
        ]
        for call in calls:
            start = time.perf_counter()
            call()
            assert time.perf_counter() - start < 0.01, f"{call} blocked"

    def test_arity_mismatch_is_refused_not_raised(self) -> None:
        task = _task()
        assert task.set_position([1.0, 2.0]) is False
        assert task.set_normalized([]) is False
        assert task.compute(_state()) is None

    def test_get_normalized_maps_measured_to_fraction(self) -> None:
        task = _task()
        assert task.get_normalized() is None

        task.compute(_state(**{"arm/gripper": 425.0}))
        assert task.get_normalized() == pytest.approx([0.5])

        task.compute(_state(**{"arm/gripper": 9999.0}))
        assert task.get_normalized() == [1.0]

    def test_get_state_carries_the_range_so_callers_need_no_second_rpc(self) -> None:
        task = _task()
        state = task.get_state()
        assert state["limits"] == [(0.0, 850.0)]
        assert state["joints"] == ["arm/gripper"]
        assert state["holding"] is False

        task.set_normalized([1.0], t_now=0.0)
        assert task.get_state()["holding"] is True
        assert task.get_state()["target"] == [850.0]


class TestTriggerAndToggle:
    @staticmethod
    def _buttons(*, primary: bool, trigger: float, hand: str = "right") -> Buttons:
        b = Buttons()
        setattr(b, f"{hand}_primary", primary)
        left, right = (trigger, 0.0) if hand == "left" else (0.0, trigger)
        b.pack_analog_triggers(left, right)
        return b

    def test_squeezing_the_trigger_closes(self) -> None:
        task = _task(hand="right")

        task.on_teleop_buttons(self._buttons(primary=True, trigger=1.0), 0.0)
        assert task.compute(_state()).positions == pytest.approx([0.0], abs=7.0)

        task.on_teleop_buttons(self._buttons(primary=True, trigger=0.0), 0.0)
        assert task.compute(_state()).positions == pytest.approx([850.0], abs=7.0)

    def test_the_trigger_is_ignored_while_disengaged(self) -> None:
        task = _task(hand="right")

        assert task.on_teleop_buttons(self._buttons(primary=False, trigger=1.0), 0.0) is False
        assert task.compute(_state()) is None

    def test_engagement_gate_can_be_turned_off(self) -> None:
        task = _task(hand="right", require_engagement=False)
        assert task.on_teleop_buttons(self._buttons(primary=False, trigger=1.0), 0.0)
        assert task.compute(_state()).positions == pytest.approx([0.0], abs=7.0)

    def test_only_the_configured_hand_drives_this_gripper(self) -> None:
        task = _task(hand="left")

        # Right hand fully engaged and squeezing; this is the LEFT gripper.
        b = Buttons()
        b.left_primary = True  # left is engaged, so the gate is open
        b.right_primary = True
        b.pack_analog_triggers(0.0, 1.0)  # only the right trigger is squeezed

        assert task.on_teleop_buttons(b, 0.0)
        assert task.compute(_state()).positions == [850.0], "must read its own hand only"

    def test_a_press_on_the_other_hand_does_not_engage_this_one(self) -> None:
        task = _task(hand="left")
        assert (
            task.on_teleop_buttons(self._buttons(primary=True, trigger=1.0, hand="right"), 0.0)
            is False
        )
        assert task.compute(_state()) is None

    def test_a_gripper_with_no_hand_ignores_triggers(self) -> None:
        task = _task()
        assert task.on_teleop_buttons(self._buttons(primary=True, trigger=1.0), 0.0) is False

    def test_the_toggle_is_a_wish_not_a_value(self) -> None:
        task = _task()

        task.on_gripper_command(Bool(data=True), 0.0)
        assert task.compute(_state()).positions == [0.0]

        task.on_gripper_command(Bool(data=False), 0.0)
        assert task.compute(_state()).positions == [850.0]


class TestEstop:
    def test_estop_stops_emitting_and_drops_the_target(self) -> None:
        task = _task()
        task.set_normalized([1.0], t_now=0.0)
        assert task.compute(_state()) is not None

        task.set_estop(True)
        assert task.compute(_state()) is None
        assert task.set_normalized([1.0], t_now=0.0) is False, "latched: no replay"

        task.set_estop(False)
        assert task.compute(_state()) is None, "must not resume a pre-estop target"
        task.set_normalized([1.0], t_now=0.0)
        assert task.compute(_state()) is not None


class TestLimitResolution:
    @staticmethod
    def _hardware(gripper_dof: int = 1, dof: int = 6, limit_len: int | None = None):
        component = HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            all_joints=[*make_joints("arm", dof), *make_gripper_joints("arm", gripper_dof)]
            if gripper_dof
            else make_joints("arm", dof),
            gripper_dof=gripper_dof,
        )
        n = limit_len if limit_len is not None else dof + gripper_dof
        adapter = SimpleNamespace(
            get_limits=lambda: SimpleNamespace(
                position_lower=[0.0] * n,
                position_upper=[*([3.14] * dof), *([0.08] * gripper_dof)][:n],
                velocity_max=[0.0] * n,
            )
        )
        return {"arm": SimpleNamespace(component=component, adapter=adapter)}

    @staticmethod
    def _cfg(joint_names: list[str], name: str = "arm_gripper"):
        return SimpleNamespace(name=name, joint_names=joint_names, priority=20, params={})

    def test_takes_the_trailing_slice_by_the_declared_count(self) -> None:
        task = create_task(self._cfg(["arm/gripper"]), self._hardware())
        assert task.get_state()["limits"] == [(0.0, 0.08)]

    def test_rejects_joints_that_are_not_that_component_s_gripper(self) -> None:
        with pytest.raises(ValueError, match="must equal"):
            create_task(self._cfg(["arm/joint1"]), self._hardware())

    def test_rejects_a_task_spanning_two_devices(self) -> None:
        with pytest.raises(ValueError, match="exactly one device"):
            create_task(self._cfg(["arm/gripper", "other/gripper"]), self._hardware())

    def test_rejects_unregistered_hardware(self) -> None:
        with pytest.raises(ValueError, match="no hardware"):
            create_task(self._cfg(["ghost/gripper"]), self._hardware())

    def test_rejects_an_adapter_whose_limits_omit_the_gripper(self) -> None:
        with pytest.raises(ValueError, match="limit entries"):
            create_task(self._cfg(["arm/gripper"]), self._hardware(limit_len=6))


class TestOwnership:
    def test_claims_exactly_its_gripper_joints(self) -> None:
        task = _task()
        assert task.claim().joints == frozenset({"arm/gripper"})

    def test_a_task_with_no_joints_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="at least one joint"):
            _task(joint_names=[])
