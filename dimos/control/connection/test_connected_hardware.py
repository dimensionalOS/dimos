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

"""Tests for ``ConnectedHardware`` and the stop calculations it uses.

Everything runs on a fake clock with no background threads: a test steps the
hardware by calling ``supervise`` and ``ingest_state`` itself. Timings are
powers of two, so "exactly at the timeout" is exact and not a rounding error.
"""

from __future__ import annotations

from dataclasses import replace
import math
import pickle
import threading
import time

import pytest

from dimos.control.connection.conftest import (
    ARM,
    FakeHooks,
    FakeModule,
    arm_description,
    arm_position_command,
    arm_reading,
    arm_velocity_command,
    base_description,
    fast_hooks,
    full_reading,
    pd_description,
)
from dimos.control.connection.connected_hardware import (
    ConnectedHardware,
    Frame,
    missing_command_keys,
)
from dimos.control.connection.safety import (
    damp_values,
    hold_values,
    ramp_values,
    zero_values,
)
from dimos.control.connection.status import LifecycleAck, LifecycleState
from dimos.control.contract.description import (
    ActivationPolicy,
    Estop,
    EstopKind,
    EstopRecovery,
    SafeStop,
    SafeStopKind,
    Timing,
)
from dimos.control.contract.validate import CommandBatch, DescriptionError, validate_command
from dimos.msgs.control_msgs.ControlValues import ControlValues

#: 64 readings a second, stale after half a second, watchdog after a quarter.
#: All exact in binary.
EXACT = Timing(state_rate_hz=64.0, stale_timeout_s=0.5, watchdog_timeout_s=0.25)
PERIOD = 1.0 / 64.0


def exact(desc):
    return replace(desc, timing=EXACT)


def base_command(epoch, sequence, vx=1.0, vy=0.5, wz=0.25):
    return ControlValues(
        source="coordinator",
        epoch=epoch,
        sequence=sequence,
        interface_names=["base/base/vx", "base/base/vy", "base/base/wz"],
        values=[vx, vy, wz],
    )


def pd_command(epoch, sequence):
    names, values = [], []
    for j in ARM:
        for interface, value in (("position", 0.3), ("effort", 1.0), ("kp", 60.0), ("kd", 1.5)):
            names.append(f"g1/{j}/{interface}")
            values.append(value)
    return ControlValues(
        source="coordinator", epoch=epoch, sequence=sequence, interface_names=names, values=values
    )


# Starting up


def test_start_connects_and_checks_the_description(rig_for):
    rig = rig_for(arm_description())
    assert rig.hooks.connected
    assert rig.hw.state is LifecycleState.STANDBY
    assert rig.hw.describe_control().epoch == 7


def test_start_refuses_a_broken_description():
    bad = arm_description(
        timing=Timing(state_rate_hz=100, stale_timeout_s=0.001, watchdog_timeout_s=0.1)
    )
    hw = ConnectedHardware(FakeModule(), FakeHooks(bad), state_mode="push")
    with pytest.raises(DescriptionError, match="stale_timeout_s"):
        hw.start(background=False)


def test_start_refuses_an_emergency_stop_only_the_driver_could_do():
    desc = pd_description(estop=Estop(kind=EstopKind.DISABLE, recovery=EstopRecovery.CLEAR))
    hw = ConnectedHardware(FakeModule(), FakeHooks(desc), state_mode="push")
    with pytest.raises(ValueError, match="needs an on_estop hook"):
        hw.start(background=False)


def test_start_refuses_polling_with_no_way_to_read():
    hooks = FakeHooks(arm_description())
    hooks.read_state = None
    hw = ConnectedHardware(FakeModule(), hooks, state_mode="poll")
    with pytest.raises(ValueError, match="needs a read_state hook"):
        hw.start(background=False)


# Arming


@pytest.mark.parametrize("policy", list(ActivationPolicy))
def test_prepare_then_commit_arms_whatever_the_activation_policy(rig_for, policy):
    # The policy only changes whether the caller asks a person first. Here it
    # is the same two steps either way.
    seen = []
    rig = rig_for(
        arm_description(activation_policy=policy), on_prepare_arm=lambda: seen.append(rig.hw.state)
    )
    rig.feed()
    assert rig.hw.prepare_arm(1).ok
    assert seen == [LifecycleState.PREPARING]
    assert rig.hw.state is LifecycleState.PREPARED
    ack = rig.hw.commit_arm(2, epoch=5)
    assert ack == LifecycleAck(2, "arm", LifecycleState.ARMED, True)
    assert rig.hw.status().epoch == 5


def test_commit_needs_fresh_readings(rig_for):
    rig = rig_for(exact(arm_description()))
    rig.feed()
    assert rig.hw.prepare_arm(1).ok
    rig.clock.advance(0.5 + PERIOD)
    ack = rig.hw.commit_arm(2, epoch=1)
    assert not ack.ok and ack.reason == "no fresh readings from the hardware"
    assert ack.state is LifecycleState.PREPARED


def test_commit_refused_while_the_hardware_reports_a_fault(rig_for):
    rig = rig_for(arm_description(), fault=lambda: "overheated")
    rig.feed()
    rig.hw.prepare_arm(1)
    ack = rig.hw.commit_arm(2, epoch=1)
    assert not ack.ok and "overheated" in ack.reason


def test_a_failed_prepare_leaves_the_robot_stopped(rig_for):
    def refuse():
        raise RuntimeError("could not stand up")

    rig = rig_for(arm_description(), on_prepare_arm=refuse)
    ack = rig.hw.prepare_arm(1)
    assert not ack.ok and "could not stand up" in ack.reason
    assert rig.hw.state is LifecycleState.SAFE_STOPPED


def test_abort_goes_back_to_standby_only_before_arming(rig_for):
    rig = rig_for(arm_description())
    rig.feed()
    rig.hw.prepare_arm(1)
    assert rig.hw.abort_arm(2).ok
    assert rig.hw.state is LifecycleState.STANDBY
    rig.arm()
    ack = rig.hw.abort_arm(9)
    assert not ack.ok and ack.state is LifecycleState.ARMED


def test_a_retried_request_gets_the_same_answer(rig_for):
    rig = rig_for(arm_description())
    rig.feed()
    rig.hw.prepare_arm(1)
    first = rig.hw.commit_arm(5, epoch=1)
    assert rig.hw.commit_arm(5, epoch=1) == first
    assert not rig.hw.commit_arm(6, epoch=1).ok


def test_a_wrong_state_request_answers_instead_of_raising(rig_for):
    rig = rig_for(arm_description())
    ack = rig.hw.commit_arm(1, epoch=1)
    assert not ack.ok and ack.state is LifecycleState.STANDBY and "standby" in ack.reason


# Commands


def test_a_complete_command_is_written(rig_for):
    rig = rig_for(arm_description())
    rig.arm(epoch=3)
    rig.send(arm_position_command(epoch=3, sequence=1))
    assert rig.hooks.written == [
        Frame(
            "arm",
            {"arm/j1/position": 0.1, "arm/j2/position": 0.2, "arm/gripper/position": 0.04},
            frozenset({"position", "gripper"}),
            3,
            1,
        )
    ]


def test_commands_are_dropped_and_counted_when_not_armed(rig_for):
    rig = rig_for(arm_description())
    rig.send(arm_position_command(epoch=1, sequence=1))
    assert rig.hooks.written == []
    assert rig.rejections() == {"not_armed": 1}


def test_a_command_for_other_hardware_is_ignored_without_counting(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.send(base_command(epoch=1, sequence=1))
    assert rig.hooks.written == []
    assert rig.rejections() == {}


def test_commands_from_another_arming_or_out_of_order_are_refused(rig_for):
    rig = rig_for(arm_description())
    rig.arm(epoch=2)
    rig.send(arm_position_command(epoch=1, sequence=1))
    rig.send(arm_position_command(epoch=2, sequence=4))
    rig.send(arm_position_command(epoch=2, sequence=4))
    rig.send(arm_position_command(epoch=2, sequence=3))
    assert len(rig.hooks.written) == 1
    assert rig.rejections() == {"epoch": 1, "sequence": 2}


def test_an_incomplete_command_is_refused_whole_and_names_what_is_missing(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    partial = ControlValues(
        source="coordinator",
        epoch=1,
        sequence=1,
        interface_names=["arm/j1/position", "arm/gripper/position"],
        values=[0.1, 0.04],
    )
    rig.send(partial)
    assert rig.hooks.written == []
    assert rig.rejections() == {"incomplete": 1}


def test_values_the_robot_leaves_unset_may_be_missing(rig_for):
    rig = rig_for(pd_description())
    rig.arm()
    rig.send(pd_command(epoch=1, sequence=1))
    assert len(rig.hooks.written) == 1
    assert "g1/j1/velocity" not in rig.hooks.written[0].values


def test_driving_by_position_and_driving_by_speed_are_each_complete(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    rig.send(arm_velocity_command(epoch=1, sequence=2))
    assert [frame.active_groups for frame in rig.hooks.written] == [
        frozenset({"position", "gripper"}),
        frozenset({"velocity", "gripper"}),
    ]


def test_every_command_value_at_once_can_never_be_sent():
    # Position and speed for the same joint are two ways of driving it, and
    # only one may be used at a time. So "complete" cannot mean "every
    # command value": it means every part driven, in one way, fully.
    desc = arm_description()
    keys = list(desc.command_keys())
    frame = ControlValues(
        source="c", epoch=1, sequence=1, interface_names=keys, values=[0.0] * len(keys)
    )
    assert validate_command(desc, frame, current_epoch=1, last_sequence=None).reason == "mode_group"


def test_missing_command_keys_names_undriven_parts_and_missing_values():
    desc = arm_description()
    only_joints = CommandBatch(
        values={"arm/j1/position": 0.1, "arm/j2/position": 0.2},
        active_groups=frozenset({"position"}),
    )
    assert missing_command_keys(desc, only_joints) == ["arm/gripper"]
    one_joint = CommandBatch(
        values={"arm/j1/position": 0.1, "arm/gripper/position": 0.0},
        active_groups=frozenset({"position", "gripper"}),
    )
    assert missing_command_keys(desc, one_joint) == ["arm/j2/position"]


# Switching how the hardware is driven


def test_switching_happens_once_per_change_and_the_same_command_is_then_sent(rig_for):
    switched = []
    rig = rig_for(arm_description(), set_native=switched.append)
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    rig.send(arm_position_command(epoch=1, sequence=2))
    rig.send(arm_velocity_command(epoch=1, sequence=3))
    # The gripper can always be driven, so it is never switched.
    assert switched == ["position", "velocity"]
    assert [frame.sequence for frame in rig.hooks.written] == [1, 2, 3]
    assert rig.hw.status().confirmed_groups == frozenset({"velocity", "gripper"})


def test_a_failed_switch_stops_the_robot_and_sends_nothing(rig_for):
    def refuse(group):
        raise RuntimeError("mode refused")

    rig = rig_for(arm_description(), set_native=refuse)
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    assert rig.hw.state is LifecycleState.SAFE_STOPPED
    assert rig.hw.status().fault == "set_native(position): RuntimeError: mode refused"
    assert rig.hooks.written == []


def test_a_switch_that_hangs_stops_the_robot(rig_for):
    release = threading.Event()
    rig = rig_for(fast_hooks(arm_description()), set_native=lambda group: release.wait(5))
    rig.arm()
    try:
        rig.send(arm_position_command(epoch=1, sequence=1))
        assert rig.hw.state is LifecycleState.SAFE_STOPPED
        assert rig.hw.status().fault == "set_native(position): hook_timeout"
    finally:
        release.set()


# Writes that fail


def test_three_failed_writes_in_a_row_stop_the_robot(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.hooks.fail_writes = 3
    for sequence in (1, 2):
        rig.send(arm_position_command(epoch=1, sequence=sequence))
        assert rig.hw.state is LifecycleState.ARMED
    rig.send(arm_position_command(epoch=1, sequence=3))
    assert rig.hw.state is LifecycleState.SAFE_STOPPED
    assert rig.hw.status().fault == "write_failed"


def test_a_successful_write_resets_the_count(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.hooks.fail_writes = 2
    for sequence in (1, 2, 3):
        rig.send(arm_position_command(epoch=1, sequence=sequence))
    rig.hooks.fail_writes = 2
    for sequence in (4, 5):
        rig.send(arm_position_command(epoch=1, sequence=sequence))
    assert rig.hw.state is LifecycleState.ARMED


# Stopping by itself


def test_the_watchdog_fires_after_the_timeout_and_not_at_it(rig_for):
    rig = rig_for(exact(arm_description()))
    rig.arm()
    rig.clock.advance(0.25)
    rig.feed()
    rig.hw.supervise()
    assert rig.hw.state is LifecycleState.ARMED
    rig.clock.advance(PERIOD)
    rig.feed()
    rig.hw.supervise()
    assert rig.hw.state is LifecycleState.SAFE_STOPPED
    assert rig.hw.status().fault == "watchdog"


def test_each_command_resets_the_watchdog(rig_for):
    rig = rig_for(exact(arm_description()))
    rig.arm()
    for sequence in range(1, 5):
        rig.clock.advance(0.25)
        rig.feed()
        rig.send(arm_position_command(epoch=1, sequence=sequence))
        rig.hw.supervise()
    assert rig.hw.state is LifecycleState.ARMED


def test_a_hardware_fault_stops_the_robot(rig_for):
    fault = [None]
    rig = rig_for(arm_description(), fault=lambda: fault[0])
    rig.arm()
    fault[0] = "encoder lost"
    rig.hw.supervise()
    assert rig.hw.state is LifecycleState.SAFE_STOPPED
    assert rig.hw.status().fault == "encoder lost"


@pytest.mark.parametrize("state_mode", ["push", "poll"])
def test_readings_that_stop_arriving_stop_the_robot(rig_for, state_mode):
    rig = rig_for(exact(arm_description()), state_mode=state_mode)
    rig.hooks.sample = (arm_reading(), 0.0)
    rig.arm()
    rig.hooks.sample = None
    rig.clock.advance(0.5 + PERIOD)
    rig.hw.poll_state()
    rig.hw.supervise()
    assert rig.hw.state is LifecycleState.SAFE_STOPPED
    assert rig.hw.status().fault == "state_stale"


# How it stops


def test_a_hold_stops_a_robot_driven_by_speed(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.send(arm_velocity_command(epoch=1, sequence=1, speeds=(0.5, -0.5)))
    rig.hw.safe_stop(10)
    assert rig.hooks.written[-1].values == {
        "arm/j1/velocity": 0.0,
        "arm/j2/velocity": 0.0,
        "arm/gripper/position": 0.04,
    }


def test_a_hold_keeps_a_robot_driven_by_position_where_it_was_told(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    rig.hw.safe_stop(10)
    assert rig.hooks.written[-1].values == rig.hooks.written[-2].values


def test_a_hold_with_nothing_commanded_sends_nothing(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.hw.safe_stop(10)
    assert rig.hooks.written == []


def test_a_zero_stop_sets_every_speed_to_zero(rig_for):
    rig = rig_for(base_description())
    rig.arm()
    rig.send(base_command(epoch=1, sequence=1))
    rig.hw.safe_stop(10)
    assert rig.hooks.written[-1].values == {
        "base/base/vx": 0.0,
        "base/base/vy": 0.0,
        "base/base/wz": 0.0,
    }


def test_a_gradual_stop_slows_down_over_its_ramp(rig_for):
    ramp = SafeStop(kind=SafeStopKind.ZERO_RAMP, ramp_s=0.5)
    rig = rig_for(exact(base_description(safe_stop=ramp)))
    rig.arm()
    rig.send(base_command(epoch=1, sequence=1, vx=1.0, vy=0.5, wz=0.25))
    rig.hw.safe_stop(10)
    assert rig.hooks.written[-1].values["base/base/vx"] == 1.0
    rig.clock.advance(0.25)
    rig.hw.supervise()
    assert rig.hooks.written[-1].values == {
        "base/base/vx": 0.5,
        "base/base/vy": 0.25,
        "base/base/wz": 0.125,
    }
    rig.clock.advance(0.5)
    rig.hw.supervise()
    assert rig.hooks.written[-1].values == {
        "base/base/vx": 0.0,
        "base/base/vy": 0.0,
        "base/base/wz": 0.0,
    }


def test_a_damping_stop_goes_slack_and_resists_movement(rig_for):
    rig = rig_for(pd_description())
    rig.arm()
    rig.send(pd_command(epoch=1, sequence=1))
    rig.hw.safe_stop(10)
    expected = {}
    for j in ARM:
        expected |= {
            f"g1/{j}/position": 0.3,
            f"g1/{j}/effort": 0.0,
            f"g1/{j}/kp": 0.0,
            f"g1/{j}/kd": 5.0,
        }
    assert rig.hooks.written[-1].values == expected
    assert rig.hooks.written[-1].active_groups == frozenset({"pd"})


def test_a_damping_stop_before_any_command_uses_the_readings(rig_for):
    rig = rig_for(pd_description())
    reading = full_reading(rig.hw.describe_control()) | {"g1/j1/position": 0.7}
    rig.feed(reading)
    rig.hw.prepare_arm(1)
    rig.hw.commit_arm(2, epoch=1)
    rig.hw.safe_stop(10)
    stop = rig.hooks.written[-1]
    assert stop.values["g1/j1/position"] == 0.7
    assert stop.values["g1/j1/kp"] == 0.0 and stop.values["g1/j1/kd"] == 5.0
    assert "g1/j1/velocity" not in stop.values
    assert stop.active_groups == frozenset({"pd"})


def test_the_drivers_own_stop_runs_after_the_stop_is_sent(rig_for):
    order = []
    rig = rig_for(arm_description(), on_safe_stop=lambda: order.append("hook"))
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    written_before = len(rig.hooks.written)
    rig.hooks.write = lambda frame: order.append("frame")
    rig.hw.safe_stop(10)
    assert written_before == 1
    assert order == ["frame", "hook"]


def test_a_stopped_robot_keeps_being_told_to_stop_at_the_reading_rate(rig_for):
    rig = rig_for(exact(arm_description()))
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    rig.hw.safe_stop(10)
    sent = len(rig.hooks.written)
    rig.clock.advance(PERIOD / 2)
    rig.hw.supervise()
    assert len(rig.hooks.written) == sent
    rig.clock.advance(PERIOD / 2)
    rig.hw.supervise()
    assert len(rig.hooks.written) == sent + 1


def test_stopping_twice_is_harmless(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    first = rig.hw.safe_stop(10)
    again = rig.hw.safe_stop(10)
    assert first.ok and again.ok and again.reason == "already stopped"
    assert rig.hw.status().fault == "requested"


def test_a_stop_can_be_asked_for_before_arming(rig_for):
    rig = rig_for(arm_description())
    assert rig.hw.safe_stop(1).ok
    assert rig.hw.state is LifecycleState.SAFE_STOPPED
    assert not rig.hw.prepare_arm(2).ok


# Emergency stop


def test_an_emergency_stop_ends_the_arming(rig_for):
    rig = rig_for(arm_description())
    rig.arm(epoch=1)
    rig.send(arm_position_command(epoch=1, sequence=1))
    rig.hw.estop(10)
    assert rig.hw.state is LifecycleState.ESTOPPED
    assert rig.hw.status().epoch is None
    # With no driver hook, the description's estop kind (HOLD) is sent.
    assert rig.hooks.written[-1].values == rig.hooks.written[0].values
    rig.send(arm_position_command(epoch=1, sequence=2))
    assert rig.rejections() == {"not_armed": 1}


def test_a_late_command_from_before_an_emergency_stop_is_refused_after_rearming(rig_for):
    rig = rig_for(arm_description())
    rig.arm(epoch=1)
    rig.hw.estop(10)
    assert rig.hw.clear_estop(11).ok
    rig.arm(epoch=2)
    rig.send(arm_position_command(epoch=1, sequence=99))
    rig.send(arm_position_command(epoch=2, sequence=1))
    assert rig.rejections() == {"epoch": 1}
    assert [frame.epoch for frame in rig.hooks.written] == [2]


def test_an_emergency_stop_uses_the_drivers_hook_when_there_is_one(rig_for):
    calls = []
    rig = rig_for(arm_description(), on_estop=lambda: calls.append("estop"))
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    rig.hw.estop(10)
    assert calls == ["estop"]
    assert len(rig.hooks.written) == 1


def test_an_emergency_stop_during_prepare_is_repeated_when_prepare_returns(rig_for):
    calls = []
    rig = rig_for(arm_description(), on_estop=lambda: calls.append("estop"))
    rig.hooks.on_prepare_arm = lambda: rig.hw.estop(50)
    ack = rig.hw.prepare_arm(1)
    assert not ack.ok and ack.state is LifecycleState.ESTOPPED
    assert calls == ["estop", "estop"]


def test_an_emergency_stop_takes_over_from_a_safe_stop(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.hw.safe_stop(1)
    rig.hw.estop(2)
    assert rig.hw.state is LifecycleState.ESTOPPED
    assert rig.hw.safe_stop(3).reason == "already emergency-stopped"


# Clearing a stop


def test_clearing_lands_in_standby_and_arming_starts_again(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.hw.safe_stop(10)
    assert rig.hw.clear_safe_stop(11).ok
    assert rig.hw.state is LifecycleState.STANDBY
    assert rig.hw.status().fault is None
    rig.send(arm_position_command(epoch=1, sequence=1))
    assert rig.rejections() == {"not_armed": 1}
    rig.arm(epoch=2)
    assert rig.hw.state is LifecycleState.ARMED


def test_clearing_is_refused_while_the_hardware_reports_a_fault(rig_for):
    fault = [None]
    rig = rig_for(arm_description(), fault=lambda: fault[0])
    rig.arm()
    fault[0] = "estop button still pressed"
    rig.hw.supervise()
    ack = rig.hw.clear_safe_stop(10)
    assert not ack.ok and "estop button still pressed" in ack.reason
    fault[0] = None
    assert rig.hw.clear_safe_stop(11).ok


def test_clearing_the_wrong_kind_of_stop_is_refused(rig_for):
    rig = rig_for(arm_description())
    rig.hw.safe_stop(1)
    ack = rig.hw.clear_estop(2)
    assert not ack.ok and ack.state is LifecycleState.SAFE_STOPPED


def test_clearing_calls_the_drivers_recovery(rig_for):
    latches = []
    rig = rig_for(arm_description(), clear_fault=latches.append)
    rig.hw.estop(1)
    rig.hw.clear_estop(2)
    assert latches == ["estop"]


# Readings


def test_readings_are_published_under_the_hardware_name(rig_for):
    rig = rig_for(arm_description())
    rig.feed(arm_reading((0.5, 0.25)))
    [frame] = rig.module.control_state.published
    assert frame.source == "arm" and frame.epoch == 7 and frame.sequence == 0
    assert frame.as_dict()["arm/j1/position"] == 0.5


def test_readings_that_arrive_while_connecting_are_dropped_quietly():
    # A driver's reading thread often starts inside connect(), before there is
    # a description to check readings against.
    module, hooks = FakeModule(), FakeHooks(arm_description())
    hw = ConnectedHardware(module, hooks, state_mode="push")
    hooks.connect = lambda: hw.ingest_state(arm_reading(), 0.0)
    hw.start(background=False)
    assert module.control_state.published == []
    hw.stop()


def test_polling_publishes_only_when_there_is_something_new(rig_for):
    rig = rig_for(arm_description(), state_mode="poll")
    rig.hooks.sample = None
    rig.hw.poll_state()
    rig.hooks.sample = (arm_reading(), 12.5)
    rig.hw.poll_state()
    assert [frame.source_ts for frame in rig.module.control_state.published] == [12.5]


@pytest.mark.parametrize(
    "change",
    [
        lambda r: r.pop("arm/j1/effort"),
        lambda r: r.update({"arm/j1/velocity": 0.0}),
        lambda r: r.update({"arm/j1/position": math.nan}),
    ],
    ids=["missing", "extra", "not_a_number"],
)
def test_bad_readings_are_counted_and_never_published(rig_for, change):
    rig = rig_for(arm_description())
    reading = arm_reading()
    change(reading)
    rig.feed(reading)
    assert rig.module.control_state.published == []
    assert rig.rejections() == {"bad_state": 1}


# Two pieces of hardware on one pair of ports


def test_two_pieces_of_hardware_share_the_ports_and_keep_to_their_own(rig_for):
    module = FakeModule()
    arm_hooks, base_hooks = FakeHooks(arm_description()), FakeHooks(base_description())
    arm = ConnectedHardware(module, arm_hooks, state_mode="push", clock=lambda: 0.0)
    base = ConnectedHardware(module, base_hooks, state_mode="push", clock=lambda: 0.0)
    for hw in (arm, base):
        hw.start(background=False)
        hw.ingest_state(full_reading(hw.describe_control()), 0.0)
        hw.prepare_arm(1)
        hw.commit_arm(2, epoch=1)
    both = arm_position_command(epoch=1, sequence=1)
    combined = ControlValues(
        source="coordinator",
        epoch=1,
        sequence=1,
        interface_names=[*both.interface_names, "base/base/vx", "base/base/vy", "base/base/wz"],
        values=[*both.values, 1.0, 0.0, 0.0],
    )
    module.control_command.send(combined)
    assert set(arm_hooks.written[0].values) == set(both.interface_names)
    assert set(base_hooks.written[0].values) == {"base/base/vx", "base/base/vy", "base/base/wz"}
    assert [frame.source for frame in module.control_state.published] == ["arm", "base"]
    for hw in (arm, base):
        hw.stop()


# Reporting


def test_status_and_description_survive_being_sent_between_processes(rig_for):
    rig = rig_for(arm_description(), status_extras=lambda: {"mode": 1})
    rig.arm()
    status = rig.hw.status()
    assert status.extras == {"mode": 1}
    assert pickle.loads(pickle.dumps(status)) == status
    desc = rig.hw.describe_control()
    assert pickle.loads(pickle.dumps(desc)) == desc
    ack = rig.hw.safe_stop(3)
    assert pickle.loads(pickle.dumps(ack)) == ack


def test_status_reports_freshness_and_rejections(rig_for):
    rig = rig_for(exact(arm_description()))
    rig.arm()
    rig.send(arm_position_command(epoch=9, sequence=1))
    status = rig.hw.status()
    assert status.state_fresh and status.command_fresh
    assert status.rejections == (("epoch", 1),)
    rig.clock.advance(1.0)
    status = rig.hw.status()
    assert not status.state_fresh and not status.command_fresh


def test_redescribing_is_allowed_only_in_standby_and_bumps_the_number(rig_for):
    rig = rig_for(arm_description())
    assert rig.hw.redescribe().epoch == 8
    rig.feed()
    assert rig.module.control_state.published[-1].epoch == 8
    rig.arm()
    with pytest.raises(RuntimeError, match="standby"):
        rig.hw.redescribe()


# The background loops, on real time


def test_the_background_loops_read_supervise_and_stop_promptly():
    desc = arm_description(
        timing=Timing(state_rate_hz=200.0, stale_timeout_s=0.05, watchdog_timeout_s=0.1)
    )
    module, hooks = FakeModule(), FakeHooks(desc)
    hooks.sample = (arm_reading(), 0.0)
    hw = ConnectedHardware(module, hooks, state_mode="poll")
    hw.start()
    try:
        deadline = time.monotonic() + 2.0
        while not module.control_state.published and time.monotonic() < deadline:
            time.sleep(0.005)
        assert module.control_state.published, "the reading loop never ran"
        assert hw.prepare_arm(1).ok and hw.commit_arm(2, epoch=1).ok
        # No commands follow, so the watchdog must stop it on its own.
        deadline = time.monotonic() + 2.0
        while hw.state is LifecycleState.ARMED and time.monotonic() < deadline:
            time.sleep(0.005)
        assert hw.status().fault == "watchdog"
    finally:
        began = time.monotonic()
        hw.stop()
        assert time.monotonic() - began < 1.0, "stop() waited on a loop"


# Shutting down


def test_shutting_down_an_armed_robot_stops_it_first(rig_for):
    rig = rig_for(arm_description())
    rig.arm()
    rig.send(arm_position_command(epoch=1, sequence=1))
    rig.hw.stop()
    assert rig.hw.state is LifecycleState.SAFE_STOPPED
    assert len(rig.hooks.written) == 2
    assert rig.hooks.shut_down
    rig.hw.stop()


# The stop calculations on their own

LAST = {"a/j/position": 0.5, "a/j/velocity": 2.0, "a/j/effort": 3.0, "a/j/kp": 60.0}


def test_hold_values_keep_targets_and_forces_but_stop_motion():
    assert hold_values(LAST) == {
        "a/j/position": 0.5,
        "a/j/velocity": 0.0,
        "a/j/effort": 3.0,
        "a/j/kp": 60.0,
    }


def test_zero_values_also_let_go_of_forces():
    assert zero_values(LAST) == {
        "a/j/position": 0.5,
        "a/j/velocity": 0.0,
        "a/j/effort": 0.0,
        "a/j/kp": 60.0,
    }


@pytest.mark.parametrize(
    ("fraction", "speed"), [(1.0, 2.0), (0.25, 0.5), (0.0, 0.0), (-1.0, 0.0), (2.0, 2.0)]
)
def test_ramp_values_keep_the_given_share_of_each_speed(fraction, speed):
    out = ramp_values(LAST, fraction)
    assert out["a/j/velocity"] == speed
    assert out["a/j/effort"] == 0.0 and out["a/j/position"] == 0.5


def test_damp_values_hold_the_parts_that_are_not_damped():
    desc = pd_description()
    last = {"g1/j1/position": 0.1, "g1/j1/effort": 2.0, "g1/j1/kp": 60.0, "g1/j1/kd": 1.5}
    out = damp_values(desc, last, {})
    assert out["g1/j1/kp"] == 0.0 and out["g1/j1/kd"] == 5.0 and out["g1/j1/effort"] == 0.0
    # j2 was never commanded: it gets a full damping frame, position from
    # nothing measured, and its unset speed stays unset.
    assert out["g1/j2/position"] == 0.0 and "g1/j2/velocity" not in out
