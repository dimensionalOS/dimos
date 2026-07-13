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

"""Characterization tests for coordinator task commands.

These pin the observable behavior of ``task_invoke`` and the
``set_activated`` / ``set_dry_run`` fan-outs so the TASK_EXPOSES
card refactor can prove it preserves them. Tasks that declare their
commands (via ``task_type``) keep the exact same observable outcomes
after the refactor; only the mechanism underneath changes.

The stubs are registered under real declaring task types
(``trajectory`` for execute/cancel/get_state, ``g1_groot_wbc`` for
arm/disarm/set_dry_run) so the same tests stay green before and after
the command table exists.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

ARM_JOINTS = frozenset({"arm/joint1", "arm/joint2"})


class CommandRecordingTask(BaseControlTask):
    """Stub task recording every command invocation.

    Carries the trajectory-task commands (execute/cancel/get_state),
    the g1 arming commands (arm/disarm/set_dry_run), plus an undeclared
    ``record_time`` used to pin the ``t_now`` auto-injection contract.
    """

    def __init__(self, name: str, joints: frozenset[str] = ARM_JOINTS) -> None:
        self._name = name
        self._joints = frozenset(joints)
        self.executed: Any = None
        self.cancelled = False
        self.armed: bool | None = None
        self.arm_calls: list[float | None] = []
        self.dry_run: bool | None = None
        self.reset_calls: list[bool | None] = []
        self.t_now_seen: float | None = None
        self._state = "IDLE"

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=self._joints)

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass

    # Trajectory commands
    def execute(self, trajectory: Any) -> bool:
        self.executed = trajectory
        self._state = "EXECUTING"
        return True

    def cancel(self) -> bool:
        self.cancelled = True
        self._state = "ABORTED"
        return True

    def get_state(self) -> str:
        return self._state

    # g1 arming commands
    def arm(self, ramp_seconds: float | None = None) -> bool:
        self.armed = True
        self.arm_calls.append(ramp_seconds)
        return True

    def disarm(self) -> bool:
        self.armed = False
        return True

    def set_dry_run(self, enabled: bool) -> None:
        self.dry_run = bool(enabled)

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        self.reset_calls.append(reactivate)
        return True

    # Undeclared helper (never in any TASK_EXPOSES card)
    def record_time(self, t_now: float | None = None) -> float | None:
        self.t_now_seen = t_now
        return t_now


def _trajectory() -> JointTrajectory:
    return JointTrajectory(
        joint_names=["arm/joint1", "arm/joint2"],
        points=[
            TrajectoryPoint(time_from_start=0.0, positions=[0.0, 0.0]),
            TrajectoryPoint(time_from_start=1.0, positions=[0.1, 0.2]),
        ],
    )


@pytest.fixture
def coordinator(mocker) -> Iterator[ControlCoordinator]:
    """A coordinator with the tick loop stubbed; never started (RPCs are pure)."""
    mocker.patch("dimos.control.coordinator.TickLoop")
    coord = ControlCoordinator(publish_joint_state=False)
    try:
        yield coord
    finally:
        coord.stop()


class TestTaskInvoke:
    def test_execute_cancel_get_state_round_trip(self, coordinator):
        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")

        traj = _trajectory()
        assert coordinator.task_invoke("traj_arm", "execute", {"trajectory": traj}) is True
        assert coordinator.task_invoke("traj_arm", "get_state") == "EXECUTING"
        assert coordinator.task_invoke("traj_arm", "cancel") is True
        assert coordinator.task_invoke("traj_arm", "get_state") == "ABORTED"
        assert task.executed is traj

    def test_injects_t_now_when_none(self, coordinator):
        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")

        result = coordinator.task_invoke("traj_arm", "record_time", {"t_now": None})

        assert isinstance(result, float)
        assert task.t_now_seen == result

    def test_unknown_task_warns_and_returns_none(self, coordinator, mocker):
        import dimos.control.coordinator as coord_mod

        warn = mocker.patch.object(coord_mod.logger, "warning")

        assert coordinator.task_invoke("nope", "execute", {}) is None
        assert warn.called

    def test_unknown_method_raises(self, coordinator):
        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")

        with pytest.raises(AttributeError, match=r"traj_arm.+no_such_method"):
            coordinator.task_invoke("traj_arm", "no_such_method", {})


class TestActivationFanOut:
    def test_set_activated_arms_then_disarms_declaring_task(self, coordinator):
        task = CommandRecordingTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")

        coordinator.set_activated(True)
        assert task.armed is True

        coordinator.set_activated(False)
        assert task.armed is False

    def test_set_dry_run_reaches_declaring_task(self, coordinator):
        task = CommandRecordingTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")

        coordinator.set_dry_run(True)
        assert task.dry_run is True

        coordinator.set_dry_run(False)
        assert task.dry_run is False

    def test_reset_runtime_state_reaches_declaring_task(self, coordinator):
        task = CommandRecordingTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")

        assert coordinator.reset_runtime_state(reactivate=True) == {"g1": True}
        assert task.reset_calls == [True]

    def test_reset_runtime_state_defaults_reactivate_to_none(self, coordinator):
        task = CommandRecordingTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")

        assert coordinator.reset_runtime_state() == {"g1": True}
        assert task.reset_calls == [None]

    def test_restart_keeps_declared_commands_reachable(self, coordinator):
        # add_task() skips known names, so a table cleared on stop() never rebuilds.
        task = CommandRecordingTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")

        coordinator.start()
        coordinator.stop()
        coordinator.start()

        coordinator.set_activated(True)
        assert task.armed is True  # still declared after a stop/start cycle


class TestValidatedDispatch:
    """New behavior introduced by TASK_EXPOSES: validated, declared commands."""

    def test_valid_execute_passes_trajectory_by_identity(self, coordinator):
        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")

        traj = _trajectory()
        assert coordinator.task_invoke("traj_arm", "execute", {"trajectory": traj}) is True
        # The trajectory object reaches the task unchanged (kwargs go straight
        # to the method; nothing copies or re-serializes it).
        assert task.executed is traj

    def test_typo_kwarg_raises_naming_task_command_and_kwarg(self, coordinator):
        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")

        with pytest.raises(TypeError) as exc:
            coordinator.task_invoke("traj_arm", "execute", {"trajectroy": _trajectory()})

        message = str(exc.value)
        assert "trajectroy" in message  # the offending kwarg is named
        assert "traj_arm" in message and "execute" in message
        assert task.executed is None  # binding failed before dispatch

    def test_missing_required_kwarg_raises(self, coordinator):
        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")

        with pytest.raises(TypeError) as exc:
            coordinator.task_invoke("traj_arm", "execute", {})

        assert "execute" in str(exc.value)
        assert task.executed is None

    def test_defaulted_param_can_be_omitted(self, coordinator):
        task = CommandRecordingTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")

        # arm(ramp_seconds=None) — the defaulted param may be omitted entirely.
        assert coordinator.task_invoke("g1", "arm", {}) is True
        assert task.armed is True
        assert task.arm_calls == [None]

    def test_undeclared_existing_method_warns_but_dispatches(self, coordinator, mocker):
        import dimos.control.coordinator as coord_mod

        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")
        warn = mocker.patch.object(coord_mod.logger, "warning")

        result = coordinator.task_invoke("traj_arm", "record_time", {"t_now": None})

        assert isinstance(result, float)  # legacy dispatch still happened
        assert task.t_now_seen == result
        assert any("undeclared task_invoke" in str(c.args[0]) for c in warn.call_args_list)

    def test_typo_method_raises_and_names_declared_commands(self, coordinator):
        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")

        # "exceute" is a typo of the declared "execute"; no such method exists.
        with pytest.raises(AttributeError, match="exceute") as excinfo:
            coordinator.task_invoke("traj_arm", "exceute", {})
        assert "execute" in str(excinfo.value)  # the error points at the right name

    def test_reset_runtime_state_covers_declaring_tasks_only(self, coordinator):
        declaring = CommandRecordingTask("g1")
        coordinator.add_task(declaring, task_type="g1_groot_wbc")
        # Bare add: has reset_runtime_state() but declares no commands.
        bare = CommandRecordingTask("bare")
        coordinator.add_task(bare)

        assert coordinator.reset_runtime_state(reactivate=False) == {"g1": True}
        assert declaring.reset_calls == [False]
        assert bare.reset_calls == []

    def test_shims_reach_only_declaring_tasks(self, coordinator):
        declaring = CommandRecordingTask("g1")
        coordinator.add_task(declaring, task_type="g1_groot_wbc")
        # Bare add: has arm()/disarm()/set_dry_run() but declares no commands.
        bare = CommandRecordingTask("bare")
        coordinator.add_task(bare)

        coordinator.set_activated(True)
        coordinator.set_dry_run(True)

        assert declaring.armed is True
        assert declaring.arm_calls == [None]  # arm() called with defaulted ramp_seconds
        assert declaring.dry_run is True
        # The bare task is skipped even though it defines the methods.
        assert bare.armed is None
        assert bare.dry_run is None


class TestDescribeTask:
    def test_reports_command_signatures(self, coordinator):
        task = CommandRecordingTask("traj_arm")
        coordinator.add_task(task, task_type="trajectory")

        desc = coordinator.describe_task("traj_arm")

        assert desc["task"] == "traj_arm"
        assert set(desc["commands"]) == {"execute", "cancel", "get_state"}
        execute = desc["commands"]["execute"]
        assert execute["params"] == ["trajectory"]
        assert "trajectory" in execute["signature"]
        assert desc["commands"]["cancel"]["params"] == []
        assert desc["streams"] == []

    def test_reports_stream_routes(self, coordinator):
        # servo declares no commands but consumes joint_command.
        task = CommandRecordingTask("servo1")
        coordinator.add_task(task, task_type="servo")

        desc = coordinator.describe_task("servo1")

        assert desc["commands"] == {}
        assert desc["streams"] == [("joint_command", "claim_overlap")]

    def test_unknown_task_returns_none(self, coordinator):
        assert coordinator.describe_task("nope") is None

    def test_g1_reports_twist_stream_and_reset_command(self, coordinator):
        task = CommandRecordingTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")

        desc = coordinator.describe_task("g1")

        assert ("twist_command", "broadcast") in desc["streams"]
        assert "reset_runtime_state" in desc["commands"]
        assert desc["commands"]["reset_runtime_state"]["params"] == ["reactivate"]
