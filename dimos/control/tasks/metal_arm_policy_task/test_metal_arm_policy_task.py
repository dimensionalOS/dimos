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

"""Metal-specific session behavior on top of the shared policy executor.

The executor contract itself (cadence arithmetic, last-command hold, dry-run
gating, session generation, in-flight cancellation, reset/re-engage overlap) is
owned by ``dimos/control/tasks/test_policy_execution.py``.  This file keeps
only what the Metal backend adds: the checkpoint joint contract, the activation
preconditions (complete/fresh feedback, fresh intent), one causal-goal tick,
the teleop anchor, and how each Metal fault path hooks into the executor.

The runtime is a local fake so every test is fast and deterministic; the real
actor is exercised by ``policy/test_artifact_evidence.py`` and the simulator
closed-loop evidence test.
"""

from __future__ import annotations

from collections.abc import Callable
import threading

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointCommandOutput, JointStateSnapshot
from dimos.control.tasks.metal_arm_policy_task.metal_arm_policy_task import (
    MetalArmPolicyTask,
    MetalArmPolicyTaskConfig,
    PolicyTaskFault,
)
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    HOME,
    JOINT_ORDER,
    SCHEMA,
    CheckpointSpec,
    ModelSpec,
    PolicyContractError,
    PolicyManifest,
)
from dimos.teleop.webxr.controller_types import Buttons

JOINTS = tuple(f"arm/{name}" for name in JOINT_ORDER)
HOME_VEC = np.asarray(HOME, dtype=np.float32)
Q0 = HOME_VEC + np.float32(0.03)
DQ0 = np.full(12, 0.01, dtype=np.float32)


def make_manifest() -> PolicyManifest:
    return PolicyManifest(
        schema=SCHEMA,
        name="test-policy",
        checkpoint=CheckpointSpec(
            sha256="0" * 64, iteration=38000, source_sha256="1" * 64, recipe_variant="test"
        ),
        model=ModelSpec(onnx_sha256="2" * 64, normalizer_sha256="3" * 64),
    )


class FakeRuntime:
    """Deterministic stand-in for the ONNX runtime; records every call."""

    def __init__(self) -> None:
        self.manifest = make_manifest()
        self.frames: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self.act_calls = 0
        self.total_act_calls = 0
        self.reset_poses: list[np.ndarray | None] = []
        self.fail_act = False
        self.on_act: Callable[[], None] | None = None
        self.on_reset: Callable[[], None] | None = None
        self._issued = np.zeros(12, dtype=np.float32)

    def reset(self, q_measured=None) -> None:
        if self.on_reset is not None:
            self.on_reset()
        self.frames.clear()
        self.act_calls = 0
        self.reset_poses.append(None if q_measured is None else np.asarray(q_measured).copy())
        if q_measured is not None:
            self._issued = np.asarray(q_measured, dtype=np.float32).copy()

    def observe(self, q, dq, goal_pose24) -> np.ndarray:
        frame = np.asarray(q, dtype=np.float32).copy()
        self.frames.append(
            (
                frame,
                np.asarray(dq, dtype=np.float32).copy(),
                np.asarray(goal_pose24, dtype=np.float64).copy(),
            )
        )
        return frame

    def act(self) -> np.ndarray:
        self.act_calls += 1
        self.total_act_calls += 1
        if self.on_act is not None:
            self.on_act()
        if self.fail_act:
            raise PolicyContractError("actor failure")
        self._issued = self._issued + np.float32(0.01)
        return self._issued.copy()

    @property
    def issued(self) -> np.ndarray:
        return self._issued.copy()


class FakeKinematics:
    def pose24(self, q) -> np.ndarray:
        values = np.asarray(q, dtype=np.float64).reshape(-1)
        out = np.zeros(24, dtype=np.float64)
        out[:6] = values[:6]
        out[6:15] = np.eye(3).reshape(-1)
        out[15:24] = np.eye(3).reshape(-1)
        return out


class FakeClock:
    """Deterministic monotonic receipt clock; tests advance it explicitly."""

    def __init__(self, now: float = 1.0) -> None:
        self.now = float(now)

    def __call__(self) -> float:
        return self.now

    def set(self, now: float) -> None:
        self.now = float(now)


def make_task(
    *,
    teleop: bool = False,
    freshness=None,
    clock: FakeClock | None = None,
    **overrides,
) -> MetalArmPolicyTask:
    config = MetalArmPolicyTaskConfig(
        joint_names=JOINTS,
        model_dir="/unused",
        manifest_path="/unused",
        teleop=teleop,
        **overrides,
    )
    return MetalArmPolicyTask(
        "metal_arm_policy",
        config,
        runtime=FakeRuntime(),
        kinematics=FakeKinematics(),
        feedback_fresh=freshness,
        clock=clock or FakeClock(),
    )


def make_state(
    t_now: float,
    q: np.ndarray | None = None,
    dq: np.ndarray | None = None,
    missing: set[str] | None = None,
) -> CoordinatorState:
    q = Q0 if q is None else np.asarray(q, dtype=np.float32)
    dq = DQ0 if dq is None else np.asarray(dq, dtype=np.float32)
    positions = {name: float(q[i]) for i, name in enumerate(JOINTS)}
    velocities = {name: float(dq[i]) for i, name in enumerate(JOINTS)}
    for name in missing or set():
        positions.pop(name)
        velocities.pop(name)
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=positions, joint_velocities=velocities, timestamp=0.0
        ),
        t_now=t_now,
        dt=0.01,
    )


def goal24(offset: float = 0.0) -> list[float]:
    out = np.zeros(24, dtype=np.float64)
    out[0] = 0.4 + offset
    out[3] = 0.0
    out[6:15] = np.eye(3).reshape(-1)
    out[15:24] = np.eye(3).reshape(-1)
    return out.tolist()


def _session(task: MetalArmPolicyTask) -> str:
    return str(task.state_snapshot()["session"])


def activate(task: MetalArmPolicyTask, t_now: float = 1.0) -> None:
    assert task.set_goal_pose24(goal24(), t_now=t_now - 0.05)
    assert task.engage()
    output = task.compute(make_state(t_now))
    assert output is not None


# --------------------------------------------------------------------- config


def test_claim_and_config_are_the_checkpoint_contract():
    claim = make_task().claim()
    assert claim.joints == frozenset(JOINTS) and len(claim.joints) == 12
    assert claim.mode is ControlMode.SERVO_POSITION

    with pytest.raises(ValueError, match="12 joint names"):
        MetalArmPolicyTaskConfig(joint_names=JOINTS[:11], model_dir=".", manifest_path=".")
    reordered = (JOINTS[1], JOINTS[0], *JOINTS[2:])
    with pytest.raises(ValueError, match="checkpoint joint order"):
        MetalArmPolicyTaskConfig(joint_names=reordered, model_dir=".", manifest_path=".")
    with pytest.raises(ValueError, match="one robot"):
        MetalArmPolicyTaskConfig(
            joint_names=tuple(
                name.replace("arm/", f"arm{i % 2}/") for i, name in enumerate(JOINTS)
            ),
            model_dir=".",
            manifest_path=".",
        )


# ------------------------------------------------- activation preconditions


def test_inactive_task_and_ungoaled_engage_emit_nothing():
    task = make_task()
    assert task.compute(make_state(1.0)) is None
    assert _session(task) == "disengaged"
    assert not task.engage()
    assert task.compute(make_state(1.0)) is None


def test_activation_needs_complete_and_fresh_feedback():
    task = make_task()
    assert task.set_goal_pose24(goal24(), t_now=0.95)
    assert task.engage()
    assert task.compute(make_state(1.0, missing={"arm/left_joint3"})) is None
    assert _session(task) == "disengaged"
    assert task.compute(make_state(1.01)) is not None
    assert _session(task) == "active"

    fresh = {"ok": False}
    stale = make_task(freshness=lambda: fresh["ok"])
    assert stale.set_goal_pose24(goal24(), t_now=0.95)
    assert stale.engage()
    assert stale.compute(make_state(1.0)) is None
    assert _session(stale) == "disengaged"
    fresh["ok"] = True
    assert stale.compute(make_state(1.01)) is not None
    assert _session(stale) == "active"


def test_activation_seeds_from_measured_q_and_emits_12_joint_targets():
    task = make_task()
    activate(task, 1.0)
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    # Engaged at an arbitrary pose: the runtime is seeded from q, never home.
    np.testing.assert_allclose(runtime.reset_poses[0], Q0, atol=1e-6)
    np.testing.assert_allclose(runtime.issued, Q0 + 0.01, atol=1e-6)
    assert len(runtime.frames) == 1 and runtime.act_calls == 1

    output = task.compute(make_state(1.01))
    assert output is not None
    assert output.joint_names == list(JOINTS) and len(output.positions or []) == 12
    assert output.mode is ControlMode.SERVO_POSITION
    assert task.state_snapshot()["fault"] == "none"


def test_policy_cadence_comes_from_the_manifest():
    """The manifest's trained 20 Hz is what the executor schedules."""
    task = make_task()
    activate(task, 1.0)
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    assert task.compute(make_state(1.02)) is not None  # held, no new inference
    assert runtime.act_calls == 1
    assert task.compute(make_state(1.05)) is not None
    assert runtime.act_calls == 2


def test_observation_uses_the_current_causal_goal_only():
    task = make_task()
    activate(task, 1.0)
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    np.testing.assert_allclose(runtime.frames[0][2], np.asarray(goal24()), atol=1e-9)
    # A new absolute goal is honoured on the next policy tick; nothing in the
    # task needs a future goal or a recorded trace.
    assert task.set_goal_pose24(goal24(0.1), t_now=1.04)
    assert task.compute(make_state(1.05)) is not None
    assert len(runtime.frames) == 2
    np.testing.assert_allclose(np.asarray(goal24(0.1)), runtime.frames[1][2], atol=1e-9)


# --------------------------------------------------------------- fault paths


def _fault_release(task: MetalArmPolicyTask) -> None:
    assert task.release()


def _fault_estop(task: MetalArmPolicyTask) -> None:
    task.set_estop(True)


def _fault_preempt(task: MetalArmPolicyTask) -> None:
    task.on_preempted("safety", frozenset({"arm/left_joint1"}))


def _fault_feedback(task: MetalArmPolicyTask) -> None:
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    calls_before = runtime.act_calls
    # Between 20 Hz deadlines: freshness still invalidates and runs no inference.
    assert task.compute(make_state(1.02, missing={"arm/right_joint6"})) is None
    assert runtime.act_calls == calls_before


def _fault_inference(task: MetalArmPolicyTask) -> None:
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    runtime.fail_act = True
    assert task.compute(make_state(1.05)) is None
    runtime.fail_act = False


def _fault_non_finite(task: MetalArmPolicyTask) -> None:
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    runtime._issued[:] = np.nan
    assert task.compute(make_state(1.05)) is None


def _fault_stop(task: MetalArmPolicyTask) -> None:
    task.stop()


#: Every Metal cancel/fault hook, its recorded reason, and how recovery works.
FAULTS = [
    ("release", _fault_release, PolicyTaskFault.RELEASED, "engage"),
    ("estop", _fault_estop, PolicyTaskFault.ESTOP, "clear-estop"),
    ("preempt", _fault_preempt, PolicyTaskFault.PREEMPTED, "engage"),
    ("feedback", _fault_feedback, PolicyTaskFault.FEEDBACK, "engage"),
    ("inference", _fault_inference, PolicyTaskFault.INFERENCE, "engage"),
    ("non-finite", _fault_non_finite, PolicyTaskFault.INFERENCE, "engage"),
    ("stop", _fault_stop, PolicyTaskFault.STOPPED, None),
]


@pytest.mark.parametrize(
    ("apply_fault", "fault", "recovery"),
    [case[1:] for case in FAULTS],
    ids=[case[0] for case in FAULTS],
)
def test_every_fault_closes_the_session_and_never_replays(
    apply_fault, fault: PolicyTaskFault, recovery: str | None
):
    task = make_task()
    activate(task, 1.0)
    apply_fault(task)
    assert _session(task) == "disengaged"
    assert task.state_snapshot()["fault"] == fault.value
    assert task.compute(make_state(1.06)) is None  # nothing is re-emitted
    if recovery is None:
        assert not task.engage()
        return
    if recovery == "clear-estop":
        assert not task.set_goal_pose24(goal24(), t_now=1.06)  # e-stop latches intent
        task.set_estop(False)
    # A fresh same-value goal plus an explicit engage is the only way back.
    assert not task.engage()  # the goal went with the session
    assert task.set_goal_pose24(goal24(), t_now=1.06)  # same value, fresh receipt
    assert task.compute(make_state(1.07)) is None
    assert task.engage()
    assert task.compute(make_state(1.08)) is not None


def test_preemption_of_other_joints_keeps_the_session():
    task = make_task()
    activate(task, 1.0)
    task.on_preempted("gripper", frozenset({"arm/left_joint7"}))
    assert _session(task) == "active"


def test_accepted_but_unemitted_command_is_dropped_on_stale_feedback():
    fresh = {"ok": True}
    task = make_task(freshness=lambda: fresh["ok"])
    activate(task, 1.0)
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    runtime.on_act = lambda: fresh.__setitem__("ok", False)
    assert task.compute(make_state(1.05)) is None
    assert runtime.act_calls == 2  # the runtime accepted the action ...
    assert _session(task) == "disengaged"  # ... but the task refused to emit it
    assert task.state_snapshot()["fault"] == PolicyTaskFault.FEEDBACK.value


# --------------------------------------------------- intent and receipt time


def test_intent_age_refuses_or_closes_a_session():
    # A goal stamped long before the first tick cannot open a session.
    stale = make_task(intent_timeout_s=0.5)
    assert stale.set_goal_pose24(goal24(), t_now=0.0)
    assert stale.engage()
    assert stale.compute(make_state(1.0)) is None
    assert _session(stale) == "disengaged"
    assert stale.state_snapshot()["fault"] == PolicyTaskFault.INTENT.value

    # The engage request itself expires.
    delayed = make_task()
    assert delayed.set_goal_pose24(goal24(), t_now=1.0)
    assert delayed.engage(t_now=1.0)
    assert delayed.compute(make_state(10.0)) is None
    assert delayed.state_snapshot()["fault"] == PolicyTaskFault.INTENT.value

    # Once open, a streamed goal is bounded by goal_timeout_s.
    streamed = make_task(goal_timeout_s=0.2)
    activate(streamed, 1.0)
    assert streamed.compute(make_state(1.15)) is not None
    assert streamed.compute(make_state(1.30)) is None
    assert _session(streamed) == "disengaged"
    assert streamed.state_snapshot()["fault"] == PolicyTaskFault.INTENT.value


@pytest.mark.parametrize(
    ("tick_at", "session"),
    [(100.0, "disengaged"), (1.01, "active")],
    ids=["stale-at-first-tick", "prompt-first-tick"],
)
def test_untimestamped_goal_is_stamped_at_receipt(tick_at: float, session: str):
    clock = FakeClock(1.0)
    task = make_task(clock=clock)
    assert task.set_goal_pose24(goal24())  # no timestamp: stamp receipt now
    if session == "disengaged":
        clock.set(2.0)  # it sat for a second before anyone engaged
    assert task.engage()
    output = task.compute(make_state(tick_at))
    assert (output is not None) == (session == "active")
    assert _session(task) == session
    if session == "disengaged":
        assert task.state_snapshot()["fault"] == PolicyTaskFault.INTENT.value


def test_goal_size_is_validated():
    task = make_task()
    with pytest.raises(ValueError):
        task.set_goal_pose24([0.0] * 23, t_now=0.0)
    with pytest.raises(ValueError):
        task.set_goal([0.0, 0.0], [0, 0, 0, 1], [0.0, 0.0, 0.0], [0, 0, 0, 1], t_now=0.0)


# ---------------------------------------------------------------------- reset


def test_reset_is_not_an_engage_and_seeds_from_measured_pose():
    task = make_task()
    activate(task, 1.0)
    assert task.reset_runtime_state(reactivate=False)
    assert _session(task) == "disengaged"
    assert task.compute(make_state(1.05)) is None
    # A re-engage at a different measured pose seeds from that pose, not home.
    moved = Q0 + 0.1
    assert task.set_goal_pose24(goal24(), t_now=1.05)
    assert task.engage()
    assert task.compute(make_state(1.06, q=moved)) is not None
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    np.testing.assert_allclose(runtime.reset_poses[-1], moved, atol=1e-6)
    np.testing.assert_allclose(runtime.issued, moved + 0.01, atol=1e-6)

    # reactivate=True keeps the intent but still waits for complete feedback.
    reactive = make_task()
    activate(reactive, 1.0)
    assert reactive.reset_runtime_state(reactivate=True)
    assert _session(reactive) == "disengaged"
    assert reactive.compute(make_state(1.02, missing={"arm/right_joint4"})) is None
    assert reactive.compute(make_state(1.03)) is not None
    assert _session(reactive) == "active"


# ---------------------------------------------------------------------- teleop


def make_pose(x: float, y: float, z: float) -> Pose:
    return Pose(position=Vector3(x, y, z), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))


def grips(left: bool, right: bool) -> Buttons:
    buttons = Buttons()
    buttons.left_grip = left
    buttons.right_grip = right
    return buttons


def _teleop_at(task: MetalArmPolicyTask, left, right, buttons, t_now: float) -> None:
    assert task.on_left_cartesian_command(make_pose(*left), t_now)
    assert task.on_right_cartesian_command(make_pose(*right), t_now)
    assert task.on_teleop_buttons(grips(*buttons), t_now)


def _active_teleop(goal_timeout_s: float = 0.25) -> MetalArmPolicyTask:
    task = make_task(teleop=True, goal_timeout_s=goal_timeout_s)
    _teleop_at(task, (0.5, 0.1, 0.2), (0.5, -0.1, 0.2), (True, True), 0.9)
    assert task.on_teleop_buttons(grips(True, True), 0.95)
    assert task.compute(make_state(1.0)) is not None
    assert _session(task) == "active"
    return task


def test_teleop_deadman_waits_for_both_poses_then_maps_motion():
    task = make_task(teleop=True)
    assert task.on_teleop_buttons(grips(True, True), 0.9)
    assert task.compute(make_state(1.0)) is None  # no controller poses yet
    assert _session(task) == "disengaged"
    assert task.on_left_cartesian_command(make_pose(0.5, 0.1, 0.2), 0.9)
    assert task.compute(make_state(1.0)) is None  # one side is still missing
    assert task.on_right_cartesian_command(make_pose(0.5, -0.1, 0.2), 0.99)
    assert task.compute(make_state(1.0)) is not None
    assert _session(task) == "active"
    assert task.state_snapshot()["mapper_engaged"] is True

    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    anchor_goal = runtime.frames[0][2].copy()
    # Moving the left controller +0.1 m in x moves the left TCP goal by 0.1 m.
    assert task.on_left_cartesian_command(make_pose(0.6, 0.1, 0.2), 1.01)
    assert task.compute(make_state(1.05)) is not None
    moved_goal = runtime.frames[1][2]
    assert moved_goal[0] == pytest.approx(anchor_goal[0] + 0.1, abs=1e-6)
    assert moved_goal[3] == pytest.approx(anchor_goal[3], abs=1e-9)

    # Releasing either grip is a release, not a staleness fault.
    assert task.on_teleop_buttons(grips(True, False), 1.07)
    assert _session(task) == "disengaged"
    assert task.compute(make_state(1.08)) is None
    assert task.state_snapshot()["fault"] == PolicyTaskFault.RELEASED.value


@pytest.mark.parametrize("source", ["left", "right", "buttons"])
def test_one_stale_teleop_source_invalidates_even_with_live_others(source: str):
    task = _active_teleop()
    if source != "left":
        assert task.on_left_cartesian_command(make_pose(0.6, 0.1, 0.2), 10.0)
    if source != "right":
        assert task.on_right_cartesian_command(make_pose(0.6, -0.1, 0.2), 10.0)
    assert task.compute(make_state(10.0)) is None
    assert _session(task) == "disengaged"
    assert task.state_snapshot()["fault"] == PolicyTaskFault.INTENT.value


def test_stale_inputs_invalidate_between_ticks_and_recovery_needs_both_sides():
    task = _active_teleop()
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    assert runtime.act_calls == 1
    # Still inside the freshness window: holds without a new inference.
    assert task.compute(make_state(1.02)) is not None
    assert runtime.act_calls == 1
    # No source has reported since activation; the tick itself invalidates.
    assert task.compute(make_state(1.3)) is None
    assert runtime.act_calls == 1
    assert _session(task) == "disengaged"
    assert task.state_snapshot()["fault"] == PolicyTaskFault.INTENT.value

    # One live side plus a fresh deadman is not enough: the other side was
    # cleared with the session and may never be replayed.
    assert task.on_left_cartesian_command(make_pose(0.6, 0.1, 0.2), 10.1)
    assert task.on_teleop_buttons(grips(True, True), 10.1)
    assert task.compute(make_state(10.2)) is None
    assert _session(task) == "disengaged"
    assert task.on_right_cartesian_command(make_pose(0.6, -0.1, 0.2), 10.2)
    assert task.compute(make_state(10.3)) is not None
    assert _session(task) == "active"


# ------------------------------------------- responsive cancellation


def _blocked(task: MetalArmPolicyTask) -> tuple[threading.Event, threading.Event]:
    """Make the next runtime call block so a concurrent cancel is observable."""
    entered, finish = threading.Event(), threading.Event()
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)

    def block() -> None:
        entered.set()
        finish.wait(5)

    runtime.on_act = block
    return entered, finish


def _thread(run: Callable[[], None]) -> threading.Thread:
    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    return worker


def test_cancellation_is_responsive_and_reengage_seeds_from_measured_q():
    """A release during act() must not wait, and its result must be dropped."""
    task = make_task()
    assert task.set_goal_pose24(goal24(), t_now=0.95)
    assert task.engage(t_now=0.95)
    entered, finish = _blocked(task)
    outputs: list[JointCommandOutput | None] = []
    worker = _thread(lambda: outputs.append(task.compute(make_state(1.0))))
    assert entered.wait(2)
    cancelled = threading.Event()

    def run_cancel() -> None:
        task.release()
        cancelled.set()

    canceller = _thread(run_cancel)
    assert cancelled.wait(2), "cancellation blocked on the in-flight inference"
    finish.set()
    worker.join(2)
    canceller.join(2)
    assert outputs == [None]
    assert _session(task) == "disengaged"
    assert task.state_snapshot()["fault"] == PolicyTaskFault.RELEASED.value

    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    assert runtime.total_act_calls == 1
    assert task.set_goal_pose24(goal24(), t_now=1.01)
    assert task.engage(t_now=1.01)
    output = task.compute(make_state(1.02))
    assert output is not None
    # The reset re-seeds from the measured pose, discarding the dropped step.
    assert runtime.total_act_calls == 2
    assert len(runtime.reset_poses) == 2
    np.testing.assert_allclose(runtime.reset_poses[-1], Q0, atol=1e-6)
    np.testing.assert_allclose(
        np.asarray(output.positions, dtype=np.float32), Q0 + np.float32(0.01), atol=1e-6
    )


@pytest.mark.parametrize(
    "clear_while_blocked",
    [True, False],
    ids=["clear-before-completion", "completed-before-clear"],
)
def test_estop_clear_ordering_around_an_inflight_compute(clear_while_blocked: bool) -> None:
    """Clearing e-stop before or after the blocked result returns never replays it."""
    task = make_task()
    assert task.set_goal_pose24(goal24(), t_now=0.95)
    assert task.engage(t_now=0.95)
    entered, finish = _blocked(task)
    outputs: list[JointCommandOutput | None] = []
    worker = _thread(lambda: outputs.append(task.compute(make_state(1.0))))
    assert entered.wait(2)

    task.set_estop(True)  # completes while act() is still blocked
    assert not task.session_active
    assert task.state_snapshot()["fault"] == PolicyTaskFault.ESTOP.value
    runtime = task._runtime
    assert isinstance(runtime, FakeRuntime)
    calls = runtime.total_act_calls
    epoch = task.session_epoch

    if clear_while_blocked:
        # The clear itself must not wait for the in-flight inference.
        task.set_estop(False)
        assert worker.is_alive()
    finish.set()
    worker.join(2)
    assert outputs == [None]
    assert runtime.total_act_calls == calls
    if not clear_while_blocked:
        task.set_estop(False)

    # Clearing neither advances the generation nor replays the dropped command.
    assert task.session_epoch == epoch
    assert task.compute(make_state(1.03)) is None
    assert runtime.total_act_calls == calls
    assert not task.engage()  # the goal went with the session
    assert task.set_goal_pose24(goal24(), t_now=1.03)  # same value, fresh receipt
    assert task.engage()
    output = task.compute(make_state(1.04))
    assert output is not None and _session(task) == "active"
    np.testing.assert_allclose(runtime.reset_poses[-1], Q0, atol=1e-6)
    assert runtime.total_act_calls == calls + 1
