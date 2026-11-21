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

"""Calibrates one arm of a Damiao robot, in the same ``dimos run`` as its connection.

``run`` starts the steps (``steps``) on one thread and returns. A motion streams one target per
``joint_state`` sample on ``position_command`` and the thread waits for it to end; while it asks
the operator or computes, nothing is sent, so the connection's deadman holds the arm and undoes
live changes. The operator, in ``dimos shell`` or a program, reads ``status`` and replies with
``answer``; unanswered, the thread waits and the arm stays held. ``abort`` stops streaming at once
and offers to go home.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import itertools
import math
from pathlib import Path
import threading
from typing import Any, Literal, Protocol

import numpy as np
from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.control.calibration.gravity_fit import Measurement
from dimos.control.calibration.gravity_poses import ArmModel, Pose
from dimos.control.calibration.motion import HoldMode, Motion, Sample, plan_path
from dimos.control.calibration.schema import (
    ARMS_STATE_DIR,
    AppliedFile,
    ArmCalibrationError,
    bake_into_urdf,
    calibration_path,
    load_for_spec,
)
from dimos.control.calibration.spec import (
    ArmCalibrationSpec,
    arm_name,
    motor_torque_limit_nm,
    specs_from_damiao_robot,
)
from dimos.control.calibration.step_analysis import RestNoise, StepMetrics
from dimos.control.calibration.steps import (
    STEPS,
    TITLES,
    AbortedError,
    SkippedError,
    StalledError,
    plan,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

LOG_SIZE = 50
NOT_HOME = "The arm is not home: support it before stopping the run, which turns its motors off."
RunState = Literal["idle", "running", "waiting", "done", "failed", "aborted"]
StepState = Literal["pending", "running", "done", "skipped", "failed", "aborted"]


class CalibrationModuleConfig(ModuleConfig):
    """Which arm to calibrate, and how. These are all of the calibration's settings.

    Attributes:
        arm: The arm ("left"), its bus ("left_arm") or its id; needed when the robot has several.
        floor_z_m: The table's height from the arm's base, m. Required, with ``min_x_m``.
        min_x_m: The fingertip stays in front of this x from the base, m.
        poses: How many gravity poses to visit.
        speed_deg_s: Peak joint speed of every move; at most 60 and the URDF velocity limits.
        max_hold_s: A hold that waits to settle and has not after this long ends not converging.
        rest_window_s: How long the arm sits still to measure its noise; a hold is settled
            once one window of samples stays inside that noise. Long enough to catch the
            noise's peaks: the buzz check compares a step against them.
        dither_deg: The smallest dither of a measured joint either way at a gravity pose.
        max_dither_deg: The largest dither, also the farthest a pose's dithers turn a joint.
        dither_torque_fraction: Each joint dithers by this share of its torque cap over its
            kp, so its spring beats its friction, within ``dither_deg`` and ``max_dither_deg``.
        dither_hold_s: How long each dither holds; its last rest window is the joint's rest.
        stuck_fraction: A dither pair the joint followed less than this share of the way is
            stuck; it is redone once at twice the dither (at most ``max_dither_deg``).
        dither_span_nm: A joint the spec does not measure is dithered too when its gravity
            torque over the poses, or that of one column scale of correction, spans more.
        kp_scales: Multiples of each joint's kp the gain sweep tries, each with every kd scale;
            must include 1, the baseline.
        kd_scales: Multiples of each joint's kd it tries with each kp; must include 1.
        kd_per_kp: A candidate's kd is at most this times its kp, and the driver's limit.
        step_deltas_deg: The gain sweep's steps, applied in turn; both signs needed.
        max_overshoot_pct: A sweep step that overshoots this share of itself fails.
        rest_limit_deg: The smallest rest limit of a sweep step; ``friction_rest_factor``
            times the joint's friction over kp widens it.
        friction_rest_factor: How many times its friction over kp a joint may rest off target.
        settle_s: A candidate scores its worst rest error over its limit plus its worst settle
            over this; the lowest wins.
        hold_s: How long a joint rests at full gains at its float pose before it floats.
        float_window_s: How long each joint floats.
        float_drift_limit_mm: A float passes when the fingertip ends within this of where it was
            at the first limp sample.
        float_trip_deg: A floating joint that turns this far is held again, and fails.
        confirm_poses: How many gravity poses confirm_tracking revisits.
        tracking_rest_ratio: The proposed gains are saved only if every joint rests within this
            times its first visit, or within its rest limit at its new kp.
        grid_points: About how many candidate poses the pose search considers.
        steps: Which steps to run after check (``steps.STEPS``).
        confirm_each: Ask before every move, not only at the start of each step.
    """

    arm: str | None = None
    floor_z_m: float | None = None
    min_x_m: float | None = None
    poses: int = Field(default=12, ge=3)
    speed_deg_s: float = Field(default=10.0, gt=0.0, le=60.0)
    rest_window_s: float = Field(default=0.5, gt=0.0)
    max_hold_s: float = Field(default=5.0, gt=0.0)
    dither_deg: float = Field(default=1.0, gt=0.0)
    max_dither_deg: float = Field(default=5.0, gt=0.0)
    dither_torque_fraction: float = Field(default=0.3, gt=0.0)
    dither_hold_s: float = Field(default=1.0, gt=0.0)
    stuck_fraction: float = Field(default=0.5, gt=0.0, lt=1.0)
    dither_span_nm: float = Field(default=0.1, gt=0.0)
    kp_scales: list[float] = Field(default_factory=lambda: [0.6, 1.0, 1.5])
    kd_scales: list[float] = Field(default_factory=lambda: [1.0, 2.5, 5.0])
    kd_per_kp: float = Field(default=0.25, gt=0.0)
    step_deltas_deg: list[float] = Field(default_factory=lambda: [1.0, -1.0, 3.0, -3.0])
    max_overshoot_pct: float = Field(default=20.0, gt=0.0)
    rest_limit_deg: float = Field(default=0.1, gt=0.0)
    friction_rest_factor: float = Field(default=1.5, gt=0.0)
    settle_s: float = Field(default=0.5, gt=0.0)
    hold_s: float = Field(default=2.0, gt=0.0)
    float_window_s: float = Field(default=10.0, gt=0.0)
    float_drift_limit_mm: float = Field(default=2.0, gt=0.0)
    float_trip_deg: float = Field(default=5.0, gt=0.0)
    confirm_poses: int = Field(default=3, ge=1)
    tracking_rest_ratio: float = Field(default=1.5, gt=0.0)
    grid_points: int = Field(default=3000, ge=1)
    steps: list[str] = Field(default_factory=lambda: list(STEPS))
    confirm_each: bool = True

    @model_validator(mode="after")
    def _check(self) -> Self:
        unknown = set(self.steps) - set(STEPS)
        if unknown:
            raise ValueError(f"unknown steps {sorted(unknown)}; the steps are {list(STEPS)}")
        if self.dither_deg > self.max_dither_deg:
            raise ValueError("dither_deg must not exceed max_dither_deg")
        if self.max_hold_s <= 2.0 * self.rest_window_s:
            raise ValueError("max_hold_s must exceed two rest windows, the least a hold judges")
        if self.dither_hold_s <= self.rest_window_s:
            raise ValueError("dither_hold_s must outlast rest_window_s, the rest it scores")
        for name, scales in (("kp_scales", self.kp_scales), ("kd_scales", self.kd_scales)):
            if 1.0 not in scales or min(scales) <= 0.0:
                raise ValueError(f"{name} must be positive and include 1, the baseline")
        if not (min(self.step_deltas_deg) < 0.0 < max(self.step_deltas_deg)):
            raise ValueError("step_deltas_deg needs a positive and a negative step")
        return self

    def arm_spec(self, robot: DamiaoRobot) -> ArmCalibrationSpec:
        """The arm's spec on the bench guard; refused without one or above its velocity limits."""
        if self.floor_z_m is None or self.min_x_m is None:
            raise ValueError("give the table height and the front line: floor_z_m and min_x_m")
        spec = _choose_arm(robot, self.arm).on_bench(self.floor_z_m, self.min_x_m)
        limits = {joint.name: joint.velocity for joint in spec.model.load().joints}
        slow = {
            name: f"{math.degrees(limit):.0f}"
            for name in spec.joint_names
            if (limit := limits[name]) is not None and math.radians(self.speed_deg_s) > limit
        }
        if slow:
            raise ValueError(f"speed_deg_s {self.speed_deg_s:g} exceeds URDF limits (deg/s) {slow}")
        return spec


def _choose_arm(robot: DamiaoRobot, arm: str | None) -> ArmCalibrationSpec:
    """One arm of ``robot``, by arm ("left"), bus ("left_arm") or arm id; optional for one arm."""
    specs = specs_from_damiao_robot(robot)
    names = [spec.arm_id.rsplit("/", 1)[-1] for spec in specs]
    if arm is None:
        if len(specs) != 1:
            raise ValueError(f"{robot.name} has several arms; choose one of {names}")
        return specs[0]
    for spec, bus in zip(specs, robot.buses, strict=True):
        if arm in (spec.arm_id, spec.arm_id.rsplit("/", 1)[-1], bus.name, arm_name(bus.name)):
            return spec
    raise ValueError(f"{robot.name} has no arm {arm!r}; choose one of {names}")


class ArmConfigSpec(Spec, Protocol):
    """A Damiao connection deployed with ``config_writes``."""

    def get_config(self) -> dict[str, Any]: ...
    def set_config(self, patch: dict[str, Any], save: bool = False) -> dict[str, Any]: ...


class StepRow(BaseConfig):
    """One step of the plan: what it does until it ends, then its one-line result."""

    name: str
    state: StepState
    result: str


class CalibrationStatus(BaseConfig):
    """What the run does and asks, all an operator needs to answer it; ``print`` it.

    Attributes:
        state: "idle" before the first run, "running", "waiting" on ``question``, then how the
            run ended: "done", "failed" or "aborted".
        arm: The arm's id.
        step: The step running, or the last one that ran.
        steps: Every step of the plan in order, with its state and result.
        message: The latest progress the run showed.
        question: What the run waits on, or None.
        answers: The words ``answer`` takes for ``question``.
        file: The calibration file the arm runs, as the run last read it.
        summary: Each step's result, or why the run ended, once it has.
        log: The last messages, questions (``?``) and answers (``>``), oldest first.
    """

    state: RunState
    arm: str
    step: str | None = None
    steps: list[StepRow]
    message: str = ""
    question: str | None = None
    answers: list[str] = Field(default_factory=list)
    file: AppliedFile | None = None
    summary: str = ""
    log: list[str] = Field(default_factory=list)

    def __str__(self) -> str:
        lines = [f"{self.arm}: {self.state}" + (f" at {self.step}" if self.step else "")]
        lines += [f"  {row.name:<17} {row.state:<8} {row.result}" for row in self.steps]
        if self.file is not None:
            lines.append(f"runs {self.file.path}")
        if self.message:
            lines.append(self.message)
        if self.question is not None:
            lines += [f"? {self.question}", "  answer: " + " | ".join(self.answers)]
        return "\n".join(lines)

    def _repr_pretty_(self, printer: Any, cycle: bool) -> None:
        printer.text(str(self))


def _rows(steps: list[tuple[str, str | None]]) -> list[StepRow]:
    return [
        StepRow(name=step, state="pending", result=TITLES[step])
        if reason is None
        else StepRow(name=step, state="skipped", result=reason)
        for step, reason in steps
    ]


class Operator:
    """The run's operator: the status ``show`` and ``ask`` keep, and the answer the run waits on.

    ``ask`` poses a question and returns the event ``answer`` sets; the run waits on it with
    ``Calibration.wait``, so ``abort`` and ``stop`` wake it too.

    Args:
        arm: The arm's id.
        steps: The plan, as ``steps.plan`` gives it.
    """

    def __init__(self, arm: str, steps: list[tuple[str, str | None]]) -> None:
        self._lock = threading.Lock()
        self._log: deque[str] = deque(maxlen=LOG_SIZE)
        self._status = CalibrationStatus(state="idle", arm=arm, steps=_rows(steps))
        self._answered = threading.Event()
        self.reply = ""

    def status(self) -> CalibrationStatus:
        with self._lock:
            return self._status.model_copy(update={"log": list(self._log)}, deep=True)

    def begin(self, steps: list[tuple[str, str | None]]) -> None:
        """A new run of the plan ``steps``; the log goes on."""
        with self._lock:
            self._status = CalibrationStatus(
                state="running", arm=self._status.arm, steps=_rows(steps)
            )

    def show(self, text: str) -> None:
        with self._lock:
            self._status.message = text
            self._log.append(text)

    def ask(self, prompt: str, answers: list[str]) -> threading.Event:
        with self._lock:
            self._answered = threading.Event()
            self._status.state, self._status.question = "waiting", prompt
            self._status.answers = list(answers)
            self._log.append(f"? {prompt}")
            return self._answered

    def answer(self, text: str) -> str:
        """Answer the pending question with one of its answers: "ok", or why not."""
        word = text.strip().lower()
        with self._lock:
            if self._status.question is None:
                return "no question pending"
            if word not in self._status.answers:
                return f"unknown answer {text!r}; answer {' or '.join(self._status.answers)}"
            self.reply = word
            self._status.state, self._status.question, self._status.answers = "running", None, []
            self._log.append(f"> {word}")
            self._answered.set()
        return "ok"

    def withdraw(self) -> None:
        """The question was answered, or the run interrupted: it runs on."""
        with self._lock:
            self._status.state, self._status.question, self._status.answers = "running", None, []

    def mark(self, step: str, state: StepState, result: str | None = None) -> None:
        """``step`` is now ``state``, with ``result`` once it has one."""
        with self._lock:
            row = next(row for row in self._status.steps if row.name == step)
            row.state, row.result = state, row.result if result is None else result
            self._status.step = step

    def finish(self, state: RunState, summary: str, home: bool) -> None:
        """The run ended ``state``, as ``summary`` says; ``home``: the arm is home."""
        message = f"Calibration {'done' if state == 'done' else 'not done'}:\n{summary}"
        message += "" if home else f"\n{NOT_HOME}"
        with self._lock:
            self._status.state, self._status.summary, self._status.message = state, summary, message
            self._log.append(message)


class Calibration:
    """One calibration run: ``feed`` it every ``joint_state``, ``run`` it on its own thread.

    Args:
        spec: The arm.
        config: The settings.
        connection: The connection's ``get_config`` and ``set_config``.
        operator: What it shows and asks, the plan of its ``config.steps`` begun.
        publish: Sends one position command.
        torque_limit_nm: Each joint's motor torque limit, joint order: past it over kp off
            target, the motor cannot push through, and a motion stops.
        block: How the run thread waits on the arm (tests step a fake arm instead); it waits
            on the operator in real time.
    """

    def __init__(
        self,
        spec: ArmCalibrationSpec,
        config: CalibrationModuleConfig,
        connection: ArmConfigSpec,
        operator: Operator,
        publish: Callable[[JointState], None],
        torque_limit_nm: np.ndarray,
        block: Callable[[threading.Event], Any] = threading.Event.wait,
    ) -> None:
        self.spec, self.arm, self.config, self.connection = spec, ArmModel(spec), config, connection
        # The farthest a pose's dithers and steps turn a joint; a floating joint may drift on top.
        self.excursion_rad = math.radians(
            max(
                config.max_dither_deg,
                *(abs(x) for x in itertools.accumulate(config.step_deltas_deg)),
            )
        )
        self.arm.clearance_m = self.arm.reach_m * (
            self.excursion_rad + math.radians(config.float_trip_deg)
        )
        self.operator, self._publish, self._block = operator, publish, block
        self._torque_limit = torque_limit_nm
        self._lock = threading.Lock()
        self._motion: Motion | None = None
        self._waiting: threading.Event | None = None
        self._interrupt: Exception | None = None
        self._stopping = False
        self._names: list[str] = []
        self._index: list[int] = []
        self._latest: Sample | None = None
        self.first_sample = threading.Event()
        self.state: dict[str, Any] = {}
        self.noise: dict[str, RestNoise] = {}
        self.poses: list[Pose] | None = None
        self.dithered: list[str] = []
        self.measurements: list[Measurement] = []
        self.baseline: dict[str, np.ndarray] = {}
        self.step_metrics: list[StepMetrics] = []
        self.stopped: dict[str, dict[tuple[float, float], str]] = {}
        self.friction: dict[str, float] = {}
        self.proposed: dict[str, dict[str, float]] = {}
        self.at_home = False

    def feed(self, msg: JointState) -> None:
        """Take one ``joint_state``: keep it, and send the running motion's next target."""
        if not self._names:
            short = [name.rsplit("/", 1)[-1] for name in msg.name]
            missing = [joint for joint in self.spec.joint_names if joint not in short]
            if missing or len(msg.velocity) != len(msg.name):
                self._stop_with(RuntimeError(f"joint_state lacks {missing or 'velocities'}"))
                return
            self._index = [short.index(joint) for joint in self.spec.joint_names]
            self._names = [msg.name[i] for i in self._index]
        sample = Sample(
            msg.ts, np.asarray(msg.position)[self._index], np.asarray(msg.velocity)[self._index]
        )
        with self._lock:
            self._latest = sample
            motion = self._motion
        self.first_sample.set()
        target = None if motion is None else motion.feed(sample)
        with self._lock:
            if target is not None and self._motion is motion:
                self._publish(JointState(ts=sample.t, name=self._names, position=target.tolist()))

    def abort(self) -> None:
        self._stop_with(AbortedError("aborted by the operator"))

    def stop(self) -> None:
        self._stopping = True
        self._stop_with(AbortedError("the calibration module stopped"))

    def _stop_with(self, error: Exception) -> None:
        with self._lock:
            self._interrupt = error
            self._motion = None
            if self._waiting is not None:
                self._waiting.set()

    def wait(
        self, event: threading.Event, block: Callable[[threading.Event], Any] | None = None
    ) -> None:
        """Wait for ``event``, as for the arm unless ``block`` says how; raises what interrupted
        the run (``abort``, ``stop``, a bad feed)."""
        with self._lock:
            if self._interrupt is not None:
                raise self._interrupt
            self._waiting = event
        try:
            (block or self._block)(event)
        finally:
            with self._lock:
                self._waiting = None
        with self._lock:
            if self._interrupt is not None:
                raise self._interrupt

    def show(self, text: str) -> None:
        self.operator.show(text)

    def ask(self, prompt: str, choices: list[str]) -> str:
        """The operator's answer, one of ``choices``; ``abort`` ends the wait."""
        answered = self.operator.ask(prompt, choices)
        try:
            self.wait(answered, threading.Event.wait)
        finally:
            self.operator.withdraw()
        return self.operator.reply

    def status(self) -> CalibrationStatus:
        """The operator's status, with the file the arm runs as the run last read it."""
        status = self.operator.status()
        file = self.state.get("file")
        status.file = None if file is None else AppliedFile.model_validate(file)
        return status

    def confirm(self, prompt: str, *, skippable: bool = True) -> None:
        if self.ask(prompt, ["go", "skip"] if skippable else ["go"]) == "skip":
            raise SkippedError(prompt)

    def configure(self, patch: dict[str, Any], save: bool = False) -> None:
        """``set_config`` for this arm; keeps the state it returns."""
        self.state = self.connection.set_config({self.spec.arm_id: patch}, save)[self.spec.arm_id]

    def gains(self, joint: str) -> tuple[float, float]:
        gains = self.state["gains"][joint]
        return float(gains["kp"]), float(gains["kd"])

    def measured(self) -> np.ndarray:
        with self._lock:
            assert self._latest is not None
            return self._latest.q

    def motion(
        self,
        path: list[np.ndarray],
        mode: HoldMode,
        window_s: float | None = None,
        **hold: Any,
    ) -> Motion:
        """A motion along ``path`` with the current gains, noise and settings (``window_s``:
        a hold's window other than the noise's)."""
        kp = np.asarray([self.gains(joint)[0] for joint in self.spec.joint_names])
        stall = np.divide(self._torque_limit, kp, out=np.full(kp.shape, np.inf), where=kp > 0.0)
        band = [
            self.noise[joint].band_deg if self.noise else 0.0 for joint in self.spec.joint_names
        ]
        # Once measured, the noise's own window: a settled hold then outlasts it.
        window = window_s or max(
            (n.window_s for n in self.noise.values()), default=self.config.rest_window_s
        )
        return Motion(
            self.arm,
            path,
            math.radians(self.config.speed_deg_s),
            mode,
            window,
            2.0 * np.radians(band),
            stall,
            max_hold_s=self.config.max_hold_s,
            **hold,
        )

    def stream(self, motion: Motion, patch: dict[str, Any] | None = None) -> Motion:
        """Stream ``motion`` until it ends, applying the live ``patch`` once it streams."""
        self.at_home = False
        with self._lock:
            if self._interrupt is not None:
                raise self._interrupt
            self._motion = motion
        try:
            if patch is not None:
                self.configure(patch)
            self.wait(motion.done)
        finally:
            with self._lock:
                if self._motion is motion:
                    self._motion = None
        if motion.error:
            raise (StalledError if motion.stalled else RuntimeError)(motion.error)
        return motion

    def move(
        self,
        target: np.ndarray,
        what: str,
        *,
        ask: bool = True,
        patch: dict[str, Any] | None = None,
    ) -> Motion:
        """Go to ``target`` along a clear path and settle there, asking first with ``confirm_each``,
        with the live ``patch`` applied as it streams."""
        if ask and self.config.confirm_each:
            degrees = ", ".join(f"{math.degrees(v):.1f}" for v in target)
            self.confirm(f"Move to {what} [{degrees}] deg?")
        path = plan_path(self.arm, self.measured(), target)
        motion = self.stream(self.motion(path, "converge"), patch)
        if motion.ended != "settled":
            raise RuntimeError(f"the arm is {motion.ended} at {what}")
        self.at_home = what == "home"
        return motion

    def home(
        self, trail: list[np.ndarray] | None = None, patch: dict[str, Any] | None = None
    ) -> None:
        """Go home, back through the poses of ``trail`` until a clear path home opens."""
        for back in reversed(trail or []):
            if self.arm.transition_ok(self.measured(), self.arm.home):
                break
            self.move(back, "the previous pose", ask=False, patch=patch)
        self.move(self.arm.home, "home", patch=patch)

    def _step(self, name: str, reason: str | None) -> str:
        """Step ``name``'s result, run unless ``reason`` says why not; its row follows it."""
        if reason is not None:
            self.operator.mark(name, "skipped", reason)
            return f"skipped: {reason}"
        self.operator.mark(name, "running")
        try:
            result = STEPS[name](self)
        except SkippedError:
            self.operator.mark(name, "skipped", "skipped by the operator")
            return "skipped by the operator"
        self.operator.mark(name, "done", result)
        return result

    def run(self) -> None:
        """Every step in order; on a failure or abort the arm holds and home is offered."""
        ended: RunState = "failed"
        current, results = "check", {}
        try:
            chosen = ["check", *(step for step in self.config.steps if step != "check")]
            self.show(f"Calibrating {self.spec.arm_id}: {', '.join(chosen)}")
            results["check"] = self._step("check", None)
            for current, reason in plan(self.config.steps, self.state["can_float"]):
                if current != "check":
                    results[current] = self._step(current, reason)
                    self.show(f"{current}: {results[current]}")
            ended = "done"
            text = "\n".join(f"  {step}: {result}" for step, result in results.items())
        except AbortedError as error:
            self.operator.mark(current, "aborted", str(error))
            ended, text = "aborted", f"{current}: {error}"
        except Exception as error:
            logger.exception(f"calibration step {current} failed")
            self.operator.mark(current, "failed", f"{type(error).__name__}: {error}")
            text = f"{current} failed: {type(error).__name__}: {error}"
        if self._stopping:
            return
        if ended != "done":
            with self._lock:
                self._interrupt = None
            try:
                if (
                    self.ask(f"{text}\nThe arm holds where it is. Go home?", ["home", "stay"])
                    == "home"
                ):
                    self.home()
            except AbortedError:
                text += "; going home was aborted"
            except Exception as error:
                logger.exception("going home failed")
                text += f"; going home failed: {type(error).__name__}: {error}"
            with self._lock:
                self._interrupt = None
        self.operator.finish(ended, text, self.at_home)


def bake_calibration(
    robot: DamiaoRobot, urdf_path: str, out_path: str | None = None, root: Path = ARMS_STATE_DIR
) -> str:
    """Bake the fitted links of every arm of ``robot`` with a calibration file under ``root`` into
    the URDF at ``urdf_path``, written to ``out_path`` (default: ``calibrated.urdf`` in the
    robot's directory there); says what it wrote and which arms have no file."""
    baked, missing = [], []
    for spec in specs_from_damiao_robot(robot):
        path, arm = calibration_path(spec.arm_id, root), spec.arm_id.rsplit("/", 1)[-1]
        if path.exists():
            baked.append((arm, load_for_spec(spec, path)))
        else:
            missing.append(arm)
    if not baked:
        raise ArmCalibrationError(f"no arm of {robot.name} has a calibration file under {root}")
    out = (
        Path(out_path)
        if out_path
        else calibration_path(robot.name, root).with_name("calibrated.urdf")
    )
    urdf = bake_into_urdf([calibration for _, calibration in baked], Path(urdf_path).read_text())
    out.write_text(urdf)
    skipped = "".join(f"; {arm} has no calibration file" for arm in missing)
    return f"wrote {out}: baked {', '.join(arm for arm, _ in baked)}{skipped}"


class CalibrationModule(Module):
    """Calibrates one arm of ``robot`` (see the module docstring), driven by its RPCs.

    ``run`` starts a calibration, ``status`` says what it does and asks, ``answer`` replies,
    ``abort`` stops it, and ``bake`` writes the arm's fitted links into a URDF.

    Args:
        robot: The robot its connection deploys, from ``calibration_blueprint``; not a setting.
    """

    config: CalibrationModuleConfig
    _connection: ArmConfigSpec
    joint_state: In[JointState]
    position_command: Out[JointState]

    _calibration: Calibration | None = None
    _thread: threading.Thread | None = None

    def __init__(self, robot: DamiaoRobot, **kwargs: Any) -> None:
        self.robot = robot
        self._connected = threading.Event()
        self._run_lock = threading.Lock()
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self._spec = self.config.arm_spec(self.robot)
        motors = {m.name: m for bus in self.robot.buses for m in bus.arm}
        self._torque_limit = np.asarray(
            [motor_torque_limit_nm(motors[name]) for name in self._spec.joint_names]
        )
        self._operator = Operator(self._spec.arm_id, plan(self.config.steps, self.robot.gravity))
        self.register_disposable(Disposable(self.joint_state.subscribe(self._feed)))

    def _feed(self, msg: JointState) -> None:
        self._connected.set()
        calibration = self._calibration
        if calibration is not None:
            calibration.feed(msg)

    @rpc
    def stop(self) -> None:
        if self._calibration is not None:
            self._calibration.stop()
        super().stop()

    @rpc
    def run(self, steps: list[str] | None = None) -> str:
        """Start calibrating: check, then ``steps`` (default: the ``steps`` setting).

        Returns "started" at once, or why not; ``status`` follows the run.
        """
        chosen = self.config.steps if steps is None else list(steps)
        unknown = sorted(set(chosen) - set(STEPS))
        if unknown:
            return f"unknown steps {unknown}; the steps are {list(STEPS)}"
        with self._run_lock:
            if self._thread is not None and self._thread.is_alive():
                return "already running"
            if not self._connected.is_set():
                return "not connected: no joint_state from the arm's connection yet"
            self._operator.begin(plan(chosen, self.robot.gravity))
            calibration = Calibration(
                self._spec,
                self.config.model_copy(update={"steps": chosen}),
                self._connection,
                self._operator,
                self.position_command.publish,
                self._torque_limit,
            )
            self._calibration = calibration
            self._thread = threading.Thread(target=calibration.run, daemon=True, name="calibration")
            self._thread.start()
        return "started"

    @rpc
    def status(self) -> CalibrationStatus:
        """What the run does, has found and asks; ``print`` it."""
        calibration = self._calibration
        return self._operator.status() if calibration is None else calibration.status()

    @rpc
    def answer(self, text: str) -> str:
        """Answer the pending question with one of ``status().answers``: "ok", or why not."""
        return self._operator.answer(text)

    @rpc
    def abort(self) -> str:
        """Stop streaming now (the connection holds the arm), then ask whether to go home."""
        with self._run_lock:
            if self._calibration is None or self._thread is None or not self._thread.is_alive():
                return "not running"
            self._calibration.abort()
        return "aborting"

    @rpc
    def bake(self, urdf_path: str, out_path: str | None = None) -> str:
        """Write the URDF at ``urdf_path`` with the fitted links of every arm of the robot that
        has a calibration file to ``out_path`` (default: ``calibrated.urdf`` beside the robot's
        files); says what it wrote, e.g. "wrote ...: baked left; right has no calibration file"."""
        return bake_calibration(self.robot, urdf_path, out_path)
