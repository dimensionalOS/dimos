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

The steps (``steps``) run on one thread. A motion streams one target per ``joint_state``
sample on ``position_command`` and the thread waits for it to end; while it asks the operator
or computes, nothing is sent, so the connection's deadman holds the arm and undoes live
changes. The operator is reached through the ``ArmCalibrateOperator/show``, ``/ask`` and
``/done`` RPCs that ``dimos arm calibrate`` serves; unanswered, the module waits and the arm
stays held. ``abort`` stops streaming at once and offers to go home.
"""

from __future__ import annotations

from collections.abc import Callable
import math
import threading
from typing import Any, Protocol

import numpy as np
from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.control.calibration import steps
from dimos.control.calibration.gravity_fit import Measurement
from dimos.control.calibration.gravity_poses import ArmModel, Pose
from dimos.control.calibration.motion import HoldMode, Motion, Sample, plan_path
from dimos.control.calibration.robots import KNOWN_ROBOTS, arm_spec
from dimos.control.calibration.spec import ArmCalibrationSpec, motor_torque_limit_nm
from dimos.control.calibration.step_analysis import RestNoise, StepMetrics
from dimos.control.calibration.steps import AbortedError, SkippedError
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

OPERATOR = "ArmCalibrateOperator"
Call = Callable[[str, list[Any], Callable[[Any], None]], Any]


class CalibrationModuleConfig(ModuleConfig):
    """Which arm to calibrate, and how. These are all of the calibration's settings.

    Attributes:
        robot: A ``robots.KNOWN_ROBOTS`` key, e.g. "dual-openyam".
        arm: The arm ("left"), its bus ("left_arm") or its id; needed when the robot has several.
        floor_z_m: The table's height from the arm's base, m. Required, with ``min_x_m``.
        min_x_m: The fingertip stays in front of this x from the base, m.
        poses: How many gravity poses to visit.
        speed_deg_s: Peak joint speed of every move; at most the URDF velocity limits.
        rest_window_s: How long the arm sits still to measure its noise; a hold is settled
            once one window of samples stays inside that noise.
        dither_deg: How far each measured joint is dithered either way at a gravity pose.
        kp_scales: Multiples of each joint's kp the gain sweep tries (kd times the square
            root); must include 1, the baseline.
        step_deltas_deg: The gain sweep's steps, applied in turn; both signs needed.
        max_overshoot_pct: A sweep step that overshoots this share of itself fails.
        float_drift_limit_mm: How far the fingertip may drift while a joint floats.
        confirm_poses: How many gravity poses confirm_tracking revisits.
        grid_points: About how many candidate poses the pose search considers.
        steps: Which steps to run after check (``steps.STEPS``).
        confirm_each: Ask before every move, not only at the start of each step.
    """

    robot: str
    arm: str | None = None
    floor_z_m: float | None = None
    min_x_m: float | None = None
    poses: int = Field(default=12, ge=3)
    speed_deg_s: float = Field(default=10.0, gt=0.0)
    rest_window_s: float = Field(default=0.5, gt=0.0)
    dither_deg: float = Field(default=1.0, gt=0.0)
    kp_scales: list[float] = Field(default_factory=lambda: [1.0, 1.5, 2.0])
    step_deltas_deg: list[float] = Field(default_factory=lambda: [1.0, -1.0, 3.0, -3.0])
    max_overshoot_pct: float = Field(default=20.0, gt=0.0)
    float_drift_limit_mm: float = Field(default=5.0, gt=0.0)
    confirm_poses: int = Field(default=3, ge=1)
    grid_points: int = Field(default=3000, ge=1)
    steps: list[str] = Field(default_factory=lambda: list(steps.STEPS))
    confirm_each: bool = True

    @model_validator(mode="after")
    def _check(self) -> Self:
        if self.robot not in KNOWN_ROBOTS:
            raise ValueError(f"unknown robot {self.robot!r}; known: {', '.join(KNOWN_ROBOTS)}")
        unknown = set(self.steps) - set(steps.STEPS)
        if unknown:
            raise ValueError(f"unknown steps {sorted(unknown)}; the steps are {list(steps.STEPS)}")
        if 1.0 not in self.kp_scales or min(self.kp_scales) <= 0.0:
            raise ValueError("kp_scales must be positive and include 1, the baseline")
        if not (min(self.step_deltas_deg) < 0.0 < max(self.step_deltas_deg)):
            raise ValueError("step_deltas_deg needs a positive and a negative step")
        return self

    def arm_spec(self) -> ArmCalibrationSpec:
        """The arm's spec on the bench guard; refused without one or above its velocity limits."""
        if self.floor_z_m is None or self.min_x_m is None:
            raise ValueError("give the table height and the front line: floor_z_m and min_x_m")
        spec = arm_spec(self.robot, self.arm).on_bench(self.floor_z_m, self.min_x_m)
        limits = {joint.name: joint.velocity for joint in spec.model.load().joints}
        slow = {
            name: f"{math.degrees(limit):.0f}"
            for name in spec.joint_names
            if (limit := limits[name]) is not None and math.radians(self.speed_deg_s) > limit
        }
        if slow:
            raise ValueError(f"speed_deg_s {self.speed_deg_s:g} exceeds URDF limits (deg/s) {slow}")
        return spec


class ArmConfigSpec(Spec, Protocol):
    """A Damiao connection deployed with ``config_writes``."""

    def get_config(self) -> dict[str, Any]: ...
    def set_config(self, patch: dict[str, Any], save: bool = False) -> dict[str, Any]: ...


class Calibration:
    """One calibration run: ``feed`` it every ``joint_state``, ``run`` it on its own thread.

    Args:
        spec: The arm.
        config: The settings.
        connection: The connection's ``get_config`` and ``set_config``.
        call: Calls an operator RPC by name with a reply callback (``rpc.call``'s form).
        publish: Sends one position command.
        torque_limit_nm: Each joint's motor torque limit, joint order: past it over kp off
            target, the motor cannot push through, and a motion stops.
        block: How the run thread waits on an event (tests step a fake arm instead).
    """

    def __init__(
        self,
        spec: ArmCalibrationSpec,
        config: CalibrationModuleConfig,
        connection: ArmConfigSpec,
        call: Call,
        publish: Callable[[JointState], None],
        torque_limit_nm: np.ndarray,
        block: Callable[[threading.Event], Any] = threading.Event.wait,
    ) -> None:
        self.spec, self.arm, self.config, self.connection = spec, ArmModel(spec), config, connection
        self._call, self._publish, self._block = call, publish, block
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
        self.measurements: list[Measurement] = []
        self.baseline: dict[str, np.ndarray] = {}
        self.step_metrics: list[StepMetrics] = []
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

    def wait(self, event: threading.Event) -> None:
        """Wait for ``event``; raises what interrupted the run (``abort``, ``stop``, a bad feed)."""
        with self._lock:
            if self._interrupt is not None:
                raise self._interrupt
            self._waiting = event
        try:
            self._block(event)
        finally:
            with self._lock:
                self._waiting = None
        with self._lock:
            if self._interrupt is not None:
                raise self._interrupt

    def _operator(self, name: str, *args: Any) -> Any:
        replies: list[Any] = []
        answered = threading.Event()

        def reply(value: Any) -> None:
            replies.append(value)
            answered.set()

        self._call(name, list(args), reply)
        self.wait(answered)
        if isinstance(replies[0], BaseException):
            raise replies[0]
        return replies[0]

    def show(self, text: str) -> None:
        self._operator("show", text)

    def ask(self, prompt: str, choices: list[str]) -> str:
        answer: str = self._operator("ask", prompt, choices)
        return answer

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

    def motion(self, path: list[np.ndarray], mode: HoldMode, **hold: Any) -> Motion:
        """A motion along ``path`` with the current gains, noise and settings."""
        kp = np.asarray([self.gains(joint)[0] for joint in self.spec.joint_names])
        stall = np.divide(self._torque_limit, kp, out=np.full(kp.shape, np.inf), where=kp > 0.0)
        band = [
            self.noise[joint].band_deg if self.noise else 0.0 for joint in self.spec.joint_names
        ]
        # Once measured, the noise's own window: a settled hold then outlasts it.
        window = max((n.window_s for n in self.noise.values()), default=self.config.rest_window_s)
        return Motion(
            self.arm,
            path,
            math.radians(self.config.speed_deg_s),
            mode,
            window,
            2.0 * np.radians(band),
            stall,
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
            raise RuntimeError(motion.error)
        return motion

    def move(self, target: np.ndarray, what: str, *, ask: bool = True) -> Motion:
        """Go to ``target`` along a clear path and settle there, asking first with ``confirm_each``."""
        if ask and self.config.confirm_each:
            degrees = ", ".join(f"{math.degrees(v):.1f}" for v in target)
            self.confirm(f"Move to {what} [{degrees}] deg?")
        motion = self.stream(self.motion(plan_path(self.arm, self.measured(), target), "settle"))
        self.at_home = what == "home"
        return motion

    def home(self, trail: list[np.ndarray] | None = None) -> None:
        """Go home, back through the poses of ``trail`` until a clear path home opens."""
        for back in reversed(trail or []):
            if self.arm.transition_ok(self.measured(), self.arm.home):
                break
            self.move(back, "the previous pose", ask=False)
        self.move(self.arm.home, "home")

    def run(self) -> None:
        """Every step in order; on a failure or abort the arm holds and home is offered."""
        ok, current, results = False, "check", {}
        try:
            chosen = ["check", *(step for step in self.config.steps if step != "check")]
            self.show(f"Calibrating {self.spec.arm_id}: {', '.join(chosen)}")
            results["check"] = steps.check(self)
            for current, reason in steps.plan(self.config.steps, self.state["can_float"]):
                if current == "check":
                    continue
                try:
                    results[current] = (
                        f"skipped: {reason}" if reason else steps.STEPS[current](self)
                    )
                except SkippedError:
                    results[current] = "skipped by the operator"
                self.show(f"{current}: {results[current]}")
            ok, text = True, "\n".join(f"  {step}: {result}" for step, result in results.items())
        except AbortedError as error:
            text = f"{current}: {error}"
        except Exception as error:
            logger.exception(f"calibration step {current} failed")
            text = f"{current} failed: {type(error).__name__}: {error}"
        if self._stopping:
            return
        if not ok:
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
        self._operator("done", ok, text, self.at_home)


class CalibrationModule(Module):
    """Calibrates one arm (see the module docstring); ``abort`` is its only RPC."""

    config: CalibrationModuleConfig
    _connection: ArmConfigSpec
    joint_state: In[JointState]
    position_command: Out[JointState]

    _calibration: Calibration | None = None

    @rpc
    def start(self) -> None:
        super().start()
        spec = self.config.arm_spec()
        motors = {m.name: m for bus in KNOWN_ROBOTS[self.config.robot].buses for m in bus.arm}
        rpc_client = self.rpc

        def call(name: str, args: list[Any], reply: Callable[[Any], None]) -> Any:
            return rpc_client.call(f"{OPERATOR}/{name}", (args, {}), reply)

        calibration = Calibration(
            spec,
            self.config,
            self._connection,
            call,
            self.position_command.publish,
            np.asarray([motor_torque_limit_nm(motors[name]) for name in spec.joint_names]),
        )
        self._calibration = calibration
        self.register_disposable(Disposable(self.joint_state.subscribe(calibration.feed)))
        threading.Thread(target=calibration.run, daemon=True, name="calibration").start()

    @rpc
    def stop(self) -> None:
        if self._calibration is not None:
            self._calibration.stop()
        super().stop()

    @rpc
    def abort(self) -> None:
        """Stop streaming now (the connection holds the arm), then offer to go home."""
        assert self._calibration is not None, "the calibration module has not started"
        self._calibration.abort()
