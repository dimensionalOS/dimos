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

"""The arm calibration procedure: steps run in order on a worker thread, driven by calls.

A :class:`Procedure` runs a list of :class:`Step` (``steps.calibration_steps``)
on one arm. ``plan`` lists them with whether each can run on this arm;
``start`` runs them in order on a thread of its own; ``status`` says where it
is. A step that needs the operator stops at a confirmation
(:meth:`StepContext.confirm`, state ``waiting_confirm``) until ``confirm``
(go on), ``skip`` (skip what was asked: one pose, or the rest of the step) or
``abort`` (stop everything). An arm in a config session keeps being streamed
its target while it waits.

A step that fails stops the procedure (``failed``), with the arm held where it
is; so does ``abort`` (``aborted``). Neither moves the arm home. Two things do,
without a confirmation and on a time budget (:meth:`ArmSession.park`, at most
``MAX_PARK_BUDGET_S`` at up to ``MAX_PARK_SPEED_DEG_S``): :meth:`Procedure.
shutdown`, which the module's ``stop`` calls so the arm is not left at a pose
when the connection de-energizes it, and a prompt left unanswered for
``ProcedureSettings.prompt_timeout_s``, which aborts the run. When the way home
is not guard-safe or would take longer than the budget, the arm holds where it
is and the reason is logged as an error; ``status()["park"]`` says what the
last park did. Everything the calls return is plain JSON: strings, numbers,
booleans, lists and dicts.

Nothing here knows the transport: the arm is a :class:`~dimos.robot.
manipulators.calibration.hooks.ArmConfigHooks` connection (an RPC proxy in a
``dimos run``, a fake in tests), its readings come from a :class:`~dimos.
robot.manipulators.calibration.arm_session.StateSource` (the connection's
``joint_state`` stream), markers and status go to an :class:`EventSink`, and
analyses read a :class:`~dimos.robot.manipulators.calibration.recording.
RecordingSource`.
"""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
import math
import threading
import time
from typing import Any, Literal, Protocol, TypedDict
import uuid

from dimos.control.connection.arm_config import (
    DEFAULT_CONFIG_LEASE_S,
    MAX_CONFIG_LEASE_S,
    ArmConfigState,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
)
from dimos.robot.manipulators.calibration.arm_session import (
    AbortRequestedError,
    ArmSession,
    Clock,
    MarkerStream,
    ParkRefusedError,
    RealClock,
    SessionHost,
    StateSource,
    StepFailedError,
    TrackingLog,
    plain_error,
)
from dimos.robot.manipulators.calibration.gravity_poses import (
    DEFAULT_POSE_COUNT,
    ArmModel,
    PoseSet,
)
from dimos.robot.manipulators.calibration.hooks import ArmConfigHooks
from dimos.robot.manipulators.calibration.motion import (
    DEFAULT_SPEED_DEG_S,
    check_step,
    speed_rad_s,
)
from dimos.robot.manipulators.calibration.recording import RecordingSource
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ProcedureStateName = Literal["idle", "running", "waiting_confirm", "done", "failed", "aborted"]
StepStatusName = Literal[
    "pending", "running", "done", "skipped", "failed", "aborted", "unsupported", "not_selected"
]

DEFAULT_KP_SCALES = (1.0, 1.5, 2.0)
DEFAULT_STEP_DELTAS_DEG = (1.0, -1.0, 3.0, -3.0)
DEFAULT_DITHER_DEG = 1.0
DEFAULT_HOLD_S = 2.0
DEFAULT_DITHER_HOLD_S = 1.5
DEFAULT_STEP_HOLD_S = 2.0
# Damping a floating joint keeps: stops it coasting, still yields to a hand.
DEFAULT_FLOAT_KD = 0.5
DEFAULT_FLOAT_SETTLE_S = 1.0
DEFAULT_FLOAT_WINDOW_S = 10.0
DEFAULT_TRACKING_POSES = 5
MIN_STREAM_HZ = 50.0
# Parking: the arm goes home on its own, so slowly-ish and within a budget that
# fits a module's stop.
MAX_PARK_BUDGET_S = 8.0
MAX_PARK_SPEED_DEG_S = 20.0
DEFAULT_PROMPT_TIMEOUT_S = 600.0
# How long a stopping procedure waits for its run to let go of the arm before
# it parks, real seconds; the run lets go within a tick of being aborted.
PARK_SLOT_WAIT_S = 2.0
# What ``status()["step"]`` names while the arm goes home or parks, which is no step.
HOME_ACTION = "home"
PARK_ACTION = "park"
_FINISHED: tuple[ProcedureStateName, ...] = ("idle", "done", "failed", "aborted")


class SkipRequestedError(Exception):
    """The operator skipped what a confirmation asked for."""


class StepUnavailableError(Exception):
    """A step cannot run in this procedure (e.g. nothing to fit); it is skipped with the reason."""


@dataclass(frozen=True)
class ProcedureSettings:
    """How the procedure moves the arm and how long it waits.

    Attributes:
        poses: Gravity identification poses to visit.
        dither_deg: Dither of each measured joint at every pose, +d then -d.
        hold_s: Rest at each pose before dithering, and at each tracking pose.
        dither_hold_s: Rest after each dither.
        step_hold_s: Rest after each step of the gain sweep.
        speed_deg_s: Peak joint speed of every ramp, at most ``MAX_SPEED_DEG_S``.
        confirm_each: Ask before every pose (and every joint of the gain sweep).
        kp_scales: Multiples of each joint's robot kp the gain sweep tries.
        step_deltas_deg: The sweep's steps, in order; they sum to zero.
        sweep_pose_deg: Where the gain sweep steps the joints, one angle per arm
            joint in degrees; ``None`` means the gravity pose farthest from
            every joint limit.
        float_pose_deg: One pose every joint floats at, one angle per arm joint
            in degrees; ``None`` means, per joint, the gravity poses where it
            carries the most gravity torque.
        float_kd: Damping a floating joint keeps, N m s/rad.
        float_settle_s: Wait after a joint starts floating before its window.
        float_window_s: How long a floating joint is watched for drift.
        tracking_poses: At most this many held-out gravity poses for the
            tracking check.
        stream_hz: Targets sent per second during a config session.
        lease_s: The config session's lease, at most ``MAX_CONFIG_LEASE_S``.
        park_budget_s: How long parking may take, at most ``MAX_PARK_BUDGET_S``.
        park_speed_deg_s: Peak joint speed of a park, at most ``MAX_PARK_SPEED_DEG_S``.
        prompt_timeout_s: A prompt unanswered this long aborts the run and parks.
    """

    poses: int = DEFAULT_POSE_COUNT
    dither_deg: float = DEFAULT_DITHER_DEG
    hold_s: float = DEFAULT_HOLD_S
    dither_hold_s: float = DEFAULT_DITHER_HOLD_S
    step_hold_s: float = DEFAULT_STEP_HOLD_S
    speed_deg_s: float = DEFAULT_SPEED_DEG_S
    confirm_each: bool = True
    kp_scales: tuple[float, ...] = DEFAULT_KP_SCALES
    step_deltas_deg: tuple[float, ...] = DEFAULT_STEP_DELTAS_DEG
    sweep_pose_deg: tuple[float, ...] | None = None
    float_pose_deg: tuple[float, ...] | None = None
    float_kd: float = DEFAULT_FLOAT_KD
    float_settle_s: float = DEFAULT_FLOAT_SETTLE_S
    float_window_s: float = DEFAULT_FLOAT_WINDOW_S
    tracking_poses: int = DEFAULT_TRACKING_POSES
    stream_hz: float = MIN_STREAM_HZ
    lease_s: float = DEFAULT_CONFIG_LEASE_S
    park_budget_s: float = MAX_PARK_BUDGET_S
    park_speed_deg_s: float = MAX_PARK_SPEED_DEG_S
    prompt_timeout_s: float = DEFAULT_PROMPT_TIMEOUT_S

    def __post_init__(self) -> None:
        speed_rad_s(self.speed_deg_s)
        check_step(self.dither_deg)
        for delta in self.step_deltas_deg:
            check_step(delta)
        if abs(sum(self.step_deltas_deg)) > 1e-9:
            raise ValueError(f"the sweep's steps must return the joint, got {self.step_deltas_deg}")
        if self.poses < 1 or self.tracking_poses < 0:
            raise ValueError("poses must be at least 1 and tracking_poses not negative")
        holds = (self.hold_s, self.dither_hold_s, self.step_hold_s, self.float_window_s)
        if not all(math.isfinite(s) and s > 0.0 for s in holds) or self.float_settle_s < 0.0:
            raise ValueError("holds and windows must be positive")
        if not self.kp_scales or not all(math.isfinite(s) and s > 0.0 for s in self.kp_scales):
            raise ValueError(f"kp scales must be positive, got {self.kp_scales}")
        if not math.isfinite(self.float_kd) or self.float_kd < 0.0:
            raise ValueError(f"float kd must not be negative, got {self.float_kd}")
        if not self.stream_hz >= MIN_STREAM_HZ:
            raise ValueError(f"stream_hz must be at least {MIN_STREAM_HZ:g}, got {self.stream_hz}")
        if not 0.0 < self.lease_s <= MAX_CONFIG_LEASE_S:
            raise ValueError(
                f"lease_s must be in (0, {MAX_CONFIG_LEASE_S:g}] s, got {self.lease_s}"
            )
        if not 0.0 < self.park_budget_s <= MAX_PARK_BUDGET_S:
            raise ValueError(
                f"park_budget_s must be in (0, {MAX_PARK_BUDGET_S:g}] s, got {self.park_budget_s}"
            )
        if not 0.0 < self.park_speed_deg_s <= MAX_PARK_SPEED_DEG_S:
            raise ValueError(
                f"park_speed_deg_s must be in (0, {MAX_PARK_SPEED_DEG_S:g}] deg/s, "
                f"got {self.park_speed_deg_s}"
            )
        if not (math.isfinite(self.prompt_timeout_s) and self.prompt_timeout_s > 0.0):
            raise ValueError(f"prompt_timeout_s must be positive, got {self.prompt_timeout_s}")


@dataclass(frozen=True)
class Capabilities:
    """What the arm and the run offer the steps.

    Attributes:
        torque_readback: The motors report the torque they produce.
        gravity_compensation: The connection adds each joint's gravity torque.
        set_gains: Gains can be changed live.
        recording_unavailable: Why nothing recorded can be read back, or
            ``None`` when it can.
    """

    torque_readback: bool = True
    gravity_compensation: bool = True
    set_gains: bool = True
    recording_unavailable: str | None = None


@dataclass
class RunState:
    """What one ``start`` learns as it goes, for the steps after.

    Attributes:
        run_id: Names this run in its markers.
        pose_set: The gravity identification poses the sweep visited, once generated.
        generated_poses: The same poses generated for a later step when the
            sweep did not run.
        gravity_span: When the gravity sweep's markers start and end.
        gravity_applied: How the arm was driven during the sweep, as the
            connection reported it (``config_get``): its file and link masses.
        step_span: When the gain sweep started and ended.
        baseline: Tracking with the values the arm had before this run, by pose.
    """

    run_id: str
    pose_set: PoseSet | None = None
    generated_poses: PoseSet | None = None
    gravity_span: tuple[float, float] | None = None
    gravity_applied: ArmConfigState | None = None
    step_span: tuple[float, float] | None = None
    baseline: TrackingLog | None = None


@dataclass(frozen=True)
class StepResult:
    """What a step reports: a line for the operator and plain-JSON data."""

    message: str
    data: dict[str, Any] = field(default_factory=dict)
    skipped: bool = False


class PlannedStep(TypedDict):
    id: str
    title: str
    instructions: str
    supported: bool
    reason: str | None


class StepState(TypedDict):
    id: str
    title: str
    status: StepStatusName
    message: str


class Progress(TypedDict):
    done: int
    total: int
    label: str


class ProcedureStatus(TypedDict):
    """Where the procedure is.

    Attributes:
        state: idle, running, waiting_confirm, done, failed or aborted.
        arm_id: The arm.
        run_id: The current or last run, ``None`` before the first.
        step: The step running or waiting, else the last one.
        message: What is happening, or what happened, in words.
        prompt: What the operator is asked while ``waiting_confirm``.
        prompt_id: Counts the prompts of this procedure; pass it to ``confirm``
            or ``skip`` so a late answer cannot answer the next prompt.
        prompt_deadline: When the waiting prompt times out, aborting the run
            and parking the arm, Unix seconds.
        progress: The current step's progress, if it counts.
        steps: Every step and its status.
        park: What the last park did ("parked at home ...", or why the arm
            holds where it is), ``None`` before the first.
        updated: When this changed, Unix seconds.
    """

    state: ProcedureStateName
    arm_id: str
    run_id: str | None
    step: str | None
    message: str
    prompt: str | None
    prompt_id: int | None
    prompt_deadline: float | None
    progress: Progress | None
    steps: list[StepState]
    park: str | None
    updated: float


class EventSink(Protocol):
    """Where markers and status go: the module's streams, or a test's lists."""

    def marker(self, stream: MarkerStream, payload: dict[str, Any]) -> None: ...

    def status(self, status: ProcedureStatus) -> None: ...

    def probe(self, token: str) -> None:
        """Publish ``token`` where the recorder records it, and nowhere a report reads."""
        ...


@dataclass(frozen=True)
class _ParkRequest:
    reason: str
    # Clock time the park must be done by.
    deadline: float
    # Asked for by shutdown: no run starts after it.
    final: bool


class _ParkHost:
    """A park's session host: only a second abort stops it, and it marks nothing."""

    def __init__(self, cancel: threading.Event) -> None:
        self._cancel = cancel

    def check_abort(self) -> None:
        if self._cancel.is_set():
            raise AbortRequestedError("the park was aborted")

    def marker(self, stream: MarkerStream, payload: dict[str, Any]) -> None:
        pass


class Step:
    """One calibration step: what it does, whether it can run, and how.

    Attributes:
        id: Names the step in calls and results.
        title: A few words for a list.
        instructions: One paragraph for the operator, shown before it runs.
    """

    id: str = ""
    title: str = ""
    instructions: str = ""

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        """True when the step can run on this arm, else why not."""
        return True

    def run(self, ctx: StepContext) -> StepResult:
        raise NotImplementedError


class StepContext:
    """What a running step uses: the arm, its settings, confirmations and the recording."""

    def __init__(self, procedure: Procedure, run: RunState) -> None:
        self._procedure = procedure
        self.run = run

    @property
    def spec(self) -> ArmCalibrationSpec:
        return self._procedure.spec

    @property
    def arm(self) -> ArmModel:
        return self._procedure.arm

    @property
    def settings(self) -> ProcedureSettings:
        return self._procedure.settings

    @property
    def caps(self) -> Capabilities:
        return self._procedure.caps

    @property
    def connection(self) -> ArmConfigHooks:
        return self._procedure.connection

    @property
    def state(self) -> StateSource | None:
        """The arm's joint readings."""
        return self._procedure.state

    def keys(self) -> ConfigKeys:
        """What the connection can read and change; see ``Procedure.keys``."""
        return self._procedure.keys()

    def persist(
        self, patch: ConfigPatch, *, force: bool = False, apply_now: bool = True
    ) -> ConfigApplied:
        """Save ``patch`` to the arm's calibration file; see ``Procedure.persist``."""
        return self._procedure.persist(patch, force=force, apply_now=apply_now)

    @property
    def clock(self) -> Clock:
        return self._procedure.clock

    @property
    def recording(self) -> RecordingSource | None:
        return self._procedure.recording

    @property
    def results(self) -> dict[str, dict[str, Any]]:
        """The data of the steps already done in this run, by step id."""
        return self._procedure.results()

    @property
    def side(self) -> str:
        """The arm's name, the last part of its id: "left", or "openyam"."""
        return self.spec.arm_id.rsplit("/", 1)[-1]

    def confirm(self, prompt: str) -> None:
        """Wait for the operator: returns on ``confirm``, raises ``SkipRequestedError`` or ``AbortRequestedError``.

        Unanswered for ``settings.prompt_timeout_s``, it aborts the run and parks the arm.
        """
        self._procedure.wait_for_operator(prompt)

    def probe_recording(self, token: str) -> None:
        """Publish ``token`` for the recorder; see ``EventSink.probe``."""
        self._procedure.probe(token)

    def progress(self, done: int, total: int, label: str) -> None:
        self._procedure.set_progress(Progress(done=done, total=total, label=label))

    def say(self, message: str) -> None:
        self._procedure.set_message(message)

    def check_abort(self) -> None:
        self._procedure.check_abort()

    def step_states(self) -> list[StepState]:
        """Every step of this run and its status so far."""
        return self._procedure.status()["steps"]

    def require_recording(self) -> RecordingSource:
        recording = self.recording
        if recording is None:
            raise StepUnavailableError(
                self.caps.recording_unavailable or "nothing is being recorded"
            )
        return recording

    @contextmanager
    def session(self) -> Iterator[ArmSession]:
        """A config session on the arm for the ``with`` block; ended however the block ends."""
        with self._procedure.open_session() as session:
            yield session


class Procedure:
    """Runs calibration steps on one arm, one run at a time; every public method is thread-safe.

    Args:
        spec: The arm.
        connection: Its connection.
        steps: The steps, in order (``steps.calibration_steps()``).
        state: The arm's joint readings, the connection's ``joint_state``.
        settings: How it moves and waits.
        caps: What the arm and the run offer.
        recording: Where recorded markers and joint states are read back.
        events: Where markers and status go.
        clock: Time and waiting.
        arm: ``ArmModel(spec)``, when already built.
    """

    def __init__(
        self,
        spec: ArmCalibrationSpec,
        connection: ArmConfigHooks,
        steps: Sequence[Step],
        *,
        state: StateSource | None = None,
        settings: ProcedureSettings | None = None,
        caps: Capabilities | None = None,
        recording: RecordingSource | None = None,
        events: EventSink | None = None,
        clock: Clock | None = None,
        arm: ArmModel | None = None,
    ) -> None:
        ids = [step.id for step in steps]
        if len(set(ids)) != len(ids) or not all(ids):
            raise ValueError(f"step ids must be distinct and non-empty, got {ids}")
        self.spec = spec
        self.connection = connection
        self.state = state
        self.settings = settings or ProcedureSettings()
        self.caps = caps or Capabilities()
        self.recording = recording
        self.clock = clock or RealClock()
        self.arm = arm if arm is not None else ArmModel(spec)
        self._steps = list(steps)
        self._events = events
        # Guards everything below; held only briefly, never across a step.
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._abort = threading.Event()
        # Set when a decision (confirm, skip or abort) arrives for a prompt.
        self._decided = threading.Event()
        self._decision: Literal["confirm", "skip"] | None = None
        self._session: ArmSession | None = None
        # What the connection can read and change, once asked.
        self._keys: ConfigKeys | None = None
        # Taken from before a session begins until after it ends.
        self._slot_taken = False
        self._state: ProcedureStateName = "idle"
        self._run_id: str | None = None
        self._current: str | None = None
        self._message = "ready"
        self._prompt: str | None = None
        self._prompts = 0
        self._prompt_deadline: float | None = None
        self._progress: Progress | None = None
        # A park asked for (by shutdown or a prompt timeout), whether a thread
        # has taken it on, set once it is done, and what it did.
        self._park_request: _ParkRequest | None = None
        self._park_claimed = False
        self._park_done = threading.Event()
        self._park_cancel = threading.Event()
        self._park_outcome: str | None = None
        self._step_states: dict[str, StepState] = {}
        self._results: dict[str, dict[str, Any]] = {}
        self._updated = self.clock.now()
        self._reset_steps()

    def _reset_steps(self) -> None:
        self._step_states = {
            step.id: StepState(id=step.id, title=step.title, status="pending", message="")
            for step in self._steps
        }

    def set_capabilities(self, caps: Capabilities) -> bool:
        """Replace what the arm and the run offer, e.g. once the recording has started.

        Nothing changes while a run is in progress; returns whether it changed.
        """
        with self._lock:
            if self._state not in _FINISHED:
                return False
            self.caps = caps
            return True

    def keys(self) -> ConfigKeys:
        """What the connection can read and change (``config_keys``), asked once and kept.

        Raises ``StepFailedError`` when the connection does not answer.
        """
        with self._lock:
            keys = self._keys
        if keys is None:
            try:
                keys = self.connection.config_keys()
            except Exception as error:
                raise StepFailedError(
                    f"the connection did not say what it serves: {plain_error(error)}"
                )
            with self._lock:
                self._keys = keys
        return keys

    def persist(
        self, patch: ConfigPatch, *, force: bool = False, apply_now: bool = True
    ) -> ConfigApplied:
        """Save ``patch`` to the arm's calibration file through the connection (scope "persist").

        The connection merges it into the file and applies it; see
        ``ConnectionModule.config_set``. Inside a config session the call
        carries its id, as the connection requires.
        """
        with self._lock:
            session = self._session
        return self.connection.config_set(
            patch,
            scope="persist",
            force=force,
            apply_now=apply_now,
            session_id=None if session is None else session.session_id,
        )

    def plan(self) -> list[PlannedStep]:
        """Every step, its instructions, and whether it can run on this arm."""
        planned: list[PlannedStep] = []
        for step in self._steps:
            support = step.supported(self.spec, self.caps)
            planned.append(
                PlannedStep(
                    id=step.id,
                    title=step.title,
                    instructions=step.instructions,
                    supported=support is True,
                    reason=None if support is True else str(support),
                )
            )
        return planned

    def start(self, steps: Sequence[str] | None = None) -> ProcedureStatus:
        """Run the steps (all, or those named) in order on a new thread; refused while one runs."""
        known = [step.id for step in self._steps]
        selected = set(known if steps is None else steps)
        unknown = sorted(selected - set(known))
        if unknown:
            raise ValueError(f"unknown steps {unknown}; the steps are {known}")
        with self._lock:
            if self._state not in _FINISHED:
                raise RuntimeError(f"the procedure is already {self._state}")
            previous = self._thread
        if previous is not None:
            # It has set its final state; let it return before the next run.
            previous.join()
        with self._lock:
            if self._state not in _FINISHED or self._thread is not previous:
                raise RuntimeError("the procedure was started meanwhile")
            if self._park_request is not None:
                raise RuntimeError("the procedure is shutting down")
            self._abort.clear()
            self._decided.clear()
            self._decision = None
            self._run_id = uuid.uuid4().hex[:8]
            self._results = {}
            self._reset_steps()
            for step_id in set(known) - selected:
                self._step_states[step_id]["status"] = "not_selected"
                self._step_states[step_id]["message"] = "not selected"
            self._progress = None
            self._prompt = None
            self._current = None
            self._set_state("running", "starting")
            run = RunState(run_id=self._run_id)
            self._thread = threading.Thread(
                target=self._run,
                args=(run, selected),
                name=f"calibration-{self.spec.arm_id}",
                daemon=True,
            )
            self._thread.start()
            return self._status_locked()

    def go_home(self) -> ProcedureStatus:
        """Ramp to the spec's home after a confirmation; only while no run is on.

        Ends ``done`` (or ``aborted`` when the operator aborts, ``failed`` when
        the move fails); a skipped confirmation leaves the arm where it is.
        """
        with self._lock:
            if self._state not in _FINISHED:
                raise RuntimeError(f"the procedure is already {self._state}")
            previous = self._thread
        if previous is not None:
            previous.join()
        with self._lock:
            if self._state not in _FINISHED or self._thread is not previous:
                raise RuntimeError("the procedure was started meanwhile")
            if self._park_request is not None:
                raise RuntimeError("the procedure is shutting down")
            self._abort.clear()
            self._decided.clear()
            self._decision = None
            self._progress = None
            self._prompt = None
            self._current = HOME_ACTION
            self._set_state("running", "going home")
            self._thread = threading.Thread(
                target=self._run_home, name=f"calibration-home-{self.spec.arm_id}", daemon=True
            )
            self._thread.start()
            return self._status_locked()

    def confirm(self, prompt_id: int) -> bool:
        """Answer prompt ``prompt_id`` (``status()["prompt_id"]``) with "go on".

        False when it is not the prompt waiting. Raises ``TypeError`` without
        a prompt id: an answer must name the prompt it answers.
        """
        return self._decide("confirm", prompt_id)

    def skip(self, prompt_id: int) -> bool:
        """Answer prompt ``prompt_id`` with "skip"; False when it is not the one waiting."""
        return self._decide("skip", prompt_id)

    def _decide(self, decision: Literal["confirm", "skip"], prompt_id: int) -> bool:
        if isinstance(prompt_id, bool) or not isinstance(prompt_id, int):
            raise TypeError(
                f"{decision} needs the prompt_id of the prompt it answers "
                f'(status()["prompt_id"]), got {prompt_id!r}'
            )
        with self._lock:
            if self._state != "waiting_confirm" or self._decision is not None:
                return False
            if prompt_id != self._prompts:
                return False
            self._decision = decision
            self._decided.set()
            return True

    def abort(self) -> bool:
        """Stop the run now: no new targets, the config session ends, the arm holds where it is.

        The arm is not moved home; a park under way stops too. False when nothing runs.
        """
        with self._lock:
            if self._state in _FINISHED:
                return False
            self._abort.set()
            self._decided.set()
            if self._park_claimed:
                self._park_cancel.set()
            self._message = "aborting"
        try:
            # Ends the session at once, even while the run thread waits on a call;
            # config_reset needs no session id, so this thread may end it.
            self.connection.config_reset()
        except Exception as error:
            logger.warning(f"config_reset on abort failed: {plain_error(error)}")
        return True

    def status(self) -> ProcedureStatus:
        with self._lock:
            return self._status_locked()

    def results(self) -> dict[str, dict[str, Any]]:
        """The data of every step done in the current or last run, by step id."""
        with self._lock:
            return dict(self._results)

    def join(self, timeout: float | None = None) -> bool:
        """Wait for the run thread; whether it has finished."""
        with self._lock:
            thread = self._thread
        if thread is None or thread is threading.current_thread():
            return True
        thread.join(timeout)
        return not thread.is_alive()

    def close(self, timeout: float | None = None) -> None:
        """Abort any run and wait for its thread; the arm holds where it is."""
        self.abort()
        self.join(timeout)

    def shutdown(self, reason: str = "the calibration module is stopping") -> str:
        """Park the arm, then end: abort any run and back out home on the park budget.

        What a module's ``stop`` calls before its connection may de-energize
        the arm. A run lets go of the arm within a tick; whichever thread is
        free first parks it (``ArmSession.park``). Returns what the park did,
        logged as an error when the arm was left where it is.
        """
        settings = self.settings
        self._request_park(reason, final=True)
        thread: threading.Thread | None
        with self._lock:
            thread = self._thread if self._thread is not None and self._thread.is_alive() else None
        # Park from here once no run holds the arm, unless the run parks first.
        waited_until = time.monotonic() + PARK_SLOT_WAIT_S
        while True:
            with self._lock:
                if self._park_claimed:
                    claimed = False
                    break
                if not self._slot_taken:
                    self._park_claimed = claimed = True
                    break
            if time.monotonic() >= waited_until:
                with self._lock:
                    # Nobody parks after this: the module is about to stop.
                    self._park_claimed = True
                outcome = (
                    f"held where it is, not parked ({reason}): the run did not let go of the "
                    f"arm within {PARK_SLOT_WAIT_S:g} s"
                )
                logger.error(f"{self.spec.arm_id}: {outcome}")
                self._finish_park(outcome)
                claimed = False
                break
            time.sleep(0.01)
        if claimed:
            self._park_now()
        self._park_done.wait(settings.park_budget_s + PARK_SLOT_WAIT_S)
        if thread is not None:
            thread.join(settings.park_budget_s + PARK_SLOT_WAIT_S)
        with self._lock:
            return self._park_outcome or "held where it is: the park did not finish"

    # Parking.

    def _request_park(self, reason: str, *, final: bool) -> None:
        """Ask for one park: aborts any run, which parks on its way out unless shutdown does.

        A ``final`` park (shutdown's) refuses every later run; another one is
        released once its run has ended.
        """
        with self._lock:
            if self._park_request is None:
                self._park_request = _ParkRequest(
                    reason, self.clock.now() + self.settings.park_budget_s, final
                )
                self._park_claimed = False
                self._park_outcome = None
                self._park_done.clear()
                self._park_cancel.clear()
            elif final:
                self._park_request = replace(self._park_request, final=True)
            self._abort.set()
            self._decided.set()

    def _release_park(self) -> None:
        """After a run's own park (a prompt timeout's), let the next run start."""
        with self._lock:
            if self._park_request is not None and not self._park_request.final:
                self._park_request = None
                self._park_claimed = False

    def _park_if_requested(self) -> str | None:
        """On a run thread's way out: park if asked and nobody has; ``None`` if not asked."""
        with self._lock:
            if self._park_request is None:
                return None
            claimed = not self._park_claimed
            self._park_claimed = True
        if claimed:
            self._park_now()
        budget = self.settings.park_budget_s + PARK_SLOT_WAIT_S
        self._park_done.wait(budget)
        with self._lock:
            return self._park_outcome or "held where it is: the park did not finish"

    def _park_now(self) -> None:
        """Park in a session of its own, then put back the state it found unless a run moved on."""
        request = self._park_request
        assert request is not None
        settings = self.settings
        with self._lock:
            before = (self._state, self._message, self._current)
            self._current = PARK_ACTION
            self._progress = None
            self._set_state(
                "running",
                f"parking ({request.reason}): the arm goes home at up to "
                f"{settings.park_speed_deg_s:g} deg/s within {settings.park_budget_s:g} s",
            )
        try:
            with self.open_session(host=_ParkHost(self._park_cancel)) as session:
                done = session.park(speed_rad_s(settings.park_speed_deg_s), request.deadline)
            outcome = f"{done} ({request.reason})"
            logger.info(f"{self.spec.arm_id}: {outcome}")
        except (ParkRefusedError, StepFailedError, AbortRequestedError) as refused:
            outcome = f"held where it is, not parked ({request.reason}): {plain_error(refused)}"
        except Exception as error:
            outcome = (
                f"held where it is, not parked ({request.reason}): {type(error).__name__}: "
                f"{plain_error(error)}"
            )
        if outcome.startswith("held"):
            logger.error(
                f"{self.spec.arm_id}: {outcome}. Support the arm: it drops when its connection "
                "stops."
            )
        with self._lock:
            if self._state == "running" and self._current == PARK_ACTION:
                state, message, self._current = before
                self._state = state
                self._message = f"{message}; {outcome}" if state in _FINISHED else message
        self._finish_park(outcome)

    def _finish_park(self, outcome: str) -> None:
        with self._lock:
            if self._park_outcome is None:
                self._park_outcome = outcome
            self._touch()
        self._park_done.set()

    # Called from the run thread, through the step context.

    def check_abort(self) -> None:
        if self._abort.is_set():
            raise AbortRequestedError("aborted by the operator")

    def marker(self, stream: MarkerStream, payload: dict[str, Any]) -> None:
        data = payload.get("data")
        if isinstance(data, dict):
            data.setdefault("run", self._run_id)
            data.setdefault("arm", self.spec.arm_id)
        if self._events is not None:
            self._events.marker(stream, payload)

    def wait_for_operator(self, prompt: str) -> None:
        self.check_abort()
        timeout = self.settings.prompt_timeout_s
        deadline = self.clock.now() + timeout
        with self._lock:
            self._decision = None
            self._decided.clear()
            self._prompt = prompt
            self._prompts += 1
            self._prompt_deadline = deadline
            self._set_state("waiting_confirm", prompt)
            session = self._session
        try:
            if session is not None:
                answered = session.wait_for(self._decided, deadline)
            else:
                answered = False
                while self.clock.now() < deadline:
                    left = min(1.0 / MIN_STREAM_HZ, deadline - self.clock.now())
                    if self.clock.wait(self._decided, left):
                        answered = True
                        break
        finally:
            with self._lock:
                decision = self._decision
                self._decision = None
                self._prompt = None
                self._prompt_deadline = None
                self._set_state("running", "running")
        self.check_abort()
        if not answered:
            unanswered = f"the prompt went unanswered for {_duration(timeout)}"
            logger.warning(f"{self.spec.arm_id}: {unanswered}; aborting and parking: {prompt}")
            self._request_park(unanswered, final=False)
            raise AbortRequestedError(unanswered)
        if decision == "skip":
            raise SkipRequestedError(prompt)

    def probe(self, token: str) -> None:
        if self._events is not None:
            self._events.probe(token)

    def set_progress(self, progress: Progress) -> None:
        with self._lock:
            self._progress = progress
            self._message = progress["label"]
            self._touch()

    def set_message(self, message: str) -> None:
        with self._lock:
            self._message = message
            self._touch()

    @contextmanager
    def open_session(self, host: SessionHost | None = None) -> Iterator[ArmSession]:
        """A config session on the arm; one at a time, ``host`` this procedure unless given.

        Once aborted (a park asked for included), only a park opens one.
        """
        with self._lock:
            if host is None:
                self.check_abort()
            if self._slot_taken:
                raise RuntimeError("a step already holds a config session")
            self._slot_taken = True
        try:
            session = ArmSession(
                self.connection,
                self.state,
                self.keys().source,
                self.spec,
                self.arm,
                host or self,
                self.clock,
                speed_rad_s=speed_rad_s(self.settings.speed_deg_s),
                stream_hz=self.settings.stream_hz,
                lease_s=self.settings.lease_s,
            )
            with session:
                with self._lock:
                    self._session = session
                try:
                    yield session
                finally:
                    with self._lock:
                        self._session = None
        finally:
            with self._lock:
                self._slot_taken = False

    # The run thread.

    def _run(self, run: RunState, selected: set[str]) -> None:
        try:
            self._run_steps(run, selected)
        except Exception as error:
            logger.exception("the calibration procedure raised")
            with self._lock:
                self._set_state("failed", f"{type(error).__name__}: {plain_error(error)}")

    def _run_steps(self, run: RunState, selected: set[str]) -> None:
        ctx = StepContext(self, run)
        try:
            for step in self._steps:
                if step.id not in selected:
                    continue
                support = step.supported(self.spec, self.caps)
                if support is not True:
                    self._finish_step(step, "unsupported", str(support))
                    continue
                self.check_abort()
                if not self._run_step(step, ctx):
                    return
            with self._lock:
                self._current = None
                self._progress = None
                self._set_state("done", "every step has run")
        except AbortRequestedError:
            with self._lock:
                aborted = self._current
                if aborted is not None:
                    self._step_states[aborted]["status"] = "aborted"
            parked = self._park_if_requested()
            with self._lock:
                where = f" during {aborted}" if aborted else ""
                self._current = aborted
                self._set_state(
                    "aborted",
                    f"aborted{where}: the arm holds where it is; it was not moved home"
                    if parked is None
                    else f"aborted{where}: {parked}",
                )
            self._release_park()

    def _run_home(self) -> None:
        try:
            self._home()
        except AbortRequestedError:
            parked = self._park_if_requested()
            self._end_home(
                "aborted",
                "aborted going home: the arm holds where it is"
                if parked is None
                else f"aborted going home: {parked}",
            )
            self._release_park()
        except SkipRequestedError:
            self._end_home("done", "not moved home: the arm holds where it is")
        except Exception as error:
            if self._abort.is_set():
                self._end_home("aborted", "aborted going home: the arm holds where it is")
                return
            logger.exception("going home raised")
            self._end_home("failed", f"going home failed: {plain_error(error)}")
        else:
            self._end_home("done", "the arm is at home")

    def _home(self) -> None:
        speed = self.settings.speed_deg_s
        self.wait_for_operator(
            f"The arm will ramp to home at {speed:g} deg/s from wherever it is. Clear the "
            "workspace. Enter to continue"
        )
        with self.open_session() as session:
            session.move_to(self.arm.home)

    def _end_home(self, state: ProcedureStateName, message: str) -> None:
        with self._lock:
            self._current = None
            self._progress = None
            self._set_state(state, message)

    def _run_step(self, step: Step, ctx: StepContext) -> bool:
        """Run one step; whether the procedure goes on. ``AbortRequestedError`` propagates."""
        with self._lock:
            self._current = step.id
            self._progress = None
            self._step_states[step.id]["status"] = "running"
            self._set_state("running", f"{step.title}: {step.instructions}")
        try:
            result = step.run(ctx)
        except SkipRequestedError as skipped:
            self._finish_step(step, "skipped", f"skipped at the prompt: {skipped}")
        except StepUnavailableError as unavailable:
            self._finish_step(step, "skipped", str(unavailable))
        except AbortRequestedError:
            raise
        except StepFailedError as failure:
            # Aborting ends the session under the step, which then fails for it.
            self.check_abort()
            self._fail(step, str(failure))
            return False
        except Exception as error:
            self.check_abort()
            logger.exception(f"calibration step {step.id} raised")
            self._fail(step, f"{type(error).__name__}: {plain_error(error)}")
            return False
        else:
            with self._lock:
                self._results[step.id] = result.data
            self._finish_step(step, "skipped" if result.skipped else "done", result.message)
        return True

    def _fail(self, step: Step, message: str) -> None:
        self._finish_step(step, "failed", message)
        with self._lock:
            self._set_state("failed", f"{step.title} failed: {message}")

    def _finish_step(self, step: Step, status: StepStatusName, message: str) -> None:
        with self._lock:
            self._step_states[step.id]["status"] = status
            self._step_states[step.id]["message"] = message
            self._touch()

    def _set_state(self, state: ProcedureStateName, message: str) -> None:
        self._state = state
        self._message = message
        self._touch()

    def _touch(self) -> None:
        self._updated = self.clock.now()
        if self._events is not None:
            self._events.status(self._status_locked())

    def _status_locked(self) -> ProcedureStatus:
        return ProcedureStatus(
            state=self._state,
            arm_id=self.spec.arm_id,
            run_id=self._run_id,
            step=self._current,
            message=self._message,
            prompt=self._prompt,
            prompt_id=None if self._prompt is None else self._prompts,
            prompt_deadline=None if self._prompt is None else self._prompt_deadline,
            progress=None if self._progress is None else Progress(**self._progress),
            steps=[StepState(**state) for state in self._step_states.values()],
            park=self._park_outcome,
            updated=self._updated,
        )


def _duration(seconds: float) -> str:
    return f"{seconds / 60.0:g} min" if seconds >= 60.0 else f"{seconds:g} s"
