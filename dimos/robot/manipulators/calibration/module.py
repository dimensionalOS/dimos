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

"""The arm calibration procedure as a module, driven over RPC.

A :class:`CalibrationModule` runs :class:`~dimos.robot.manipulators.
calibration.procedure.Procedure` on one arm of a known Damiao robot
(``robots.KNOWN_ROBOTS``). A terminal wizard or a Desktop tab drives it:
``plan`` lists the steps, ``run`` starts them, ``status`` (also streamed on
``calibration_status``) says when a confirmation waits, and ``confirm``,
``skip`` and ``abort`` answer it. ``dimos_run_id`` names the ``dimos run`` the
module belongs to, so a client that started a run can tell it is talking to
that one before it drives the arm.

The arm is reached through the ``_connection`` module reference: the
blueprint injects the robot's connection (a ``DamiaoConnection`` deployed
with ``tuning=True``) and every call is an RPC to it over the bus. Targets are
streamed as ``tuning_command`` calls at ``ProcedureSettings.stream_hz``, 50
per second by default, each followed by a ``tuning_read``.

Markers go out on ``gravity_events``, ``step_events`` and ``float_events``,
the streams the offline reports (``gravity_fit``, ``step_report``,
``float_report``) read from a recording, and all of them together on
``calibration_events``. The fit and the gain analysis read the run's own
``--record sqlite`` recording, ``recordings/<run-id>/memory.db``
(``recording.SqliteRecording``); the calibration blueprints pin recording on.

Stopping the module parks the arm first (``Procedure.shutdown``): any run is
aborted and the arm backs out home within ``park_budget_s``, then the session
ends and the connection may de-energize it. ``tuning_command`` and
``tuning_read`` time out after ``TUNING_CALL_TIMEOUT_S``: a call that slow has
already let the connection's deadman hold the arm.
"""

from __future__ import annotations

import json
import os
import threading
from typing import Any, Protocol

from pydantic import Field, model_validator
from typing_extensions import Self

from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.std_msgs.String import String
from dimos.robot.manipulators.calibration.arm_session import MarkerStream, TuningConnection
from dimos.robot.manipulators.calibration.gravity_poses import DEFAULT_POSE_COUNT
from dimos.robot.manipulators.calibration.motion import (
    DEFAULT_SPEED_DEG_S,
    MAX_SPEED_DEG_S,
    MAX_STEP_DEG,
)
from dimos.robot.manipulators.calibration.procedure import (
    DEFAULT_DITHER_DEG,
    DEFAULT_HOLD_S,
    DEFAULT_KP_SCALES,
    DEFAULT_PROMPT_TIMEOUT_S,
    MAX_PARK_BUDGET_S,
    MAX_PARK_SPEED_DEG_S,
    Capabilities,
    PlannedStep,
    Procedure,
    ProcedureSettings,
    ProcedureStatus,
)
from dimos.robot.manipulators.calibration.recording import SqliteRecording, probe_payload
from dimos.robot.manipulators.calibration.robots import KNOWN_ROBOTS, arm_spec
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec
from dimos.robot.manipulators.calibration.steps import calibration_steps
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

NOT_RECORDING = "not in a dimos run, so nothing is recorded to read back"
# The streamed calls fail fast: by then the connection's 0.1 s deadman holds the arm.
TUNING_CALL_TIMEOUT_S = 0.5
STREAMED_CALLS = ("tuning_command", "tuning_read")


class ArmTuningConnectionSpec(Spec, TuningConnection, Protocol):
    """The connection a calibration module drives: one deployed with ``tuning=True``."""


class CalibrationModuleConfig(ModuleConfig):
    """Which arm to calibrate and how.

    Attributes:
        robot: A ``robots.KNOWN_ROBOTS`` key, e.g. "dual-openyam".
        arm: The arm ("left"), its bus ("left_arm") or its arm id; needed when
            the robot has several.
        floor_z_m: The table's height in the arm's workspace frame, m, for an
            arm with no workspace guard of its own; with ``min_x_m``.
        min_x_m: The fingertip stays in front of this x, m; with ``floor_z_m``.
        poses: Gravity identification poses.
        dither_deg: Dither of each measured joint at every pose.
        hold_s: Rest at each pose.
        speed_deg_s: Peak joint speed of every ramp.
        float_pose_deg: One pose every joint floats at, one angle per arm joint
            in degrees; when unset, each joint floats at the gravity poses
            where it carries the most gravity torque.
        sweep_pose_deg: Where the gain sweep steps the joints, one angle per
            arm joint in degrees; when unset, the gravity pose farthest from
            every joint limit.
        confirm_each: Ask before every pose.
        kp_scales: Multiples of each joint's robot kp the gain sweep tries.
        park_budget_s: How long parking the arm on stop or a prompt timeout
            may take.
        park_speed_deg_s: Peak joint speed of a park.
        prompt_timeout_s: A prompt unanswered this long aborts and parks.
        auto_run: Start every step as soon as the module starts.
    """

    robot: str
    arm: str | None = None
    floor_z_m: float | None = None
    min_x_m: float | None = None
    poses: int = Field(default=DEFAULT_POSE_COUNT, ge=1)
    dither_deg: float = Field(default=DEFAULT_DITHER_DEG, gt=0.0, le=MAX_STEP_DEG)
    hold_s: float = Field(default=DEFAULT_HOLD_S, gt=0.0)
    speed_deg_s: float = Field(default=DEFAULT_SPEED_DEG_S, gt=0.0, le=MAX_SPEED_DEG_S)
    float_pose_deg: list[float] | None = None
    sweep_pose_deg: list[float] | None = None
    confirm_each: bool = True
    kp_scales: list[float] = Field(default_factory=lambda: list(DEFAULT_KP_SCALES), min_length=1)
    park_budget_s: float = Field(default=MAX_PARK_BUDGET_S, gt=0.0, le=MAX_PARK_BUDGET_S)
    park_speed_deg_s: float = Field(default=MAX_PARK_SPEED_DEG_S, gt=0.0, le=MAX_PARK_SPEED_DEG_S)
    prompt_timeout_s: float = Field(default=DEFAULT_PROMPT_TIMEOUT_S, gt=0.0)
    auto_run: bool = False

    @model_validator(mode="after")
    def _known_arm(self) -> Self:
        robot = KNOWN_ROBOTS.get(self.robot)
        if robot is None:
            raise ValueError(f"unknown robot {self.robot!r}; known: {', '.join(KNOWN_ROBOTS)}")
        if self.arm is None and len(robot.buses) > 1:
            arms = [bus.name.removesuffix("_arm") for bus in robot.buses]
            raise ValueError(f"{self.robot} has several arms; set arm to one of {arms}")
        if (self.floor_z_m is None) != (self.min_x_m is None):
            raise ValueError("give both floor_z_m and min_x_m, or neither")
        return self

    def arm_spec(self) -> ArmCalibrationSpec:
        """The arm's spec, on a bench guard when ``floor_z_m`` and ``min_x_m`` are given."""
        spec = arm_spec(self.robot, self.arm)
        if self.floor_z_m is not None and self.min_x_m is not None:
            spec = spec.on_bench(self.floor_z_m, self.min_x_m)
        return spec

    def settings(self) -> ProcedureSettings:
        return ProcedureSettings(
            poses=self.poses,
            dither_deg=self.dither_deg,
            hold_s=self.hold_s,
            speed_deg_s=self.speed_deg_s,
            confirm_each=self.confirm_each,
            float_pose_deg=None if self.float_pose_deg is None else tuple(self.float_pose_deg),
            sweep_pose_deg=None if self.sweep_pose_deg is None else tuple(self.sweep_pose_deg),
            kp_scales=tuple(self.kp_scales),
            park_budget_s=self.park_budget_s,
            park_speed_deg_s=self.park_speed_deg_s,
            prompt_timeout_s=self.prompt_timeout_s,
        )


class _StreamSink:
    """Publishes the procedure's markers and status on the module's streams."""

    def __init__(self, module: CalibrationModule) -> None:
        self._module = module

    def marker(self, stream: MarkerStream, payload: dict[str, Any]) -> None:
        text = json.dumps(payload)
        outputs = {
            "gravity_events": self._module.gravity_events,
            "step_events": self._module.step_events,
            "float_events": self._module.float_events,
        }
        outputs[stream].publish(String(text))
        event = {"stream": stream, "marker": payload}
        self._module.calibration_events.publish(String(json.dumps(event)))

    def status(self, status: ProcedureStatus) -> None:
        self._module.calibration_status.publish(String(json.dumps(status)))

    def probe(self, token: str) -> None:
        self._module.calibration_events.publish(String(json.dumps(probe_payload(token))))


class CalibrationModule(Module):
    """Calibrates one arm of a Damiao robot, step by step, as its RPCs direct.

    Every RPC takes and returns plain JSON values; see ``procedure`` for the
    status and plan shapes.
    """

    config: CalibrationModuleConfig
    _connection: ArmTuningConnectionSpec
    gravity_events: Out[String]
    step_events: Out[String]
    float_events: Out[String]
    calibration_events: Out[String]
    calibration_status: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._procedure: Procedure | None = None
        self._recording: SqliteRecording | None = None

    @rpc
    def start(self) -> None:
        super().start()
        spec = self.config.arm_spec()
        self._shorten_streamed_calls()
        self._recording = SqliteRecording.of_this_run()
        procedure = Procedure(
            spec,
            self._connection,
            calibration_steps(),
            settings=self.config.settings(),
            caps=self._capabilities(),
            recording=self._recording,
            events=_StreamSink(self),
        )
        with self._lock:
            self._procedure = procedure
        if self.config.auto_run:
            procedure.start()

    @rpc
    def stop(self) -> None:
        """Park the arm (abort any run, back out home on the park budget), then stop."""
        with self._lock:
            procedure, self._procedure = self._procedure, None
        if procedure is not None:
            outcome = procedure.shutdown()
            logger.info(f"{procedure.spec.arm_id}: calibration module stopping: {outcome}")
        super().stop()

    def _shorten_streamed_calls(self) -> None:
        """Give the calls streamed every tick a ``TUNING_CALL_TIMEOUT_S`` timeout.

        The connection reference's RPC client resolves a call's timeout from
        its ``rpc_timeouts`` by method name, falling back to the module default
        (two minutes); a fake connection has no client and is left alone.
        """
        client = getattr(self._connection, "rpc", None)
        timeouts = getattr(client, "rpc_timeouts", None)
        if isinstance(timeouts, dict):
            for name in STREAMED_CALLS:
                timeouts[name] = TUNING_CALL_TIMEOUT_S

    def _capabilities(self) -> Capabilities:
        robot = KNOWN_ROBOTS[self.config.robot]
        recording = self._recording
        return Capabilities(
            torque_readback=True,
            gravity_compensation=robot.gravity,
            set_gains=True,
            recording_unavailable=NOT_RECORDING if recording is None else recording.unavailable(),
        )

    def _require(self) -> Procedure:
        with self._lock:
            procedure = self._procedure
        if procedure is None:
            raise RuntimeError("the calibration module has not started")
        return procedure

    def _refresh(self, procedure: Procedure) -> None:
        """Recheck the recording, which starts after the modules do; kept while a run is on."""
        procedure.set_capabilities(self._capabilities())

    @rpc
    def plan(self) -> list[PlannedStep]:
        """Every step, its instructions, and whether it can run on this arm and why not."""
        procedure = self._require()
        self._refresh(procedure)
        return procedure.plan()

    @rpc
    def run(self, steps: list[str] | None = None) -> ProcedureStatus:
        """Start the steps (every step, or those named) in order; refused while a run is on."""
        procedure = self._require()
        self._refresh(procedure)
        return procedure.start(steps)

    @rpc
    def go_home(self) -> ProcedureStatus:
        """Ramp to home after a confirmation; only while idle, done, failed or aborted."""
        procedure = self._require()
        self._refresh(procedure)
        return procedure.go_home()

    @rpc
    def confirm(self, prompt_id: int) -> bool:
        """Go on at confirmation ``prompt_id`` (``status()["prompt_id"]``); False unless it waits.

        Raises ``TypeError`` without a prompt id.
        """
        return self._require().confirm(prompt_id)

    @rpc
    def skip(self, prompt_id: int) -> bool:
        """Skip what confirmation ``prompt_id`` asks for; False unless it waits."""
        return self._require().skip(prompt_id)

    @rpc
    def abort(self) -> bool:
        """Stop now and hold the arm where it is, without moving it home; False when idle."""
        return self._require().abort()

    @rpc
    def status(self) -> ProcedureStatus:
        """Where the procedure is, what it asks, and every step's status."""
        return self._require().status()

    @rpc
    def results(self) -> dict[str, Any]:
        """The data of every step done in the current or last run, by step id."""
        return self._require().results()

    @rpc
    def dimos_run_id(self) -> str | None:
        """The ``dimos run`` this module runs in (its ``DIMOS_RUN_ID``), ``None`` outside one."""
        return os.environ.get(DIMOS_RUN_ID_ENV)
