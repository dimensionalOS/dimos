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

"""Left-arm-forward skill on the guarded SONIC controller, through SonicExecutionModule.

Encodes a min-jerk left-shoulder-pitch reference from the measured pose (sonic_arm_reference),
streams it as a sonic78 source and hands control back to the native planner. The whole 11 s plan
is one open-loop chunk planned from the start pose, exactly as commissioned on a G1, so the
execution profile's max_observation_age must cover it. Resubmitting it like a policy source was
rejected: every chunk restarts the session blend, a 2.5 Hz speed ripple. observed_at uses this
host's monotonic clock: run on the same host as the execution module. Never starts or stops native
control. Cancels to the planner if a joint leaves its per-group envelope or exceeds `max_dq`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import threading
import time
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.control.components import make_humanoid_joints
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.robot.unitree.g1.sonic_arm_reference import (
    LEFT_SHOULDER_PITCH,
    RATE,
    SonicEncoder,
    arm_poses,
    group_margins,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.spec.utils import Spec

MAX_ANGLE_DEG = 28.0  # Commissioned at 0.5 rad (28.6 deg) on a G1.
_MOVED_LIMIT = 0.05  # rad: body motion allowed while encoding.
_JOINTS = [name.removeprefix("g1/") for name in make_humanoid_joints("g1")]
_GROUPS = {
    "legs": list(range(12)),
    "waist": [12, 13, 14],
    "arms": [16, 17, 18, 22, 23, 24, 25],
    "wrists": [19, 20, 21, 26, 27, 28],
}


def _peak(peak: tuple[float, int]) -> dict[str, Any]:
    return {"rad_s": round(peak[0], 3), "joint": _JOINTS[peak[1]] if peak[0] > 0 else None}


class SonicArmExecutionSpec(Spec, Protocol):
    def get_status(self) -> dict[str, Any]: ...
    def get_feedback(self) -> SonicFeedback | None: ...
    def start_task(self) -> dict[str, Any]: ...
    def submit_chunk(
        self, task_id: str, sequence: int, observed_at: float, actions: NDArray[Any]
    ) -> dict[str, Any]: ...
    def finish_task(self, task_id: str) -> dict[str, Any]: ...
    def cancel_task(self, task_id: str) -> dict[str, Any]: ...


class SonicArmSkillConfig(ModuleConfig):
    encoder_path: Path
    body_margin: float = Field(default=0.15, gt=0, le=0.3)  # legs and waist, rad
    arm_margin: float = Field(default=0.20, gt=0, le=0.4)  # shoulders and elbows, rad
    wrist_margin: float = Field(default=0.30, gt=0, le=0.5)  # wrists, rad
    max_dq: float = Field(default=3.0, gt=0, le=5.0)  # rad/s, any joint
    return_watch_seconds: float = Field(default=3.0, ge=0, le=5.0)


@dataclass
class _Watch:
    base: NDArray[np.float64]
    allowed: NDArray[np.float64]
    max_dq: float
    returning: bool = False
    reached: float = 0.0
    deviation: NDArray[np.float64] = field(default_factory=lambda: np.zeros(29))
    stream_dq: tuple[float, int] = (0.0, -1)
    return_dq: tuple[float, int] = (0.0, -1)
    abort: str | None = None


class SonicArmSkillContainer(Module):
    """Arm-reference skills over the guarded SONIC controller (no hands required)."""

    config: SonicArmSkillConfig
    feedback: In[SonicFeedback]
    _execution: SonicArmExecutionSpec

    def __init__(self, **kwargs: Any) -> None:
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._watch: _Watch | None = None
        self._encoder: SonicEncoder | None = None
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        self._encoder = SonicEncoder(self.config.encoder_path)
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    async def handle_feedback(self, feedback: SonicFeedback) -> None:
        """Track the active move's envelope, reach and speeds from every telemetry sample."""
        with self._lock:
            watch = self._watch
            if watch is None:
                return
            q = np.asarray(feedback.body_q) - watch.base
            dq = np.abs(np.asarray(feedback.body_dq))
            k = int(dq.argmax())
            if watch.returning:
                watch.return_dq = max(watch.return_dq, (float(dq[k]), k))
                return
            watch.reached = min(watch.reached, float(q[LEFT_SHOULDER_PITCH]))
            watch.deviation = np.maximum(watch.deviation, np.abs(q))
            watch.stream_dq = max(watch.stream_dq, (float(dq[k]), k))
            if watch.abort is None:
                outside = np.nonzero(np.abs(q) > watch.allowed)[0]
                if len(outside):
                    j = int(outside[0])
                    watch.abort = f"{_JOINTS[j]} left its envelope ({q[j]:+.3f} rad)"
                elif dq[k] > watch.max_dq:
                    watch.abort = f"{_JOINTS[k]} speed {dq[k]:.2f} rad/s > {watch.max_dq}"

    @skill(uses=[CAP_MOVEMENT])
    def move_left_arm_forward(self, angle_deg: float) -> SkillResult:
        """Swing the left arm forward, hold it for 1 second, swing it back, then hand control back.

        Moves only the left shoulder (min-jerk, 2 s out and 2 s back) while SONIC keeps the robot
        balanced. Takes about 18 s. Requires the SONIC controller running in confirmed planner idle;
        it never starts or stops the controller. Stops early and returns control to the planner if
        any joint strays from the plan. Raises, having sent nothing, when it cannot start.

        Args:
            angle_deg: How far to swing the arm forward, in degrees (1 to 28).
        """
        if not 1.0 <= angle_deg <= MAX_ANGLE_DEG or not math.isfinite(angle_deg):
            raise ValueError(f"angle_deg must be within 1..{MAX_ANGLE_DEG}")
        if not self._busy.acquire(blocking=False):
            raise RuntimeError("an arm move is already running")
        try:
            return self._move(-math.radians(angle_deg))
        finally:
            with self._lock:
                self._watch = None
            self._busy.release()

    def _move(self, delta: float) -> SkillResult:
        assert self._encoder is not None
        status = self._execution.get_status()
        profile = status["profile"]
        if not status["ready"]:
            raise RuntimeError("SONIC is not in confirmed planner idle; nothing sent")
        state = self._execution.get_feedback()
        if state is None or state.token_state is None:
            raise RuntimeError("no fresh controller feedback; nothing sent")
        base = np.asarray(state.body_q, np.float64)
        started = time.monotonic()
        motion = self._encoder.encode(arm_poses(base, LEFT_SHOULDER_PITCH, delta))
        encode_s = time.monotonic() - started
        current = self._execution.get_feedback()
        if current is None or np.abs(np.asarray(current.body_q) - base).max() > _MOVED_LIMIT:
            raise RuntimeError("the body moved while the reference was encoded; nothing sent")
        tokens = token_schedule(np.asarray(state.token_state), motion)
        hands = np.r_[state.left_hand_q, state.right_hand_q]
        plan = np.hstack([tokens, np.tile(hands, (len(tokens), 1))])
        if profile["max_observation_age"] <= len(plan) / RATE + 0.5:
            raise RuntimeError("the execution profile's max_observation_age must cover the plan")
        allowed = group_margins(
            self.config.body_margin, self.config.arm_margin, self.config.wrist_margin
        )
        allowed[LEFT_SHOULDER_PITCH] += abs(delta)
        watch = _Watch(base=base, allowed=allowed, max_dq=self.config.max_dq)
        with self._lock:
            self._watch = watch

        task_id = self._execution.start_task()["task_id"]  # Raises when refused; nothing sent.
        try:
            outcome = self._stream(task_id, plan, profile, watch)
        except Exception as exc:
            outcome = f"{type(exc).__name__}: {exc}"
            try:
                self._execution.cancel_task(task_id)
            except Exception:
                pass  # The execution module fails the session on its own faults.
        task = self._wait_done(profile)
        with self._lock:
            watch.returning = True
        journal = self._execution.get_status()["journal_path"]
        if task is None or not task["handback_confirmed"]:
            raise RuntimeError(
                f"planner handback unconfirmed ({outcome}); check the controller. Journal: {journal}"
            )
        if outcome == "finished" and task["outcome"] != "finished":
            outcome = f"session {task['outcome']}: {task['reason']}"
        if outcome not in ("finished", "envelope"):
            raise RuntimeError(f"{outcome}; control returned to the planner. Journal: {journal}")
        time.sleep(self.config.return_watch_seconds)

        reached = abs(math.degrees(watch.reached))
        report = {
            "task_id": task_id,
            "reached_rad": round(watch.reached, 3),
            "planned_rad": round(delta, 3),
            "max_deviation_rad": {
                g: round(float(watch.deviation[i].max()), 3) for g, i in _GROUPS.items()
            },
            "stream_peak_dq": _peak(watch.stream_dq),
            "return_peak_dq": _peak(watch.return_dq),
            "encode_s": round(encode_s, 2),
            "journal_path": journal,
        }
        if outcome == "envelope":
            message = f"Stopped early at {reached:.1f} deg: {watch.abort}"
        elif watch.reached > delta / 2:  # SONIC tracked less than half the planned swing.
            message = f"The arm reached only {reached:.1f} of {math.degrees(-delta):.1f} deg"
        else:
            message = f"Left arm swung forward to {reached:.1f} deg and back"
        return SkillResult(f"{message}; control returned to SONIC's planner", metadata=report)

    def _stream(
        self, task_id: str, plan: NDArray[np.float64], profile: dict[str, Any], watch: _Watch
    ) -> str:
        startup = time.monotonic() + profile["seed_seconds"] + profile["arm_timeout"] + 1.0
        while (task := self._task())["phase"] != "waiting_for_chunk":
            if task["phase"] == "done" or time.monotonic() > startup:
                return f"stream did not arm ({task['reason']})"
            time.sleep(0.01)
        plan_start = next_check = time.monotonic()
        task = self._execution.submit_chunk(task_id, 0, plan_start, plan)
        while True:
            with self._lock:
                abort = watch.abort
            if abort is not None:
                self._execution.cancel_task(task_id)
                return "envelope"
            now = time.monotonic()
            if (now - plan_start) * RATE >= len(plan) - 2:
                # The plan already blended back to the planner token.
                self._execution.finish_task(task_id)
                return "finished"  # Checked against the session's own outcome afterwards.
            if now >= next_check:
                task, next_check = self._task(), now + 0.1
            if task["phase"] not in ("waiting_for_chunk", "running"):
                return f"session ended: {task['reason']}"
            time.sleep(0.005)

    def _task(self) -> dict[str, Any]:
        task: dict[str, Any] = self._execution.get_status()["task"]
        return task

    def _wait_done(self, profile: dict[str, Any]) -> dict[str, Any] | None:
        deadline = time.monotonic() + profile["handback_timeout"] + 1.0
        while time.monotonic() < deadline:
            task = self._task()
            if task["phase"] == "done":
                return task
            time.sleep(0.01)
        return None


sonic_arm_skill_container = SonicArmSkillContainer.blueprint
