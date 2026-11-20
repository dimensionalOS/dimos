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

"""Agent tools for an operator-requested, unattended eye-in-hand calibration."""

from __future__ import annotations

from concurrent.futures import CancelledError
import json
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Any
from uuid import uuid4

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.calibration import (
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_IMAGE_TOPIC,
    DEFAULT_MARKER_SIZE_M,
    DEFAULT_TF_TOPIC,
    MIN_TRAINING_SAMPLES,
    MIN_VALIDATION_SAMPLES,
)
from dimos.calibration.hand_eye_run import check_orbit_axis, check_view_counts, run_live_hand_eye
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
_START_TOOL = "start_hand_eye_calibration"


class HandEyeCalibrationConfig(ModuleConfig):
    session_root: Path = Path("hand-eye-sessions")
    image_topic: str = DEFAULT_IMAGE_TOPIC
    camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC
    tf_topic: str = DEFAULT_TF_TOPIC
    marker_size_m: float = DEFAULT_MARKER_SIZE_M
    marker_id: int = 0


def _saved_grade(path: Path) -> str:
    result = json.loads(path.read_text())
    grade = "PASS" if result["validated"] else "FAIL"
    if result["training_metrics"]["passed"] and "validation_metrics" not in result:
        grade = "Candidate awaiting validation"
    parts = [f"{grade}  {path}"]
    for role in ("training", "validation"):
        if metrics := result.get(f"{role}_metrics"):
            parts.append(
                f"{role} ({metrics['sample_count']} views): "
                f"{1000 * metrics['max_translation_error_m']:.3f} mm, "
                f"{metrics['max_rotation_error_deg']:.3f} deg, "
                f"{metrics['max_reprojection_error_px']:.3f} px max"
            )
    return ". ".join(parts)


class HandEyeCalibrationSkills(Module):
    """Run one calibration at a time using the running planner and configured camera topics."""

    config: HandEyeCalibrationConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = Lock()
        self._cancel = Event()
        self._worker: Thread | None = None
        self._closing = False
        self._status = "No calibration has been started."

    @skill(uses=[CAP_MOVEMENT], lifecycle="background", allow_reentry=False)
    def start_hand_eye_calibration(
        self,
        home: list[float],
        ee_frame: str,
        count: int = MIN_TRAINING_SAMPLES,
        validation_count: int = MIN_VALIDATION_SAMPLES,
        orbit_axis: str | None = None,
        orbit_calibration: str | None = None,
    ) -> str:
        """Move the arm and calibrate once the operator has placed the fixed target and asked to start.

        An angled first sighting and four training tilts aim the camera for the two sweeps.
        After the first sighting, a rejected observation gets one retry halfway toward home.
        A completion update reports the grade or failure reason and session path.

        Args:
            home: Operator-supplied joint angles that center the tag, in planner order and radians.
            ee_frame: TF name of the link carrying the camera, supplied by the operator.
            count: Number of training views, at least 8.
            validation_count: Number of separate validation views, at least 4.
            orbit_axis: For smaller rings, supply the planner TCP axis closest to camera forward
                at home (x, -x, y, -y, z, -z). The example A1Z/D405 mount uses x.
            orbit_calibration: To skip the four aiming tilts, supply the path on the robot stack
                to a validated hand_eye.json from this camera mount. Choose at most one orbit option.
        """
        with self._lock:
            if self._closing:
                raise RuntimeError("Calibration module is shutting down.")
            if self._worker is not None:
                raise RuntimeError(f"Calibration already in progress. {self._status}")
            self.start_tool(_START_TOOL)
            try:
                check_view_counts(count, validation_count)
                calibration_path = (
                    Path(orbit_calibration) if orbit_calibration is not None else None
                )
                check_orbit_axis(orbit_axis, calibration_path)
                if not home:
                    raise ValueError("Pass home as a nonempty list of joint angles in radians.")
                if not ee_frame or not ee_frame.strip():
                    raise ValueError("Pass the TF name of the camera mounting link as ee_frame.")
                root = self.config.session_root.expanduser().resolve()
                session = root / f"hand-eye-{uuid4().hex}"
                session.mkdir(parents=True)
                self._cancel.clear()
                worker = Thread(
                    target=self._run,
                    args=(
                        session,
                        count,
                        validation_count,
                        list(home),
                        ee_frame,
                        orbit_axis,
                        calibration_path,
                    ),
                    name="hand-eye",
                    daemon=True,
                )
                worker.start()
            except Exception as exc:
                self._status = f"Could not start calibration. {exc}"
                self.stop_tool(_START_TOOL)
                raise
            self._worker = worker
            self._status = f"Starting calibration in {session}"
            return self._status

    def _check_cancelled(self) -> None:
        if self._cancel.is_set():
            raise CancelledError

    def _run(
        self,
        session: Path,
        count: int,
        validation_count: int,
        home: list[float],
        ee_frame: str,
        orbit_axis: str | None,
        orbit_calibration: Path | None,
    ) -> None:
        def progress(message: str) -> None:
            with self._lock:
                self._status = f"{message} ({session})"

        problem = ""
        try:
            run_live_hand_eye(
                session,
                home=home,
                ee_frame=ee_frame,
                orbit_axis=orbit_axis,
                orbit_calibration=orbit_calibration,
                count=count,
                validation_count=validation_count,
                image_topic=self.config.image_topic,
                camera_info_topic=self.config.camera_info_topic,
                tf_topic=self.config.tf_topic,
                marker_size_m=self.config.marker_size_m,
                marker_id=self.config.marker_id,
                report=progress,
                check_cancelled=self._check_cancelled,
            )
        except CancelledError:
            problem = f"STOPPED calibration in {session}."
        except Exception as exc:
            logger.exception("Hand-eye calibration failed", session=str(session))
            problem = f"Run failed in {session}. {exc}"

        # A failed return move can still leave a valid, graded calibration on disk.
        path = session / "hand_eye.json"
        try:
            grade = _saved_grade(path) if path.is_file() or not problem else ""
        except (OSError, ValueError, KeyError, TypeError) as exc:
            grade = f"Could not read the saved grade from {path}. {exc}"
        with self._lock:
            self._status = " ".join(filter(None, (problem, grade)))
            try:
                self.tool_update(_START_TOOL, self._status)
            finally:
                self.stop_tool(_START_TOOL)
                self._worker = None

    @skill
    def hand_eye_calibration_status(self) -> str:
        """Return the current view or the last run's outcome and session path."""
        with self._lock:
            if self._worker is not None and self._cancel.is_set():
                return f"Stop requested. {self._status}"
            return self._status

    @skill
    def stop_hand_eye_calibration(self) -> str:
        """Stop calibration after the current operation finishes, before the next move."""
        with self._lock:
            if self._worker is None:
                return self._status
            self._cancel.set()
            return "Stop requested. Any move already in progress can finish."

    @rpc
    def stop(self) -> None:
        with self._lock:
            self._closing = True
            self._cancel.set()
            worker = self._worker
        if worker is not None:
            worker.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if worker.is_alive():
                # Module.stop closes all tool streams. Keep the movement hold until the run exits.
                raise RuntimeError(
                    "Calibration is still stopping. Retry unloading the module after the "
                    "current operation finishes."
                )
        super().stop()
