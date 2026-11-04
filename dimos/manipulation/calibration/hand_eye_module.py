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

"""Hand-eye calibration of a camera against a ChArUco board, driving the arm through the planner.

The module takes the camera's ``color_image`` and ``camera_info``, the ``tf`` tree the planner
publishes, and moves the arm through ``ManipulationSpec``: every target is planned with
collision checking and executed blocking. ``calibrate()`` runs the whole thing and writes the
result under ``STATE_DIR/calibration``; ``approve()`` lets the operator release the first moves
one by one; ``abort()`` cancels the running trajectory and ends the run.

Policy limits that are not the planner's job live in the config: a cap on how far any joint may
move in one target, a cap on moves per run, the planner speed scale, and a keep-out sphere
around a fixed camera, registered with the planner as an obstacle for the run.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Literal, Protocol

import cv2
import numpy as np
from pydantic import BaseModel, Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.calibration import store
from dimos.manipulation.calibration.hand_eye import (
    BoardSpec,
    CameraNominal,
    FloatArray,
    Frame,
    HandEyeResult,
    solve_hand_eye,
)
from dimos.manipulation.calibration.store import save_hand_eye
from dimos.manipulation.manipulation_spec import ExecutionStatus, ManipulationSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The arm has settled when no joint moved more than SETTLE_TOL_RAD over SETTLE_WINDOW_S. What the
# solve needs is a still arm at the moment of the frame, not an arm exactly on its target.
SETTLE_TOL_RAD = 0.002
SETTLE_WINDOW_S = 0.3
SETTLE_POLL_S = 0.05
# A frame counts as taken after the move when its stamp is newer than the move's end.
FRAME_TIMEOUT_S = 5.0
# The flange pose is read at the frame's stamp; how far the tf buffer may interpolate for it.
TF_TOLERANCE_S = 0.2
EXECUTE_TIMEOUT_S = 60.0
KEEPOUT_OBSTACLE = "hand_eye_camera_keepout"
TABLE_OBSTACLE = "hand_eye_table"
TABLE_THICKNESS_M = 0.2


class ObstacleSpec(Spec, Protocol):
    """The planner's obstacle RPCs, as ManipulationModule exposes them."""

    def add_obstacle(
        self,
        name: str,
        pose: Pose,
        shape: str,
        dimensions: list[float] | None = None,
        mesh_path: str | None = None,
    ) -> str: ...

    def remove_obstacle(self, obstacle_id: str) -> bool: ...


class HandEyeBoardConfig(BaseModel):
    """The printed ChArUco board. ``square_m`` is the measured pitch of the print."""

    squares_x: int = 8
    squares_y: int = 5
    square_m: float = Field(default=0.030, gt=0.0)
    marker_m: float = Field(default=0.022, gt=0.0)
    ids_from: int = 0
    dictionary: str = "DICT_APRILTAG_36h11"


class HandEyeCalibrationConfig(ModuleConfig):
    # The result file's name under STATE_DIR/calibration, e.g. "xarm6_wrist_camera".
    name: str
    # "flange": the camera rides the arm and the board is fixed. "base": the camera is fixed and
    # the board is held by the arm.
    camera_frame: Literal["flange", "base"] = "flange"
    # Where the camera's colour optical frame is believed to be in that frame, and how far the
    # solve may land from it before the verdict refuses it as a gross blunder.
    nominal_translation: tuple[float, float, float]
    nominal_rotation_xyzw: tuple[float, float, float, float]
    nominal_tolerance_m: float = Field(default=0.01, gt=0.0)
    nominal_tolerance_deg: float = Field(default=5.0, gt=0.0)
    board: HandEyeBoardConfig = Field(default_factory=HandEyeBoardConfig)
    # A pose with the board in the middle of the frame about PROBE_M away; the run draws poses
    # about it.
    home_joints: list[float]
    joint_limits_rad: list[tuple[float, float]]
    planning_group: str = "manipulator"
    base_frame: str = "world"
    flange_frame: str = "link6"
    # The frame the camera module stamps its colour frames with, and the frame a blueprint mounts
    # the camera by; both come from the camera's own tf.
    optical_frame: str = "camera_color_optical_frame"
    camera_link_frame: str = "camera_link"
    max_poses: int = Field(default=20, ge=3)
    max_seconds: float = Field(default=300.0, gt=0.0)
    speed_scale: float = Field(default=0.25, gt=0.0, le=1.0)
    # How long to wait for the arm to come to rest after the planner reports the move complete.
    settle_timeout_s: float = Field(default=5.0, gt=0.0)
    # Policy limits in front of the planner.
    max_joint_step_rad: float = Field(default=1.2, gt=0.0)
    max_moves: int = Field(default=120, gt=0)
    # The operator releases each of the first N moves with approve(); 0 runs unattended.
    approve_first_moves: int = Field(default=3, ge=0)
    # Plan every target but execute none; reports how many the planner accepted.
    dry_run: bool = False
    # For a fixed camera: no part of the arm plans within this of the nominal camera position.
    camera_keepout_m: float = Field(default=0.15, ge=0.0)
    # The bench the board lies on, as a box the planner keeps the arm above for the run: its
    # top in the base frame (None adds nothing) and its extent along x and y.
    table_top_z: float | None = None
    table_x_m: tuple[float, float] = (0.25, 1.0)
    table_y_m: tuple[float, float] = (-0.6, 0.6)
    # Keep every captured frame with the joints and flange pose it was taken at, plus the
    # report, under STATE_DIR/calibration/records/<name>_<time>, for inspection afterwards.
    record: bool = False
    # The pose draw's seed; None draws a fresh sequence each run, so repeat runs are independent.
    seed: int | None = None


@dataclass
class _Run:
    """State of one calibrate() call, created fresh per run."""

    t0: float
    moves: int = 0
    planned: int = 0
    refused: int = 0
    last_move_end: float = 0.0
    waiting_for_approval: bool = False
    last_frame_ts: float | None = None
    # One record per target the solver asked for: its joints, what became of it, and the planned
    # path, so a dry run can be inspected before anything moves.
    targets: list[dict[str, Any]] = field(default_factory=list)
    frames: int = 0
    record_dir: Path | None = None
    seed: int = 0


class HandEyeCalibrationModule(Module):
    """Calibrate one camera against the arm. See the module docstring."""

    config: HandEyeCalibrationConfig

    color_image: In[Image]
    camera_info: In[CameraInfo]
    tf: In[TFMessage]

    _manipulation: ManipulationSpec
    _obstacles: ObstacleSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._frames = threading.Condition()
        self._latest_image: Image | None = None
        self._latest_info: CameraInfo | None = None
        self._run_lock = threading.Lock()
        self._run: _Run | None = None
        self._abort = threading.Event()
        self._approve = threading.Event()
        self._result: HandEyeResult | None = None
        self._targets: list[dict[str, Any]] = []
        self._seed: int | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))
        self.register_disposable(Disposable(self.camera_info.subscribe(self._on_info)))
        # the buffer subscribes on first touch; touch it now so the flange is known by the first move
        _ = self.tfbuffer

    def _on_image(self, msg: Image) -> None:
        with self._frames:
            self._latest_image = msg
            self._frames.notify_all()

    def _on_info(self, msg: CameraInfo) -> None:
        with self._frames:
            self._latest_info = msg

    # ---- operator RPCs -------------------------------------------------------------------------

    @rpc
    def start_calibration(self) -> dict[str, Any]:
        """Start a calibration run in the background and return at once.

        ``status()`` reports progress and, when the run is over, carries the report under
        ``last_result``. For an operator-approved run this is the call to use: the run waits
        on ``approve()`` longer than an RPC client waits for ``calibrate()``.
        """
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("a calibration run is already in progress")
        worker = threading.Thread(
            target=self._run_calibration, args=(True,), name="HandEyeCalibration", daemon=True
        )
        worker.start()
        return {"started": True}

    @rpc
    def calibrate(self) -> dict[str, Any]:
        """Run the calibration and write the result. Blocks until the verdict.

        Returns the verdict, the confidence numbers, the solved mount transform and the file
        written. Refuses to start while a run is in progress.
        """
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("a calibration run is already in progress")
        return self._run_calibration(True)

    def _run_calibration(self, lock_held: bool) -> dict[str, Any]:
        """The run itself; the caller holds the run lock, released here."""
        assert lock_held
        try:
            self._abort.clear()
            self._approve.clear()
            self._run = _Run(t0=time.monotonic())
            self._run.seed = (
                self.config.seed if self.config.seed is not None else int(time.time_ns() % (2**31))
            )
            if self.config.record:
                stamp = time.strftime("%Y%m%d_%H%M%S")
                self._run.record_dir = (
                    store.HAND_EYE_DIR / "records" / f"{self.config.name}_{stamp}"
                )
                self._run.record_dir.mkdir(parents=True, exist_ok=True)
                logger.info("Hand-eye recording", path=str(self._run.record_dir))
            obstacles = self._add_obstacles()
            try:
                result = solve_hand_eye(
                    _Robot(self),
                    _Camera(self),
                    BoardSpec(**self.config.board.model_dump()),
                    self._nominal(),
                    list(self.config.home_joints),
                    self.config.max_poses,
                    self.config.max_seconds,
                    seed=self._run.seed,
                )
            finally:
                for obstacle in obstacles:
                    self._obstacles.remove_obstacle(obstacle)
            if self._abort.is_set():
                result.passed = False
                result.reason = "aborted"
            self._result = result
            self._targets = self._run.targets
            self._seed = self._run.seed
            report = self._report(result)
            if self._run.record_dir is not None:
                report["record_dir"] = str(self._run.record_dir)
                (self._run.record_dir / "report.json").write_text(json.dumps(report, indent=1))
            return report
        finally:
            run, self._run = self._run, None
            self._run_lock.release()
            if run is not None and self.config.dry_run:
                logger.info(
                    "Hand-eye dry run", planned=run.planned, refused=run.refused, moves_sent=0
                )

    @rpc
    def approve(self) -> None:
        """Release the move the run is waiting on (the first ``approve_first_moves`` of a run)."""
        self._approve.set()

    @rpc
    def abort(self) -> None:
        """End the run: cancel what the planner is executing and refuse every further move."""
        self._abort.set()
        self._approve.set()
        self._manipulation.cancel()

    @rpc
    def status(self) -> dict[str, Any]:
        run = self._run
        return {
            "running": run is not None,
            "moves": 0 if run is None else run.moves,
            "elapsed_s": 0.0 if run is None else time.monotonic() - run.t0,
            "waiting_for_approval": run is not None and run.waiting_for_approval,
            "last_result": None if self._result is None else self._report(self._result),
        }

    # ---- the run -------------------------------------------------------------------------------

    def _nominal(self) -> CameraNominal:
        T = np.eye(4)
        T[:3, :3] = Quaternion(*self.config.nominal_rotation_xyzw).to_rotation_matrix()
        T[:3, 3] = self.config.nominal_translation
        return CameraNominal(
            self.config.camera_frame,
            T,
            self.config.nominal_tolerance_m * 1000.0,
            self.config.nominal_tolerance_deg,
        )

    def _add_obstacles(self) -> list[str]:
        """The bench and, for a fixed camera, the camera itself, as planner obstacles for this
        run: a box under the table top and a sphere the arm keeps out of."""
        added: list[str] = []
        cfg = self.config
        if cfg.table_top_z is not None:
            (x0, x1), (y0, y1) = cfg.table_x_m, cfg.table_y_m
            centre = Vector3((x0 + x1) / 2, (y0 + y1) / 2, cfg.table_top_z - TABLE_THICKNESS_M / 2)
            added.append(
                self._obstacles.add_obstacle(
                    TABLE_OBSTACLE,
                    Pose(centre, Quaternion()),
                    "box",
                    [x1 - x0, y1 - y0, TABLE_THICKNESS_M],
                )
            )
        if cfg.camera_frame == "base" and cfg.camera_keepout_m > 0.0:
            pose = Pose(Vector3(*cfg.nominal_translation), Quaternion())
            added.append(
                self._obstacles.add_obstacle(
                    KEEPOUT_OBSTACLE, pose, "sphere", [cfg.camera_keepout_m]
                )
            )
        return added

    def read_joints(self) -> FloatArray:
        state = self._manipulation.get_state().groups.get(self.config.planning_group)
        if state is None or state.joints is None:
            raise RuntimeError(f"no joint state for planning group {self.config.planning_group!r}")
        return np.asarray(state.joints.position, dtype=float)

    def read_flange_pose(self) -> FloatArray:
        """Base -> flange at the last captured frame's stamp, so the pose matches the pixels."""
        run = self._run
        at = None if run is None else run.last_frame_ts
        t = self.tfbuffer.get(
            self.config.base_frame,
            self.config.flange_frame,
            time_point=at,
            time_tolerance=TF_TOLERANCE_S,
        )
        if t is None:
            raise RuntimeError(
                f"no transform {self.config.base_frame} -> {self.config.flange_frame} on tf "
                f"(frames seen: {sorted(self.tfbuffer.get_frames())}); "
                "the planner publishes the flange when its model lists it in tf_extra_links"
            )
        matrix: FloatArray = np.asarray(t.to_matrix(), dtype=float)
        return matrix

    def move_to(self, q: list[float]) -> bool:
        """One target through the planner. False means the arm did not move."""
        run = self._run
        assert run is not None, "move outside a run"
        if self._abort.is_set():
            return False
        if run.moves >= self.config.max_moves:
            logger.warning("Hand-eye move cap reached", max_moves=self.config.max_moves)
            return False
        now = self.read_joints()
        step = float(np.max(np.abs(np.asarray(q) - now)))
        record: dict[str, Any] = {"joints": [round(v, 4) for v in q], "step_rad": round(step, 4)}
        run.targets.append(record)
        if not all(math.isfinite(v) for v in q) or step > self.config.max_joint_step_rad:
            run.refused += 1
            record["outcome"] = "refused_step"
            logger.warning("Hand-eye target refused", step_rad=round(step, 3), target=q)
            return False
        names = self._manipulation.list_planning_groups()
        group = next(g for g in names if g.id == self.config.planning_group)
        target = JointState(name=list(group.joint_names), position=list(q))
        plan = self._manipulation.plan_to_joints(
            {self.config.planning_group: target}, speed_scale=self.config.speed_scale
        )
        if not plan.succeeded or plan.plan is None:
            run.refused += 1
            record["outcome"] = "refused_plan"
            record["message"] = plan.message
            logger.warning("Planner refused the target", message=plan.message)
            return False
        run.planned += 1
        record["outcome"] = "planned"
        record["path"] = [[round(v, 4) for v in point.position] for point in plan.plan.path]
        record["duration_s"] = round(plan.plan.trajectory.duration, 3)
        if self.config.dry_run:
            return False
        if run.moves < self.config.approve_first_moves:
            # approve() releases the next move, whether it is called before or during this wait
            logger.info(
                "Waiting for approve()", move=run.moves + 1, target=[round(v, 3) for v in q]
            )
            run.waiting_for_approval = True
            remaining = self.config.max_seconds - (time.monotonic() - run.t0)
            released = self._approve.wait(timeout=max(remaining, 0.0))
            run.waiting_for_approval = False
            self._approve.clear()
            if not released or self._abort.is_set():
                record["outcome"] = "not_approved"
                return False
        run.moves += 1
        result = self._manipulation.execute(
            blocking=True, timeout=EXECUTE_TIMEOUT_S, plan_id=plan.plan.plan_id
        )
        if result.status is not ExecutionStatus.COMPLETED:
            record["outcome"] = "execution_failed"
            logger.warning(
                "Execution did not complete", status=result.status, message=result.message
            )
            return False
        self._settle(np.asarray(q, dtype=float))
        run.last_move_end = time.time()
        record["outcome"] = "executed"
        return True

    def _settle(self, target: FloatArray) -> None:
        deadline = time.monotonic() + self.config.settle_timeout_s
        window: list[tuple[float, FloatArray]] = []
        moved = float("inf")
        while time.monotonic() < deadline:
            now = time.monotonic()
            window.append((now, self.read_joints()))
            window = [(t, q) for t, q in window if now - t <= SETTLE_WINDOW_S]
            if now - window[0][0] >= SETTLE_WINDOW_S * 0.8:
                moved = float(np.max(np.abs(window[-1][1] - window[0][1])))
                if moved < SETTLE_TOL_RAD:
                    return
            time.sleep(SETTLE_POLL_S)
        logger.warning(
            "Arm still moving at the settle timeout",
            moved_rad=round(moved, 4),
            off_target_rad=round(float(np.max(np.abs(window[-1][1] - target))), 4),
        )

    def capture_frame(self) -> Frame:
        """The first frame stamped after the last move ended, with the camera's intrinsics."""
        run = self._run
        assert run is not None, "capture outside a run"
        deadline = time.monotonic() + FRAME_TIMEOUT_S
        with self._frames:
            while self._latest_image is None or self._latest_image.ts <= run.last_move_end:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not self._frames.wait(timeout=remaining):
                    raise RuntimeError("no fresh colour frame after the move")
            image, info = self._latest_image, self._latest_info
        if info is None:
            raise RuntimeError("no camera_info received")
        run.last_frame_ts = image.ts
        rgb = image.data if image.format is ImageFormat.RGB else _to_rgb(image)
        K = np.asarray(info.K, dtype=float).reshape(3, 3)
        dist = np.resize(np.asarray(info.D, dtype=float).ravel(), 5)
        if run.record_dir is not None:
            self._record_frame(run, rgb, K, dist, image.ts)
        return Frame(rgb, K, dist, image.ts)

    def _record_frame(
        self, run: _Run, rgb: Any, K: FloatArray, dist: FloatArray, ts: float
    ) -> None:
        assert run.record_dir is not None
        run.frames += 1
        stem = run.record_dir / f"frame_{run.frames:02d}"
        cv2.imwrite(str(stem.with_suffix(".png")), np.ascontiguousarray(rgb[:, :, ::-1]))
        try:
            flange = self.read_flange_pose().tolist()
        except RuntimeError as error:
            flange = str(error)
        stem.with_suffix(".json").write_text(
            json.dumps(
                {
                    "ts": ts,
                    "move": run.moves,
                    "joints": self.read_joints().tolist(),
                    "flange_T": flange,
                    "K": K.tolist(),
                    "dist": dist.tolist(),
                },
                indent=1,
            )
        )

    def _report(self, result: HandEyeResult) -> dict[str, Any]:
        parent = (
            self.config.flange_frame
            if self.config.camera_frame == "flange"
            else self.config.base_frame
        )
        optical = Transform.from_matrix(
            result.T, frame_id=parent, child_frame_id=self.config.optical_frame
        )
        mount = self._mount_transform(result.T, parent)
        path = save_hand_eye(self.config.name, mount, optical, result.confidence, result.passed)
        return {
            "passed": result.passed,
            "reason": result.reason,
            "poses_tried": result.poses_tried,
            "poses_moved": result.poses_moved,
            "confidence": result.confidence,
            "optical": _transform_dict(optical),
            "mount": _transform_dict(mount),
            "file": str(path),
            "targets": self._targets,
            "seed": self._seed,
        }

    def _mount_transform(self, T_parent_optical: FloatArray, parent: str) -> Transform:
        """parent -> camera_link, through the camera's own camera_link -> optical edge."""
        link_to_optical = self.tfbuffer.get(
            self.config.camera_link_frame, self.config.optical_frame
        )
        if link_to_optical is None:
            raise RuntimeError(
                f"no transform {self.config.camera_link_frame} -> {self.config.optical_frame} "
                "on tf; the camera module publishes it"
            )
        T = T_parent_optical @ np.linalg.inv(np.asarray(link_to_optical.to_matrix(), dtype=float))
        return Transform.from_matrix(
            T, frame_id=parent, child_frame_id=self.config.camera_link_frame
        )


class _Robot:
    """The solver's view of the arm, over the module's planner connection."""

    def __init__(self, module: HandEyeCalibrationModule) -> None:
        self._m = module

    def joint_limits(self) -> list[tuple[float, float]]:
        return [(float(lo), float(hi)) for lo, hi in self._m.config.joint_limits_rad]

    def get_joints(self) -> list[float]:
        return [float(v) for v in self._m.read_joints()]

    def get_flange_pose(self) -> FloatArray:
        return self._m.read_flange_pose()

    def move_to_joints(self, q: list[float]) -> bool:
        return self._m.move_to(q)

    def clock(self) -> float:
        run = self._m._run
        return 0.0 if run is None else time.monotonic() - run.t0


class _Camera:
    def __init__(self, module: HandEyeCalibrationModule) -> None:
        self._m = module

    def capture(self) -> Frame:
        return self._m.capture_frame()


def _to_rgb(image: Image) -> Any:
    data = image.data
    if image.format is ImageFormat.BGR:
        return data[:, :, ::-1].copy()
    if image.format is ImageFormat.RGBA:
        return data[:, :, :3].copy()
    if image.format is ImageFormat.BGRA:
        return data[:, :, 2::-1].copy()
    raise ValueError(f"unsupported image format {image.format}")


def _transform_dict(t: Transform) -> dict[str, Any]:
    return {
        "frame_id": t.frame_id,
        "child_frame_id": t.child_frame_id,
        "translation_m": [t.translation.x, t.translation.y, t.translation.z],
        "rotation_xyzw": [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
    }
