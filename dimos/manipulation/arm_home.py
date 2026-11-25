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

"""Home poses for arms: saved per arm, reached by a planned move or a slow direct one.

A planned move goes through the planner, so it avoids collisions. A manual
move skips the planner: it is a joint-space path at a fraction of each joint's
speed limit, for when planning fails or the planner is not trusted. It checks
nothing, so the operator must watch the arm.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from itertools import pairwise
import json
import math
from pathlib import Path
import threading
from typing import Any, Literal

from pydantic import BaseModel, Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import STATE_DIR
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.manipulation_spec import ExecutionStatus, ManipulationSpec
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.teleop.webxr.controller_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

HOME_STATE_DIR = STATE_DIR / "arm_home"
# Points per leg of a manual move; the coordinator interpolates between them.
_MANUAL_POINTS_PER_LEG = 40
# Longest stop() waits for a planned move to wind down, in seconds.
_STOP_TIMEOUT_S = 15.0

ManualTarget = Literal["home", "rest"]


class HomeArm(BaseModel):
    """One arm that can be sent home.

    Attributes:
        home: The default home pose, rad, in the planning group's joint order.
        rest: The folded pose a manual de-home ends at, rad.
        via: A pose every manual move passes through, rad; ``None`` goes straight.
        max_joint_speeds: Each joint's speed limit, rad/s.
        button: The Quest button that sends this arm home, e.g. "X"; ``None`` for none.
        grip: The Quest grip that teleoperates this arm. While it is held the
            button is ignored, and squeezing it cancels a planned move.
    """

    home: list[float]
    rest: list[float]
    via: list[float] | None = None
    max_joint_speeds: list[float]
    button: str | None = None
    grip: str | None = None

    @model_validator(mode="after")
    def _check(self) -> Self:
        poses = [self.home, self.rest, *([] if self.via is None else [self.via])]
        if any(len(pose) != len(self.max_joint_speeds) for pose in poses):
            raise ValueError("home, rest, via and max_joint_speeds need one value per joint")
        if any(not math.isfinite(value) for pose in poses for value in pose):
            raise ValueError("home poses must be finite")
        if any(not math.isfinite(speed) or speed <= 0.0 for speed in self.max_joint_speeds):
            raise ValueError("max_joint_speeds must be finite and positive")
        for name in (self.button, self.grip):
            if name is not None and BUTTON_ALIASES.get(name, name) not in Buttons.BITS:
                raise ValueError(f"unknown Quest button {name!r}")
        return self


class ArmHomeConfig(ModuleConfig):
    """Settings of :class:`ArmHomeModule`.

    Attributes:
        robot: Names the file the saved homes are kept in, e.g. "dual_openyam".
        arms: The arms, by planning group id.
        manual_speed_fraction: How fast a manual move runs, as a fraction of
            each joint's speed limit.
        planned_speed_scale: The planner's speed scale for a planned move;
            ``None`` uses the planner's own.
    """

    robot: str = Field(min_length=1)
    arms: dict[str, HomeArm] = Field(min_length=1)
    manual_speed_fraction: float = Field(default=0.2, gt=0.0, le=1.0)
    planned_speed_scale: float | None = Field(default=None, gt=0.0, le=1.0)

    @property
    def state_file(self) -> Path:
        return HOME_STATE_DIR / f"{self.robot}.json"


def manual_trajectory(
    joint_names: Sequence[str],
    waypoints: Sequence[Sequence[float]],
    max_joint_speeds: Sequence[float],
    speed_fraction: float,
) -> JointTrajectory:
    """A joint-space path through ``waypoints`` that starts and ends each leg at rest.

    Each leg takes as long as its slowest joint needs at ``speed_fraction`` of
    that joint's speed limit, so no joint ever exceeds that speed. The first
    waypoint is where the arm is now.
    """
    points = [TrajectoryPoint(time_from_start=0.0, positions=list(waypoints[0]))]
    elapsed = 0.0
    for start, end in pairwise(waypoints):
        # A smoothstep leg peaks at 1.5 times its mean speed.
        duration = max(
            1.5 * abs(b - a) / (speed_fraction * limit)
            for a, b, limit in zip(start, end, max_joint_speeds, strict=True)
        )
        if duration <= 0.0:
            continue
        for step in range(1, _MANUAL_POINTS_PER_LEG + 1):
            fraction = step / _MANUAL_POINTS_PER_LEG
            blend = fraction * fraction * (3.0 - 2.0 * fraction)
            points.append(
                TrajectoryPoint(
                    time_from_start=elapsed + fraction * duration,
                    positions=[a + blend * (b - a) for a, b in zip(start, end, strict=True)],
                )
            )
        elapsed += duration
    return JointTrajectory(joint_names=list(joint_names), points=points)


def load_saved_homes(path: Path, arms: Mapping[str, HomeArm]) -> dict[str, list[float]]:
    """The homes saved in ``path`` for the arms given; a missing file saves none.

    Raises ``ValueError`` when the file names an arm with the wrong joint count
    or a value that is not a finite number.
    """
    if not path.exists():
        return {}
    saved = json.loads(path.read_text())
    homes: dict[str, list[float]] = {}
    for arm, pose in saved.items():
        if arm not in arms:
            continue
        if (
            not isinstance(pose, list)
            or len(pose) != len(arms[arm].home)
            or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in pose)
            or any(not math.isfinite(value) for value in pose)
        ):
            raise ValueError(f"{path} holds an invalid home for {arm!r}")
        homes[arm] = [float(value) for value in pose]
    return homes


class ArmHomeModule(Module):
    """Send each arm to its home pose, and keep a home the operator saved.

    A Quest button press plans a collision-free move home and runs it, unless
    that arm's grip is held; squeezing the grip during the move cancels it.
    ``manual_move`` skips the planner. A saved home replaces the default for
    both and survives a restart.
    """

    config: ArmHomeConfig

    button_pressed: In[Buttons]
    teleop_buttons: In[Buttons]
    manipulation: ManipulationSpec
    coordinator: ControlCoordinator

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._saved: dict[str, list[float]] = {}
        self._joints: dict[str, tuple[str, ...]] = {}
        self._held = 0
        self._moving: str | None = None
        self._worker: threading.Thread | None = None
        self._stopping = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()
        with self._lock:
            self._saved = load_saved_homes(self.config.state_file, self.config.arms)
        self.register_disposable(Disposable(self.button_pressed.subscribe(self._on_pressed)))
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_held)))

    @rpc
    def stop(self) -> None:
        self._stopping.set()
        with self._lock:
            worker = self._worker
            moving = self._moving is not None
        if moving:
            self.manipulation.cancel()
        if worker is not None:
            worker.join(timeout=_STOP_TIMEOUT_S)
        super().stop()

    @rpc
    def get_homes(self) -> dict[str, dict[str, Any]]:
        """Each arm's home in use, and whether the operator saved it."""
        with self._lock:
            return {
                arm: {"home": list(self._saved.get(arm, spec.home)), "saved": arm in self._saved}
                for arm, spec in self.config.arms.items()
            }

    @rpc
    def save_home(self, arm: str) -> list[float]:
        """Make where ``arm`` is now its home, and keep it across restarts."""
        self._arm(arm)
        pose = self._measured(arm)
        with self._lock:
            self._saved[arm] = pose
            self._write()
        logger.info("Home saved", arm=arm, home=[round(value, 4) for value in pose])
        return pose

    @rpc
    def clear_home(self, arm: str) -> list[float]:
        """Forget ``arm``'s saved home and return the default it falls back to."""
        spec = self._arm(arm)
        with self._lock:
            self._saved.pop(arm, None)
            self._write()
        return list(spec.home)

    @rpc
    def go_home(self, arm: str) -> str:
        """Plan a collision-free move home for ``arm`` and run it; returns at once.

        The result says whether the move started, never whether it finished.
        """
        self._arm(arm)
        with self._lock:
            if self._stopping.is_set():
                return "stopping"
            if self._moving is not None:
                return f"busy: {self._moving} is moving"
            self._moving = arm
            self._worker = threading.Thread(
                target=self._planned_move, args=(arm,), name="arm-home", daemon=True
            )
            self._worker.start()
        return "started"

    @rpc
    def manual_move(self, arm: str, target: ManualTarget) -> str:
        """Move ``arm`` to its home or its rest pose without the planner.

        The path is joint-space, through the arm's ``via`` pose, at
        ``manual_speed_fraction`` of each joint's speed limit. Nothing checks
        it for collisions. Returns the coordinator's verdict on starting it.
        """
        spec = self._arm(arm)
        if target not in ("home", "rest"):
            raise ValueError(f"manual target must be 'home' or 'rest', got {target!r}")
        with self._lock:
            goal = list(self._saved.get(arm, spec.home)) if target == "home" else list(spec.rest)
        waypoints = [self._measured(arm), *([] if spec.via is None else [spec.via]), goal]
        trajectory = manual_trajectory(
            self._joint_names(arm),
            waypoints,
            spec.max_joint_speeds,
            self.config.manual_speed_fraction,
        )
        result = self.coordinator.execute_trajectory(trajectory)
        logger.info(
            "Manual move",
            arm=arm,
            target=target,
            duration_s=round(trajectory.duration, 1),
            status=result.status.name,
        )
        if result.status is TrajectoryExecutionStatus.ACCEPTED:
            return f"started: {trajectory.duration:.1f} s"
        return f"refused: {result.message or result.status.name}"

    @rpc
    def cancel(self) -> None:
        """Stop a planned or manual move where it is."""
        self.manipulation.cancel()
        self.coordinator.cancel_trajectory()

    def _arm(self, arm: str) -> HomeArm:
        spec = self.config.arms.get(arm)
        if spec is None:
            raise ValueError(f"unknown arm {arm!r}; known: {sorted(self.config.arms)}")
        return spec

    def _joint_names(self, arm: str) -> tuple[str, ...]:
        """The planning group's joints, looked up from the planner on first use."""
        with self._lock:
            names = self._joints.get(arm)
        if names is None:
            groups = {group.id: group for group in self.manipulation.list_planning_groups()}
            if arm not in groups:
                raise ValueError(f"the planner has no group {arm!r}; it has {sorted(groups)}")
            names = tuple(groups[arm].joint_names)
            if len(names) != len(self.config.arms[arm].home):
                raise ValueError(f"{arm!r} has {len(names)} joints, its home a different count")
            with self._lock:
                self._joints[arm] = names
        return names

    def _measured(self, arm: str) -> list[float]:
        positions = self.coordinator.get_joint_positions()
        names = self._joint_names(arm)
        missing = [name for name in names if not math.isfinite(positions.get(name, math.nan))]
        if missing:
            raise RuntimeError(f"no measured position for {missing}")
        return [float(positions[name]) for name in names]

    def _write(self) -> None:
        """Write the saved homes. Caller holds ``self._lock``."""
        path = self.config.state_file
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self._saved, indent=2) + "\n")

    def _on_pressed(self, msg: Buttons) -> None:
        """Send home each arm whose button was pressed while its grip is open."""
        with self._lock:
            held = self._held
        for arm, spec in self.config.arms.items():
            if spec.button is None or not msg.data & _bit(spec.button):
                continue
            if spec.grip is not None and held & _bit(spec.grip):
                logger.info("Home button ignored while the arm is teleoperated", arm=arm)
                continue
            logger.info("Home button", arm=arm, result=self.go_home(arm))

    def _on_held(self, msg: Buttons) -> None:
        """Cancel a planned move when the moving arm's grip is squeezed."""
        with self._lock:
            self._held = msg.data & Buttons.DIGITAL_MASK
            moving = self._moving
            grip = None if moving is None else self.config.arms[moving].grip
            cancel = grip is not None and bool(self._held & _bit(grip))
        if cancel:
            threading.Thread(target=self.manipulation.cancel, name="arm-home-cancel").start()

    def _planned_move(self, arm: str) -> None:
        try:
            with self._lock:
                goal = list(self._saved.get(arm, self.config.arms[arm].home))
            target = JointState(name=list(self._joint_names(arm)), position=goal)
            plan = self.manipulation.plan_to_joints(
                {arm: target}, speed_scale=self.config.planned_speed_scale
            )
            if not plan.succeeded or plan.plan is None:
                logger.warning(
                    "Home move not planned", arm=arm, reason=plan.message or plan.status.name
                )
                return
            sent = self.manipulation.execute(blocking=False, plan_id=plan.plan.plan_id)
            if sent.status is not ExecutionStatus.ACCEPTED:
                logger.warning("Home move not started", arm=arm, reason=sent.message)
                return
            done = self.manipulation.wait_for_execution()
            logger.info("Home move ended", arm=arm, status=done.status.name)
        finally:
            with self._lock:
                self._moving = None
                self._worker = None


def _bit(button: str) -> int:
    return 1 << Buttons.BITS[BUTTON_ALIASES.get(button, button)]
