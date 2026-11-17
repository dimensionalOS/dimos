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

"""Bin handling on the xArm7 kitting bench.

One cycle: the filled bin goes from the loading slot to the first free done slot, then an empty
bin goes from where it lies into the loading slot, aligned, and ``empty_bin_ready`` says so.
Bins are held by the high back wall, beside the parts, with the tool pointing down. Where the
empty bins lie is given, not detected. All poses are in the arm's base frame ("world").
"""

from __future__ import annotations

from collections.abc import Sequence
import math
import threading
import time
from typing import Any, Protocol

from pydantic import BaseModel, Field
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.manipulation.grasp_verification import (
    GraspVerificationConfig,
    await_gripper_settle,
    grasp_failure,
)
from dimos.manipulation.manipulation_spec import (
    ExecutionStatus,
    ManipulationSpec,
    PlanningGroupInfo,
)
from dimos.manipulation.sdk import Arm, MotionError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

IN = 0.0254
# The stack bin: 9.25 x 6 x 5 in, a 3 mm wall, a scoop front and a lip on the back rim.
BIN_LENGTH_M = 9.25 * IN
BIN_WIDTH_M = 6 * IN
BIN_HEIGHT_M = 5 * IN
BIN_WALL_M = 0.003
_OBSTACLE_PREFIX = "kitting/"
EXECUTE_TIMEOUT_S = 120.0
# How many times a long planned move may be split at its midpoint.
MAX_SPLITS = 2
# Plans asked for per planned move; the one that swings the joints least is kept.
PLAN_TRIES = 3


class BinPose(BaseModel):
    """Where a bin stands: its footprint centre (m) and ``yaw_deg``, the direction from its back
    wall to its scoop front, from the x axis."""

    x: float
    y: float
    yaw_deg: float


class BenchBox(BaseModel):
    """A fixed planner obstacle: centre and full size, metres."""

    name: str
    centre: tuple[float, float, float]
    size: tuple[float, float, float]


class PlannerObstacleSpec(Spec, Protocol):
    """The planner's obstacle RPCs, as ManipulationModule has them."""

    def add_obstacle(
        self,
        name: str,
        pose: Pose,
        shape: str,
        dimensions: list[float] | None = None,
        mesh_path: str | None = None,
    ) -> str: ...

    def remove_obstacle(self, obstacle_id: str) -> bool: ...


class XArm7KittingConfig(ModuleConfig):
    """The bench, the slots and how a bin is held. Defaults are the kitting bench twin's
    (FDEIssues/xarm7-bench/BENCH_SPEC.md); grip and carry values come from the twin trials."""

    planning_group: str | None = None
    table_top_z: float = -0.008
    # The loading slot between the dual OpenYAM's resting arms: back wall toward the xArm7.
    loading_slot: BinPose = BinPose(x=0.314, y=0.505, yaw_deg=90.0)
    # Done slots, back wall toward the arm, in the order they are filled: the middle one first.
    # The one nearest the wall puts the held wall 0.36 m from the base, where the arm folds hard
    # with a full bin hanging from it, so it is filled last.
    done_slots: list[BinPose] = Field(
        default_factory=lambda: [BinPose(x=x, y=-0.457, yaw_deg=-90.0) for x in (0.22, 0.38, 0.06)]
    )
    # Indexes of done slots already holding a bin when the module starts.
    done_taken: list[int] = Field(default_factory=list)
    # The grip: along the back wall from its centre (beside the parts), and the tool point
    # below the rim. The pads sit 11 to 48 mm above the tool point.
    grip_offset_m: float = 0.055
    grip_depth_m: float = 0.070
    approach_m: float = 0.10
    # Tool point height while carrying: the bin's bottom clears every bin on the bench.
    carry_z_m: float = 0.30
    # The carry follows the arc about the base in this many short moves, pausing after each
    # so a hanging bin stops swinging.
    carry_steps: int = Field(default=3, ge=1)
    settle_s: float = 1.0
    # A planned move may swing no joint further than this; a longer plan is split at its midpoint.
    max_joint_travel_rad: float = 1.2
    # The first lift, after which the grip is checked again with the bin's weight on it.
    test_lift_m: float = 0.02
    # Released this far above where the bin was gripped.
    release_gap_m: float = 0.008
    speed_scale: float = Field(default=0.3, gt=0.0, le=1.0)
    carry_speed_scale: float = Field(default=0.15, gt=0.0, le=1.0)
    # A closed gripper reading (0 closed, 1 open) at or below this holds nothing: closed on air
    # reads about 0.002, on a real 3 mm wall about 0.035, on the twin's 7 mm wall about 0.06.
    empty_reading: float = Field(default=0.012, ge=0.0, le=1.0)
    bench_boxes: list[BenchBox] = Field(
        default_factory=lambda: [
            BenchBox(name="wall", centre=(-0.422, 0.15, 0.5), size=(0.1, 2.6, 2.0)),
            BenchBox(name="table", centre=(0.314, 0.152, -0.033), size=(0.80, 1.55, 0.05)),
            # the dual OpenYAM's resting arms either side of the loading slot, and its post
            BenchBox(name="yam_wall_side", centre=(0.095, 0.80, 0.17), size=(0.23, 0.58, 0.34)),
            BenchBox(name="yam_front_side", centre=(0.56, 0.82, 0.17), size=(0.18, 0.56, 0.34)),
            BenchBox(name="yam_post", centre=(0.315, 0.82, 0.25), size=(0.13, 0.15, 0.50)),
        ]
    )


class XArm7KittingModule(Module):
    """Moves bins on the kitting bench. ``run_cycle`` is the trigger: it starts the cycle on a
    thread and returns at once; ``status`` says where it is and how it ended. On a full cycle,
    ``empty_bin_ready`` goes true once the empty bin stands in the loading slot."""

    config: XArm7KittingConfig

    empty_bin_ready: Out[Bool]

    _manipulation: ManipulationSpec
    _obstacles: PlannerObstacleSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._abort = threading.Event()
        self._thread: threading.Thread | None = None
        self._arm: Arm | None = None
        self._done_taken = set(self.config.done_taken)
        self._loading_full = True
        self._step = "idle"
        self._last: dict[str, Any] | None = None

    @rpc
    def run_cycle(self, x: float, y: float, yaw_deg: float) -> str:
        """Unload the filled bin, then load the empty bin standing at ``x``, ``y`` (m) with its
        scoop toward ``yaw_deg``. Returns at once; poll ``status``."""
        return self._start(lambda: self._cycle(BinPose(x=x, y=y, yaw_deg=yaw_deg)))

    @rpc
    def unload(self) -> str:
        """Carry the bin in the loading slot to the first free done slot. Returns at once."""
        return self._start(self._unload)

    @rpc
    def load(self, x: float, y: float, yaw_deg: float) -> str:
        """Carry the empty bin at ``x``, ``y``, ``yaw_deg`` into the loading slot. Returns at once."""
        return self._start(lambda: self._load(BinPose(x=x, y=y, yaw_deg=yaw_deg)))

    @rpc
    def abort(self) -> None:
        """Stop after the current motion, and cancel it."""
        self._abort.set()
        self._manipulation.cancel()

    @rpc
    def set_done_taken(self, slots: list[int]) -> None:
        """Say which done slots hold a bin, by index; ``[]`` once the done zone is cleared."""
        self._done_taken = set(slots)

    @rpc
    def set_loading_full(self, full: bool) -> None:
        """Say whether the loading slot holds a bin (it starts holding one)."""
        self._loading_full = full

    @rpc
    def status(self) -> dict[str, Any]:
        return {
            "busy": self._thread is not None and self._thread.is_alive(),
            "step": self._step,
            "last": self._last,
            "done_taken": sorted(self._done_taken),
            "loading_full": self._loading_full,
        }

    def _start(self, job: Any) -> str:
        if not self._lock.acquire(blocking=False):
            return "busy: a bin is already moving"
        self._abort.clear()
        self._last = None

        def run() -> None:
            try:
                self._last = job()
            except Exception as error:
                logger.error("Kitting job failed", error=str(error))
                self._last = {"ok": False, "reason": str(error)}
            finally:
                self._step = "idle"
                self._lock.release()

        self._thread = threading.Thread(target=run, name="xarm7-kitting", daemon=True)
        self._thread.start()
        return "started"

    def _cycle(self, empty: BinPose) -> dict[str, Any]:
        unloaded = self._unload()
        if not unloaded["ok"]:
            return {"ok": False, "unload": unloaded}
        loaded = self._load(empty)
        return {"ok": loaded["ok"], "unload": unloaded, "load": loaded}

    def _unload(self) -> dict[str, Any]:
        free = [i for i in range(len(self.config.done_slots)) if i not in self._done_taken]
        if not free:
            return {"ok": False, "reason": "every done slot is taken; call set_done_taken([])"}
        slot = free[0]
        target = self.config.done_slots[slot]
        others = [self.config.done_slots[i] for i in sorted(self._done_taken)]
        result = self._move_bin(self.config.loading_slot, target, others)
        if result["ok"]:
            self._done_taken.add(slot)
            self._loading_full = False
            self.empty_bin_ready.publish(Bool(False))
        result["done_slot"] = slot
        return result

    def _load(self, empty: BinPose) -> dict[str, Any]:
        if self._loading_full:
            return {"ok": False, "reason": "the loading slot holds a bin; unload it first"}
        others = [self.config.done_slots[i] for i in sorted(self._done_taken)]
        result = self._move_bin(empty, self.config.loading_slot, others)
        if result["ok"]:
            self._loading_full = True
            self.empty_bin_ready.publish(Bool(True))
            logger.info("EMPTY_BIN_IN_KITTING_AREA")
        return result

    def _move_bin(self, source: BinPose, target: BinPose, others: list[BinPose]) -> dict[str, Any]:
        added: list[str] = []
        steps: list[str] = []
        try:
            arm = self._selected_arm()
            added = self._add_obstacles(others)
            cfg = self.config
            grip_z = cfg.table_top_z + BIN_HEIGHT_M - cfg.grip_depth_m
            pick_xy, pick_yaw = grip_point(source, cfg.grip_offset_m)
            place_xy, _ = grip_point(target, cfg.grip_offset_m)
            # Carry by turning the base: the bin rides an arc, its back wall still toward the
            # arm. Then the rest of the bin's turn, the shorter way.
            turn = _wrap(math.atan2(place_xy[1], place_xy[0]) - math.atan2(pick_xy[1], pick_xy[0]))
            bin_turn = turn + _wrap(math.radians(target.yaw_deg - source.yaw_deg) - turn)
            # The wrist turn is the tool's yaw less the base's: keep it near the present one.
            now = arm.pose()
            wrist_now = tool_yaw(now.orientation.to_tuple()) - math.atan2(
                now.position.y, now.position.x
            )
            pick_yaw = wrist_friendly_yaw(
                pick_yaw,
                bin_turn,
                math.atan2(pick_xy[1], pick_xy[0]),
                math.atan2(place_xy[1], place_xy[0]),
                wrist_now,
            )
            pick_q = tool_quaternion(pick_yaw)
            place_q = tool_quaternion(pick_yaw + bin_turn)

            arm.open_gripper()
            self._next(steps, "approach")
            arm.move_pose(
                [*pick_xy, grip_z + cfg.approach_m], orientation=pick_q, speed_scale=cfg.speed_scale
            )
            self._next(steps, "descend")
            self._vertical(arm, [*pick_xy, grip_z], pick_q, cfg.speed_scale)
            self._next(steps, "grip")
            arm.close_gripper()
            failure = self._grip_failure(arm)
            if failure is not None:
                arm.open_gripper()
                self._vertical(arm, [*pick_xy, grip_z + cfg.approach_m], pick_q, cfg.speed_scale)
                return {"ok": False, "step": "grip", "reason": failure, "steps": steps}
            self._next(steps, "lift")
            # A short lift first: a bin that is not held stays on the table, not mid-air.
            self._vertical(arm, [*pick_xy, grip_z + cfg.test_lift_m], pick_q, cfg.carry_speed_scale)
            time.sleep(cfg.settle_s)
            self._check_holding(arm, "after the short lift")
            self._vertical(arm, [*pick_xy, cfg.carry_z_m], pick_q, cfg.carry_speed_scale)
            time.sleep(cfg.settle_s)
            self._check_holding(arm, "after the lift")
            self._next(steps, "carry")
            for waypoint, q in carry_waypoints(
                pick_xy, place_xy, pick_yaw, bin_turn, turn, cfg.carry_steps
            ):
                self._planned(arm, [*waypoint, cfg.carry_z_m], q, cfg.carry_speed_scale)
                time.sleep(cfg.settle_s)
                self._check_holding(arm, "while carrying")
            self._next(steps, "lower")
            self._vertical(
                arm, [*place_xy, grip_z + cfg.release_gap_m], place_q, cfg.carry_speed_scale
            )
            self._next(steps, "release")
            arm.open_gripper()
            self._vertical(
                arm,
                [*place_xy, grip_z + cfg.release_gap_m + cfg.approach_m],
                place_q,
                cfg.speed_scale,
            )
            self._next(steps, "home")
            arm.home(speed_scale=cfg.speed_scale)
            return {"ok": True, "steps": steps}
        except Exception as error:
            logger.error(
                "Kitting move failed", step=steps[-1] if steps else "start", error=str(error)
            )
            return {
                "ok": False,
                "step": steps[-1] if steps else "start",
                "reason": str(error),
                "steps": steps,
            }
        finally:
            for obstacle in added:
                self._obstacles.remove_obstacle(obstacle)

    def _vertical(self, arm: Arm, xyz: list[float], q: list[float], speed: float) -> None:
        """Straight to ``xyz``; a planned move there if the straight line has no IK solution."""
        now = arm.pose().position
        try:
            arm.move_linear(xyz[0] - now.x, xyz[1] - now.y, xyz[2] - now.z, speed_scale=speed)
        except MotionError as error:
            logger.warning("Straight move failed, planning it instead", reason=str(error)[:160])
            self._planned(arm, xyz, q, speed)

    def _planned(
        self, arm: Arm, xyz: list[float], q: list[float], speed: float, depth: int = 0
    ) -> None:
        """A planned move to ``xyz``, ``q`` that swings no joint more than
        ``max_joint_travel_rad``; a plan that would goes through the midpoint instead."""
        target = PoseStamped(frame_id="world", position=Vector3(*xyz), orientation=Quaternion(*q))
        # Plans differ from one call to the next: keep the one that swings the joints least.
        plan, travel = None, math.inf
        for _ in range(PLAN_TRIES):
            attempt = self._manipulation.plan_to_poses({arm.info.id: target}, speed_scale=speed)
            if not attempt.succeeded or attempt.plan is None:
                if plan is None:
                    plan = attempt
                continue
            attempt_travel = joint_travel(attempt.plan.path, arm.info.joint_names)
            if attempt_travel < travel:
                plan, travel = attempt, attempt_travel
            if travel <= self.config.max_joint_travel_rad:
                break
        if plan is None:
            raise RuntimeError("the planner returned no plan")
        if not plan.succeeded or plan.plan is None:
            raise MotionError("move_pose", plan)
        if travel > self.config.max_joint_travel_rad:
            if depth >= MAX_SPLITS:
                raise RuntimeError(f"every plan there swings a joint {travel:.2f} rad")
            logger.warning(
                "Plan swings too far, going through the midpoint", travel=round(travel, 2)
            )
            now = arm.pose()
            here = (now.position.x, now.position.y)
            here_yaw = tool_yaw(now.orientation.to_tuple())
            (mid_xy, mid_q), _ = carry_waypoints(
                here,
                (xyz[0], xyz[1]),
                here_yaw,
                _wrap(tool_yaw(q) - here_yaw),
                _wrap(math.atan2(xyz[1], xyz[0]) - math.atan2(here[1], here[0])),
                2,
            )
            self._planned(arm, [*mid_xy, (now.position.z + xyz[2]) / 2], mid_q, speed, depth + 1)
            self._planned(arm, xyz, q, speed, depth + 1)
            return
        result = self._manipulation.execute(
            blocking=True, timeout=EXECUTE_TIMEOUT_S, plan_id=plan.plan.plan_id
        )
        if result.status is not ExecutionStatus.COMPLETED:
            raise MotionError("move_pose", result)

    def _check_holding(self, arm: Arm, when: str) -> None:
        """Raise if the jaws have closed past the wall: the bin slipped out."""
        reading = arm.state().gripper_position
        if reading is not None and reading <= self.config.empty_reading:
            raise RuntimeError(
                f"the bin slipped out of the gripper {when} (readback {reading:.3f})"
            )

    def _next(self, steps: list[str], step: str) -> None:
        if self._abort.is_set():
            raise RuntimeError(f"aborted before {step}")
        steps.append(step)
        self._step = step

    def _grip_failure(self, arm: Arm) -> str | None:
        check = GraspVerificationConfig(
            closed_position=0.0, empty_epsilon=self.config.empty_reading
        )
        settle = await_gripper_settle(lambda: arm.state().gripper_position, 0.0, check)
        return grasp_failure(settle, check)

    def _add_obstacles(self, bins: list[BinPose]) -> list[str]:
        added = []
        for box in self.config.bench_boxes:
            added.append(
                self._obstacles.add_obstacle(
                    _OBSTACLE_PREFIX + box.name,
                    Pose(Vector3(*box.centre), Quaternion()),
                    "box",
                    list(box.size),
                )
            )
        for i, standing in enumerate(bins):
            q = Rotation.from_euler("z", standing.yaw_deg, degrees=True).as_quat()
            centre = Vector3(standing.x, standing.y, self.config.table_top_z + BIN_HEIGHT_M / 2)
            added.append(
                self._obstacles.add_obstacle(
                    f"{_OBSTACLE_PREFIX}bin{i}",
                    Pose(centre, Quaternion(*q)),
                    "box",
                    [BIN_LENGTH_M, BIN_WIDTH_M, BIN_HEIGHT_M],
                )
            )
        return [name for name in added if name]

    def _selected_arm(self) -> Arm:
        if self._arm is None:
            groups = self._manipulation.list_planning_groups()
            self._arm = Arm(self._manipulation, _pick_group(groups, self.config.planning_group))
        return self._arm


def grip_point(bin_pose: BinPose, offset_m: float) -> tuple[tuple[float, float], float]:
    """Where the tool point goes to hold ``bin_pose`` by its back wall, ``offset_m`` along the
    wall from its centre, and the yaw (rad) of the wall's normal there."""
    yaw = math.radians(bin_pose.yaw_deg)
    gx = -BIN_LENGTH_M / 2 + BIN_WALL_M / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return (bin_pose.x + c * gx - s * offset_m, bin_pose.y + s * gx + c * offset_m), yaw


def carry_waypoints(
    start: tuple[float, float],
    end: tuple[float, float],
    start_yaw: float,
    bin_turn: float,
    arc: float,
    steps: int,
) -> list[tuple[tuple[float, float], list[float]]]:
    """Points along the arc about the base from ``start`` to ``end`` (turning ``arc`` rad), with
    the tool turning ``bin_turn`` rad in step; the last point is ``end``."""
    r0, r1 = math.hypot(*start), math.hypot(*end)
    a0 = math.atan2(start[1], start[0])
    points = []
    for i in range(1, steps + 1):
        f = i / steps
        r, a = r0 + (r1 - r0) * f, a0 + arc * f
        xy = end if i == steps else (r * math.cos(a), r * math.sin(a))
        points.append((xy, tool_quaternion(start_yaw + bin_turn * f)))
    return points


def joint_travel(path: Sequence[JointState], joints: Sequence[str]) -> float:
    """The most any of ``joints`` moves from the path's first point, rad."""
    if not path:
        return 0.0
    first = dict(zip(path[0].name, path[0].position, strict=True))
    most = 0.0
    for point in path[1:]:
        for name, value in zip(point.name, point.position, strict=True):
            if name in joints and name in first:
                most = max(most, abs(value - first[name]))
    return most


def tool_yaw(xyzw: Sequence[float]) -> float:
    """The wall-normal yaw (rad) a tool orientation closes its fingers along."""
    m = Rotation.from_quat(xyzw).as_matrix()
    return math.atan2(m[1, 0], m[0, 0]) - math.pi / 2


def wrist_friendly_yaw(
    wall_yaw: float, bin_turn: float, pick_angle: float, place_angle: float, wrist_now: float
) -> float:
    """``wall_yaw`` or the same wall from the other side (the fingers are symmetric), whichever
    keeps the wrist's turn (tool yaw less the base's angle) nearer ``wrist_now`` at the pick and
    at the place, so the last joint stays clear of its limits."""

    def wrist_offset(yaw: float) -> float:
        return max(
            abs(_wrap(yaw - pick_angle - wrist_now)),
            abs(_wrap(yaw + bin_turn - place_angle - wrist_now)),
        )

    best = min((wall_yaw, wall_yaw + math.pi), key=wrist_offset)
    return pick_angle + wrist_now + _wrap(best - pick_angle - wrist_now)


def tool_quaternion(wall_normal_yaw: float) -> list[float]:
    """The tool pointing down with its fingers closing along the wall normal at
    ``wall_normal_yaw`` (rad), as xyzw. The fingers close along the tool's y axis."""
    down = Rotation.from_euler("xyz", [math.pi, 0.0, wall_normal_yaw + math.pi / 2])
    return [float(v) for v in down.as_quat()]


def _wrap(angle: float) -> float:
    """``angle`` (rad) in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _pick_group(groups: Sequence[PlanningGroupInfo], wanted: str | None) -> PlanningGroupInfo:
    matches = [g for g in groups if (g.id == wanted if wanted else g.tip_frame is not None)]
    if len(matches) != 1:
        raise ValueError(
            f"no single arm among planning groups {[g.id for g in groups]}; set planning_group"
        )
    return matches[0]
