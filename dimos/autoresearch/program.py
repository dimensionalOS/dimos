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

"""The program contract and its runner: a candidate defines ``run(robot, task)``
and imports only ``math``; ``run_program`` runs it in a ``python -I`` subprocess
with a hard timeout, and every robot call it makes is appended to the trial's
``trace.jsonl``, with a camera keyframe before and after each motion call."""

from __future__ import annotations

import ast
import builtins
from collections.abc import Callable, Mapping, Sequence
import dataclasses
from dataclasses import dataclass
import importlib
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback
from types import ModuleType
from typing import Any, Literal, Protocol, TypeVar

from pydantic import BaseModel

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    ExecutionResult,
    ExecutionStatus,
    MoveResult,
    PlanResult,
)
from dimos.manipulation.sdk import Arm, MotionError
from dimos.mapping.pointclouds.box_filter_spec import PointCloudBoxFilterSpec
from dimos.msgs.geometry_msgs.Pose import Pose as PoseMsg
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration_spec import (
    ObjectSceneRegistrationSpec,
)
from dimos.porcelain.dimos import Dimos

T = TypeVar("T")

ProgramStatus = Literal["completed", "raised", "timeout"]
CallStatus = Literal["OK", "PLANNING_FAILED", "EXECUTION_FAILED", "TIMEOUT", "RAISED"]

TRACE_NAME = "trace.jsonl"
# The box filter's name for the one object a program may leave out of the obstacle map.
PROGRAM_BOX = "autoresearch/target"
# How often the runner checks whether the lab asked it to stop, seconds.
_STOP_POLL_S = 0.2


@dataclass(frozen=True)
class Pose:
    """Where the end effector is: ``position`` in metres and ``orientation`` as
    an xyzw quaternion, both in the world frame."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class SceneObject:
    """One detected object: ``position`` is the centre of its bounding box and
    ``size`` the box extents, both in metres in the world frame."""

    id: str
    name: str
    position: tuple[float, float, float]
    size: tuple[float, float, float]


@dataclass(frozen=True)
class Task:
    """What the program is asked to do, in words."""

    text: str


class RobotCallError(RuntimeError):
    """A robot call did not complete. ``status`` says why: PLANNING_FAILED,
    EXECUTION_FAILED or TIMEOUT. A program may catch it and try another way."""

    def __init__(self, status: CallStatus, message: str) -> None:
        self.status: CallStatus = status
        super().__init__(message)


class RunStoppedError(RuntimeError):
    """The lab stopped the run before the trial could be judged: the program was
    killed, or a wait on the world was abandoned."""


class Observer(Protocol):
    """Writes the current camera frame to ``path`` as a PNG."""

    def capture(self, path: Path) -> None: ...


class RobotBackend(Protocol):
    """What a backend does for ``Robot``: reads, blocking moves that raise
    ``RobotCallError``, gripper commands in URDF joint units, object detection
    and camera frames."""

    def joints(self) -> list[float]: ...

    def pose(self) -> Pose: ...

    def move_joints(self, positions: Sequence[float]) -> None: ...

    def move_pose(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float] | None,
    ) -> None: ...

    def move_linear(self, dx: float, dy: float, dz: float) -> None: ...

    def home(self) -> None: ...

    def open_gripper(self) -> None: ...

    def close_gripper(self) -> None: ...

    def set_gripper(self, joint_value: float) -> None: ...

    def gripper_range(self) -> tuple[float, float]: ...

    def objects(self, words: Sequence[str]) -> list[SceneObject]: ...

    def clear_object(self, object_id: str, margin: float) -> None: ...

    def restore(self) -> None: ...

    def capture(self, path: Path) -> None: ...


BackendFactory = Callable[[Path], RobotBackend]


class TraceLine(BaseModel):
    """One robot call as written to ``trace.jsonl``. ``before`` and ``after``
    are keyframe paths relative to the trial directory, set for motion calls."""

    ts: float
    call: str
    args: dict[str, Any]
    status: CallStatus
    result: Any = None
    duration_s: float
    before: str | None = None
    after: str | None = None


class Robot:
    """The API a program drives: ``arm``, ``gripper``, ``scene``, ``observe()``
    and ``log()``. Every call is appended to ``trace.jsonl`` in the trial
    directory; motion calls also save a keyframe before and after.
    Failed calls raise."""

    def __init__(self, backend: RobotBackend, trial_dir: Path) -> None:
        self._backend = backend
        self._trial_dir = trial_dir
        self._trace = trial_dir / TRACE_NAME
        self._trace.write_text("")
        (trial_dir / "keyframes").mkdir(exist_ok=True)
        self._frames = 0
        self.arm = ArmApi(self)
        self.gripper = GripperApi(self)
        self.scene = SceneApi(self)

    def observe(self) -> Path:
        """Save a camera frame into the trial's ``keyframes`` folder and return its path."""
        name = self._traced("observe", {}, lambda: self._keyframe("observe"), motion=False)
        return self._trial_dir / name

    def log(self, text: str) -> None:
        """Write a line of the program's own commentary into the trace."""
        self._write(
            TraceLine(ts=time.time(), call="log", args={"text": text}, status="OK", duration_s=0.0)
        )

    def _keyframe(self, label: str) -> str:
        self._frames += 1
        name = f"keyframes/{self._frames:03d}-{label}.png"
        self._backend.capture(self._trial_dir / name)
        return name

    def _traced(
        self, call: str, args: dict[str, Any], action: Callable[[], T], *, motion: bool
    ) -> T:
        before = self._keyframe(f"{call}.before") if motion else None
        started = time.monotonic()
        status: CallStatus = "OK"
        result: Any = None
        failure: Exception | None = None
        try:
            value = action()
            result = _as_json(value)
        except RobotCallError as error:
            failure, status, result = error, error.status, str(error)
        except Exception as error:
            failure, status, result = error, "RAISED", f"{type(error).__name__}: {error}"
        duration = time.monotonic() - started
        after = self._keyframe(f"{call}.after") if motion else None
        self._write(
            TraceLine(
                ts=time.time(),
                call=call,
                args=args,
                status=status,
                result=result,
                duration_s=duration,
                before=before,
                after=after,
            )
        )
        if failure is not None:
            raise failure
        return value

    def _write(self, line: TraceLine) -> None:
        with self._trace.open("a") as trace:
            trace.write(line.model_dump_json() + "\n")


class ArmApi:
    """Arm reads and blocking moves. Positions are metres and angles radians,
    in the world frame."""

    def __init__(self, robot: Robot) -> None:
        self._robot = robot

    def joints(self) -> list[float]:
        """Current joint positions in the arm's joint order, radians."""
        return self._robot._traced("arm.joints", {}, self._robot._backend.joints, motion=False)

    def pose(self) -> Pose:
        """Current end-effector pose."""
        return self._robot._traced("arm.pose", {}, self._robot._backend.pose, motion=False)

    def move_joints(self, positions: Sequence[float]) -> None:
        """Move to joint positions (radians, arm joint order); returns when the arm is there."""
        values = [float(value) for value in positions]
        self._robot._traced(
            "arm.move_joints",
            {"positions": values},
            lambda: self._robot._backend.move_joints(values),
            motion=True,
        )

    def move_pose(
        self, position: Sequence[float], orientation: Sequence[float] | None = None
    ) -> None:
        """Move the end effector to ``position`` (xyz, metres). ``orientation`` is
        roll, pitch, yaw in radians or an xyzw quaternion; omitted keeps the
        current orientation. The path is planned, not necessarily straight."""
        if len(position) != 3:
            raise ValueError(f"expected x, y, z; got {len(position)} values")
        xyz = (float(position[0]), float(position[1]), float(position[2]))
        quaternion = None if orientation is None else _quaternion(orientation)
        self._robot._traced(
            "arm.move_pose",
            {
                "position": list(xyz),
                "orientation": None if quaternion is None else list(quaternion),
            },
            lambda: self._robot._backend.move_pose(xyz, quaternion),
            motion=True,
        )

    def move_linear(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> None:
        """Translate the end effector in a straight line by (dx, dy, dz) metres."""
        self._robot._traced(
            "arm.move_linear",
            {"dx": dx, "dy": dy, "dz": dz},
            lambda: self._robot._backend.move_linear(float(dx), float(dy), float(dz)),
            motion=True,
        )

    def home(self) -> None:
        """Move to the arm's home position."""
        self._robot._traced("arm.home", {}, self._robot._backend.home, motion=True)


class GripperApi:
    """Gripper commands in the robot's URDF joint units; ``range()`` gives the
    closed and open values."""

    def __init__(self, robot: Robot) -> None:
        self._robot = robot

    def open(self) -> None:
        """Open the gripper fully."""
        self._robot._traced("gripper.open", {}, self._robot._backend.open_gripper, motion=True)

    def close(self) -> None:
        """Close the gripper fully, or onto whatever is between the fingers."""
        self._robot._traced("gripper.close", {}, self._robot._backend.close_gripper, motion=True)

    def set(self, value: float) -> None:
        """Move the gripper joint to ``value``, between the closed and open values of ``range()``."""
        self._robot._traced(
            "gripper.set",
            {"value": value},
            lambda: self._robot._backend.set_gripper(float(value)),
            motion=True,
        )

    def range(self) -> tuple[float, float]:
        """The gripper joint's (closed, open) values in URDF units."""
        return self._robot._backend.gripper_range()


class SceneApi:
    """What the camera sees, and the one object the obstacle map may leave out."""

    def __init__(self, robot: Robot) -> None:
        self._robot = robot

    def objects(self, words: Sequence[str]) -> list[SceneObject]:
        """Detect objects matching ``words`` (e.g. ``["red cube", "bowl"]``) in the current view."""
        names = [str(word) for word in words]
        return self._robot._traced(
            "scene.objects",
            {"words": names},
            lambda: self._robot._backend.objects(names),
            motion=False,
        )

    def clear(self, object_id: str, margin: float = 0.03) -> None:
        """Leave the object out of the obstacle map so the arm may touch it. The map
        treats every object as an obstacle, so a move onto one fails planning
        until it is cleared. ``object_id`` is an id from ``objects()``; ``margin``
        is the extra clearance on every side of its box, metres. One object at a
        time: clearing another puts the first back."""
        self._robot._traced(
            "scene.clear",
            {"object_id": object_id, "margin": margin},
            lambda: self._robot._backend.clear_object(str(object_id), float(margin)),
            motion=False,
        )

    def restore(self) -> None:
        """Put the cleared object back in the obstacle map."""
        self._robot._traced("scene.restore", {}, self._robot._backend.restore, motion=False)


def _quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    if len(values) == 3:
        return Quaternion.from_euler(Vector3(*(float(v) for v in values))).to_tuple()
    if len(values) == 4:
        return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))
    raise ValueError(f"expected roll, pitch, yaw or an xyzw quaternion; got {len(values)} values")


def _as_json(value: object) -> Any:
    """The trace's view of a call result: rounded floats, dataclasses as dicts."""
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return _as_json(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {str(key): _as_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_as_json(item) for item in value]
    if isinstance(value, float):
        return round(value, 4)
    return value


@dataclass(frozen=True)
class GripperRange:
    """The gripper joint's URDF value when fully closed and when fully open."""

    closed: float
    open: float


class DimosBackend:
    """``RobotBackend`` over a running DimOS stack: the ``Arm`` SDK for motion
    and the gripper, the object-registration module for the scene, the map's
    box filter for clearing an object, and an ``Observer`` for camera frames."""

    def __init__(
        self,
        arm: Arm,
        scene: ObjectSceneRegistrationSpec,
        box_filter: PointCloudBoxFilterSpec,
        observer: Observer,
        gripper: GripperRange,
    ) -> None:
        self._arm = arm
        self._scene = scene
        self._box_filter = box_filter
        self._observer = observer
        self._gripper = gripper

    @classmethod
    def connect(
        cls,
        observer: Observer,
        gripper: GripperRange,
        *,
        group: str | None = None,
        instance_name: str | None = None,
    ) -> DimosBackend:
        """Attach to the running stack on this machine's transport bus. ``group``
        and ``instance_name`` select the arm when the stack has more than one, as
        in ``Arm.from_app``. The scene module must run with ``target_frame="world"``,
        the frame the arm plans in; ``objects()`` refuses any other frame."""
        app = Dimos.connect()
        arm = Arm.from_app(app, group=group, instance_name=instance_name)
        scene = app.find_module_by_spec(ObjectSceneRegistrationSpec)
        box_filter = app.find_module_by_spec(PointCloudBoxFilterSpec)
        return cls(arm, scene, box_filter, observer, gripper)

    def joints(self) -> list[float]:
        return [float(value) for value in self._arm.joints()]

    def pose(self) -> Pose:
        pose = self._arm.pose()
        return Pose(pose.position.to_tuple(), pose.orientation.to_tuple())

    def move_joints(self, positions: Sequence[float]) -> None:
        self._motion(lambda: self._arm.move_joints(positions))

    def move_pose(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float] | None,
    ) -> None:
        self._motion(lambda: self._arm.move_pose(position, orientation=orientation))

    def move_linear(self, dx: float, dy: float, dz: float) -> None:
        self._motion(lambda: self._arm.move_linear(dx, dy, dz))

    def home(self) -> None:
        self._motion(self._arm.home)

    def open_gripper(self) -> None:
        self._motion(self._arm.open_gripper)

    def close_gripper(self) -> None:
        self._motion(self._arm.close_gripper)

    def set_gripper(self, joint_value: float) -> None:
        closed, opened = self._gripper.closed, self._gripper.open
        travel = (joint_value - closed) / (opened - closed)
        self._motion(lambda: self._arm.set_gripper_position(travel))

    def gripper_range(self) -> tuple[float, float]:
        return (self._gripper.closed, self._gripper.open)

    def objects(self, words: Sequence[str]) -> list[SceneObject]:
        detections = self._scene.scan_scene(text=list(words))
        if detections.header.frame_id != "world":
            raise RuntimeError(
                f"Objects are in frame {detections.header.frame_id!r}; the arm plans in "
                "'world', so the scene module must run with target_frame=\"world\""
            )
        return [
            SceneObject(
                id=str(detection.id),
                name=str(detection.results[0].hypothesis.class_id),
                position=(
                    float(detection.bbox.center.position.x),
                    float(detection.bbox.center.position.y),
                    float(detection.bbox.center.position.z),
                ),
                size=(
                    float(detection.bbox.size.x),
                    float(detection.bbox.size.y),
                    float(detection.bbox.size.z),
                ),
            )
            for detection in detections.detections
            if detection.id and detection.results
        ]

    def clear_object(self, object_id: str, margin: float) -> None:
        detections = self._scene.scan_scene()
        for detection in detections.detections:
            if str(detection.id) == object_id:
                box = detection.bbox
                self._box_filter.set_box(
                    PROGRAM_BOX,
                    PoseMsg(
                        position=Vector3(
                            box.center.position.x, box.center.position.y, box.center.position.z
                        ),
                        orientation=Quaternion(
                            box.center.orientation.x,
                            box.center.orientation.y,
                            box.center.orientation.z,
                            box.center.orientation.w,
                        ),
                    ),
                    Vector3(
                        box.size.x + 2.0 * margin,
                        box.size.y + 2.0 * margin,
                        box.size.z + 2.0 * margin,
                    ),
                    detections.header.frame_id,
                )
                return
        raise ValueError(f"no object with id {object_id!r} in the scene")

    def restore(self) -> None:
        self._box_filter.clear_box(PROGRAM_BOX)

    def capture(self, path: Path) -> None:
        self._observer.capture(path)

    @staticmethod
    def _motion(action: Callable[[], object]) -> None:
        try:
            action()
        except MotionError as error:
            raise RobotCallError(_status_of(error.result), str(error)) from error


def _status_of(result: PlanResult | ExecutionResult | MoveResult | CommandResult) -> CallStatus:
    if isinstance(result, MoveResult):
        if not result.plan.succeeded:
            return "PLANNING_FAILED"
        if result.execution is None:
            return "EXECUTION_FAILED"
        result = result.execution
    if isinstance(result, PlanResult):
        return "PLANNING_FAILED"
    if isinstance(result, ExecutionResult) and result.status is ExecutionStatus.TIMED_OUT:
        return "TIMEOUT"
    return "EXECUTION_FAILED"


def validate_program(text: str) -> None:
    """Raise ``ValueError`` unless ``text`` parses, imports only ``math`` and
    defines a top-level ``run`` taking two arguments."""
    try:
        tree = ast.parse(text)
    except SyntaxError as error:
        raise ValueError(f"program does not parse: {error}") from error
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            imported = [node.module or ""]
        else:
            continue
        for name in imported:
            if name != "math":
                raise ValueError(f"program imports {name!r}; only math may be imported")
    runs = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "run"]
    if len(runs) != 1 or len(runs[0].args.args) != 2:
        raise ValueError("program must define run(robot, task) at the top level")


def _import_math(name: str, *args: object, **kwargs: object) -> ModuleType:
    if name != "math":
        raise ImportError(f"program imports {name!r}; only math may be imported")
    return math


# What a program's code may use: the builtins minus file and code access, with
# imports resolving math only. Import statements go through __import__.
_PROGRAM_BUILTINS: dict[str, Any] = {
    name: value
    for name, value in vars(builtins).items()
    if name not in ("open", "exec", "eval", "compile")
}
_PROGRAM_BUILTINS["__import__"] = _import_math


def load_program(path: Path) -> Callable[[Robot, Task], None]:
    """Run the module body of the program at ``path`` with the program builtins
    and return its ``run`` function. The session validated the file on submit."""
    namespace: dict[str, Any] = {
        "__name__": "candidate",
        "__file__": str(path),
        "__builtins__": dict(_PROGRAM_BUILTINS),
    }
    exec(compile(path.read_text(), str(path), "exec"), namespace)
    run: Callable[[Robot, Task], None] = namespace["run"]
    return run


class ProgramOutcome(BaseModel):
    """How one program run ended. ``error`` is the traceback when it raised
    or a note when it was killed; ``duration_s`` counts the program's own run
    time, not the process start-up."""

    status: ProgramStatus
    error: str = ""
    duration_s: float


_BOOTSTRAP = (
    f"import sys; sys.path.insert(0, {str(DIMOS_PROJECT_ROOT)!r}); "
    "from dimos.autoresearch.program import main; main(sys.argv[1:])"
)


def run_program(
    program_path: Path,
    trial_dir: Path,
    task: str,
    timeout_s: float,
    backend_factory: str,
    env: Mapping[str, str] | None = None,
    stop: threading.Event | None = None,
) -> ProgramOutcome:
    """Run a candidate on one trial in a fresh ``python -I`` process, killed after
    ``timeout_s`` seconds; ``env`` replaces the child's environment and setting
    ``stop`` kills the child and raises ``RunStoppedError``. The program's own
    errors come back as the outcome status; a child that dies before writing an
    outcome (its backend could not connect, or it was killed first) raises ``RuntimeError``."""
    trial_dir.mkdir(parents=True, exist_ok=True)
    (trial_dir / TRACE_NAME).unlink(missing_ok=True)
    (trial_dir / "outcome.json").unlink(missing_ok=True)
    command = [
        sys.executable,
        "-I",
        "-c",
        _BOOTSTRAP,
        str(program_path),
        str(trial_dir),
        task,
        backend_factory,
    ]
    log_path = trial_dir / "program.log"
    started = time.monotonic()
    with log_path.open("wb") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env)
        try:
            _wait(process, timeout_s, stop)
        except subprocess.TimeoutExpired as error:
            if not (trial_dir / TRACE_NAME).is_file():
                raise RuntimeError(
                    f"the program runner was killed after {timeout_s} s before its backend "
                    f"was up; see {log_path}"
                ) from error
            return ProgramOutcome(
                status="timeout",
                error=f"killed after {timeout_s} s",
                duration_s=time.monotonic() - started,
            )
    outcome = trial_dir / "outcome.json"
    if process.returncode != 0 or not outcome.is_file():
        raise RuntimeError(
            f"the program runner exited with code {process.returncode} before writing "
            f"an outcome; see {log_path}"
        )
    return ProgramOutcome.model_validate_json(outcome.read_text())


def _wait(process: subprocess.Popen[bytes], timeout_s: float, stop: threading.Event | None) -> None:
    """Wait for the child; kill it at ``timeout_s`` (``TimeoutExpired``) or once ``stop`` is set (``RunStoppedError``)."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            process.wait(timeout=min(_STOP_POLL_S, max(deadline - time.monotonic(), 0.0)))
            return
        except subprocess.TimeoutExpired:
            if stop is not None and stop.is_set():
                process.kill()
                process.wait()
                raise RunStoppedError("the run was stopped while the program was running") from None
            if time.monotonic() >= deadline:
                process.kill()
                process.wait()
                raise


def main(argv: Sequence[str]) -> None:
    """Entry point of the program subprocess. ``argv`` is the program path, the
    trial directory, the task text and the backend factory, in that order."""
    program, trial, task, factory = argv
    trial_dir = Path(trial)
    module_name, function_name = factory.split(":")
    make_backend: BackendFactory = getattr(importlib.import_module(module_name), function_name)
    robot = Robot(make_backend(trial_dir), trial_dir)
    started = time.monotonic()
    try:
        run = load_program(Path(program))
        run(robot, Task(task))
    except (Exception, SystemExit):
        outcome = ProgramOutcome(
            status="raised", error=traceback.format_exc(), duration_s=time.monotonic() - started
        )
    else:
        outcome = ProgramOutcome(status="completed", duration_s=time.monotonic() - started)
    (trial_dir / "outcome.json").write_text(outcome.model_dump_json(indent=2))
