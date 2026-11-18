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

"""What a robot-arm program sees: ``run(robot, task)`` drives ``robot.arm``,
``robot.gripper``, ``robot.scene``, ``robot.observe()`` and ``robot.log()``.
Every call is appended to ``trace.jsonl``, with a camera keyframe before and
after each motion call. ``API_VERSION`` changes when a program written for
one version could break on the next."""

from __future__ import annotations

from collections.abc import Callable, Sequence
import dataclasses
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Literal, Protocol, TypeVar

from pydantic import BaseModel

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

T = TypeVar("T")

API_VERSION = "1"

CallStatus = Literal["OK", "PLANNING_FAILED", "EXECUTION_FAILED", "TIMEOUT", "RAISED"]

TRACE_NAME = "trace.jsonl"
# Gripper opening as a fraction of travel, the unit of gripper.set and gripper.range.
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 1.0

API_REFERENCE = f"""\
Program API, version {API_VERSION}. A program is a Python file defining run(robot, task);
`import math` is the only import allowed. task.text is the task in words. Positions are metres
and angles radians, in the world frame. Motion calls block until done and raise RobotCallError
(with a .status: PLANNING_FAILED, EXECUTION_FAILED or TIMEOUT) when planning or execution fails;
catch it to try another way.
  robot.arm.joints()                      joint positions, radians, in the arm's joint order
  robot.arm.pose()                        Pose(position xyz, orientation xyzw quaternion)
  robot.arm.move_joints(positions)        planned move to joint positions
  robot.arm.move_pose(xyz, orientation)   planned move; orientation is roll, pitch, yaw or an
                                          xyzw quaternion, omitted keeps the current one
  robot.arm.move_linear(dx, dy, dz)       straight-line translation, metres
  robot.arm.home()                        the home preset
  robot.gripper.open() / .close()         fully open or closed
  robot.gripper.set(x), .range()          the opening as a fraction of travel; range() gives
                                          (closed, open) = ({GRIPPER_CLOSED}, {GRIPPER_OPEN})
  robot.scene.objects(words)              detected objects matching words, e.g. ["cube", "bowl"]:
                                          SceneObject(id, name, position, size), the box centre
                                          and extents
  robot.scene.clear(object_id, margin)    leave that object, plus margin (m, default 0.03) on
                                          every side, out of the obstacle map: the map treats
                                          every object as an obstacle, so clear each object
                                          you will touch or put something into; clears add up
  robot.scene.restore()                   put every cleared object back in the map
  robot.observe()                         saves a keyframe, returns its path
  robot.log(text)                         a line in the trace
Every call goes to trace.jsonl, with a keyframe before and after each motion call.
Example:
  import math

  def run(robot, task):
      \"\"\"Top-down grasp at the cube's centre. May fail if the fingers land on an edge.\"\"\"
      cube = robot.scene.objects(["cube"])[0]
      x, y, z = cube.position
      robot.scene.clear(cube.id)
      robot.gripper.open()
      robot.arm.move_pose([x, y, z + 0.10], [math.pi, 0.0, 0.0])
      robot.arm.move_linear(dz=-0.08)
      robot.gripper.close()
      robot.arm.move_linear(dz=0.10)
"""


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
    """The caller stopped the run before it ended: the program was killed, or
    a wait on the world was abandoned."""


class Observer(Protocol):
    """Writes the current camera frame to ``path`` as a PNG."""

    def capture(self, path: Path) -> None: ...


class RobotBackend(Protocol):
    """What a backend does for ``Robot``: reads, blocking moves that raise
    ``RobotCallError``, gripper commands as a fraction of travel (0 closed,
    1 open), object detection and camera frames."""

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

    def set_gripper(self, travel: float) -> None: ...

    def objects(self, words: Sequence[str]) -> list[SceneObject]: ...

    def clear_object(self, object_id: str, margin: float) -> None: ...

    def restore(self) -> None: ...

    def capture(self, path: Path) -> None: ...


BackendFactory = Callable[[Path], RobotBackend]


class TraceLine(BaseModel):
    """One robot call as written to ``trace.jsonl``. ``before`` and ``after``
    are keyframe paths relative to the run directory, set for motion calls."""

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
    and ``log()``. Every call is appended to ``trace.jsonl`` in ``run_dir``;
    motion calls also save a keyframe before and after. Failed calls raise."""

    def __init__(self, backend: RobotBackend, run_dir: Path) -> None:
        self._backend = backend
        self._run_dir = run_dir
        self._trace = run_dir / TRACE_NAME
        self._trace.write_text("")
        (run_dir / "keyframes").mkdir(exist_ok=True)
        self._frames = 0
        self.arm = ArmApi(self)
        self.gripper = GripperApi(self)
        self.scene = SceneApi(self)

    def observe(self) -> Path:
        """Save a camera frame into the run's ``keyframes`` folder and return its path."""
        name = self._traced("observe", {}, lambda: self._keyframe("observe"), motion=False)
        return self._run_dir / name

    def log(self, text: str) -> None:
        """Write a line of the program's own commentary into the trace."""
        self._write(
            TraceLine(ts=time.time(), call="log", args={"text": text}, status="OK", duration_s=0.0)
        )

    def _keyframe(self, label: str) -> str:
        self._frames += 1
        name = f"keyframes/{self._frames:03d}-{label}.png"
        self._backend.capture(self._run_dir / name)
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
    """Gripper commands as a fraction of travel, 0.0 closed and 1.0 open, on
    any robot; the stack maps that onto its own joint limits."""

    def __init__(self, robot: Robot) -> None:
        self._robot = robot

    def open(self) -> None:
        """Open the gripper fully."""
        self._robot._traced("gripper.open", {}, self._robot._backend.open_gripper, motion=True)

    def close(self) -> None:
        """Close the gripper fully, or onto whatever is between the fingers."""
        self._robot._traced("gripper.close", {}, self._robot._backend.close_gripper, motion=True)

    def set(self, value: float) -> None:
        """Open the gripper to ``value``, a fraction of travel from 0.0 (closed) to 1.0 (open)."""
        self._robot._traced(
            "gripper.set",
            {"value": value},
            lambda: self._robot._backend.set_gripper(float(value)),
            motion=True,
        )

    def range(self) -> tuple[float, float]:
        """The (closed, open) values ``set`` takes: (0.0, 1.0)."""
        return (GRIPPER_CLOSED, GRIPPER_OPEN)


class SceneApi:
    """What the camera sees, and the objects the obstacle map leaves out."""

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
        is the extra clearance on every side of its box, metres. Clears add up:
        clear each object you will touch or put something into."""
        self._robot._traced(
            "scene.clear",
            {"object_id": object_id, "margin": margin},
            lambda: self._robot._backend.clear_object(str(object_id), float(margin)),
            motion=False,
        )

    def restore(self) -> None:
        """Put every object this program cleared back in the obstacle map."""
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
