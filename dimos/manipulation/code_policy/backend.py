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

"""Run a program's robot calls on a running DimOS stack: the ``Arm`` SDK for
motion and the gripper, the object-registration module for the scene, the
map's box filter for clearing objects, and the camera stream for keyframes.
``backend`` is the factory a runner names as ``LIVE_BACKEND``."""

from __future__ import annotations

import atexit
from collections.abc import Callable, Sequence
import math
from pathlib import Path
import threading
import time

import cv2

from dimos.core.stream import RemoteOut
from dimos.manipulation.code_policy.api import (
    CallStatus,
    Observer,
    Pose,
    RobotBackend,
    RobotCallError,
    SceneObject,
)
from dimos.manipulation.code_policy.scene_spec import ObjectSceneSpec
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
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.porcelain.dimos import Dimos

# How long scene.clear and scene.restore wait for an obstacle map with their change, seconds.
MAP_TIMEOUT_S = 2.0
# How long the first camera frame may take to arrive, seconds.
FRAME_TIMEOUT_S = 10.0


def program_box(object_id: str) -> str:
    """The box filter's name for the box that leaves ``object_id`` out of the obstacle map."""
    return f"program/{object_id}"


class MapStamps:
    """The capture time of the newest obstacle map: feed ``on_map`` from the
    map stream, and ``wait_newer`` blocks until a map captured after a time
    arrives."""

    def __init__(self) -> None:
        self._newest = -math.inf
        self._changed = threading.Condition()

    def on_map(self, msg: PointCloud2) -> None:
        if msg.ts is None:
            return
        with self._changed:
            self._newest = max(self._newest, msg.ts)
            self._changed.notify_all()

    def wait_newer(self, than: float, timeout_s: float) -> bool:
        """True once a map captured after ``than`` (Unix time, seconds) has arrived; False after ``timeout_s``."""
        with self._changed:
            return self._changed.wait_for(lambda: self._newest > than, timeout_s)


def subscribe_map(app: Dimos, maps: MapStamps) -> Callable[[], None]:
    """Feed ``maps`` from the box filter's output, the map the planner avoids; returns the unsubscribe."""
    outputs = [
        info
        for info in app.list_streams()
        if info.name == "filtered_cloud" and info.direction == "out"
    ]
    if len(outputs) != 1 or outputs[0].module_name is None:
        raise RuntimeError(
            f"expected one filtered_cloud stream, found {[info.module_name for info in outputs]}"
        )
    stream = app.get_module(outputs[0].module_name).get_stream("filtered_cloud")
    return stream.subscribe(maps.on_map)


class StreamObserver:
    """Keeps the newest frame of an image stream and writes it as a PNG on ``capture``."""

    def __init__(self, stream: RemoteOut[Image]) -> None:
        self._frame: Image | None = None
        self._arrived = threading.Event()
        self._unsubscribe = stream.subscribe(self._on_frame)

    def _on_frame(self, image: Image) -> None:
        self._frame = image
        self._arrived.set()

    def capture(self, path: Path) -> None:
        if not self._arrived.wait(FRAME_TIMEOUT_S):
            raise RuntimeError(f"no camera frame arrived within {FRAME_TIMEOUT_S} s")
        frame = self._frame
        assert frame is not None
        if not cv2.imwrite(str(path), frame.to_opencv()):
            raise RuntimeError(f"could not write {path}")

    def close(self) -> None:
        self._unsubscribe()


class DimosBackend:
    """``RobotBackend`` over a running DimOS stack: the ``Arm`` SDK for motion
    and the gripper, the object-registration module for the scene, the map's
    box filter for clearing objects, and an ``Observer`` for camera frames.
    ``maps`` must be fed from the box filter's output: ``clear_object`` and
    ``restore`` return once a map captured after the change has arrived."""

    def __init__(
        self,
        arm: Arm,
        scene: ObjectSceneSpec,
        box_filter: PointCloudBoxFilterSpec,
        observer: Observer,
        maps: MapStamps,
    ) -> None:
        self._arm = arm
        self._scene = scene
        self._box_filter = box_filter
        self._observer = observer
        self._maps = maps
        # The boxes this program set, so restore() clears them all.
        self._boxes: set[str] = set()

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

    def set_gripper(self, travel: float) -> None:
        # The manipulation module takes the same 0 closed, 1 open travel and
        # maps it onto the gripper joint's own limits.
        self._motion(lambda: self._arm.set_gripper_position(travel))

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
                name = program_box(object_id)
                self._box_filter.set_box(
                    name,
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
                self._boxes.add(name)
                self._wait_for_map(f"a map without {object_id!r}")
                return
        raise ValueError(f"no object with id {object_id!r} in the scene")

    def restore(self) -> None:
        if not self._boxes:
            return
        for name in sorted(self._boxes):
            self._box_filter.clear_box(name)
        self._boxes.clear()
        self._wait_for_map("a map with the cleared objects back")

    def _wait_for_map(self, what: str) -> None:
        # Taken after the box filter changed, so any later capture reaches the planner with the change.
        changed = time.time()
        if not self._maps.wait_newer(changed, MAP_TIMEOUT_S):
            raise RobotCallError("TIMEOUT", f"waited {MAP_TIMEOUT_S} s for {what}")

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


def backend(run_dir: Path) -> RobotBackend:
    """Attach to the stack on this machine's bus and return its ``DimosBackend``:
    the one arm, the scene module, the map's box filter and the camera's
    ``color_image`` stream for keyframes. The scene module must run with
    ``target_frame="world"``. The connection closes when the process exits."""
    app = Dimos.connect()
    atexit.register(app.stop)
    cameras = [
        info
        for info in app.list_streams()
        if info.name == "color_image" and info.direction == "out"
    ]
    if len(cameras) != 1 or cameras[0].module_name is None:
        raise RuntimeError(
            f"expected one color_image stream, found {[c.module_name for c in cameras]}"
        )
    observer = StreamObserver(app.get_module(cameras[0].module_name).get_stream("color_image"))
    atexit.register(observer.close)
    arm = Arm.from_app(app)
    scene = app.find_module_by_spec(ObjectSceneSpec)
    box_filter = app.find_module_by_spec(PointCloudBoxFilterSpec)
    maps = MapStamps()
    atexit.register(subscribe_map(app, maps))
    return DimosBackend(arm, scene, box_filter, observer, maps)
