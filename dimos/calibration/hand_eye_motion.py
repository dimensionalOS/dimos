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

"""Move a running DimOS arm to calibration views and verify measured arrival."""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
from concurrent.futures import CancelledError
from contextlib import contextmanager
import math
from threading import Lock
from time import monotonic, time
from typing import TYPE_CHECKING

import numpy as np
from reactivex.disposable import Disposable

from dimos.calibration.hand_eye_capture import MAX_FRAME_AGE_SEC, TF_TOLERANCE_SEC
from dimos.core.introspection.module.info import ModuleInfo
from dimos.core.stream import Transport
from dimos.manipulation.manipulation_spec import ExecutionResult, ExecutionStatus, PlanResult
from dimos.manipulation.planning.groups.utils import filter_joint_state_to_selected_joints
from dimos.manipulation.sdk import Arm, MotionError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.porcelain.dimos import Dimos
from dimos.robot.manipulators.a1z.config import A1Z_HOME_TOLERANCE_RAD
from dimos.utils.timeseries.inmemory import InMemoryStore

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.Image import Image

_ANGLE_TOLERANCE = math.radians(2.0)
# Ring goals position the camera for a view. Calibration uses the measured mount pose.
_POSITION_TOLERANCE_M = 0.010


def _check_world_pose(pose: PoseStamped) -> None:
    if pose.frame_id != "world":
        raise ValueError(f"Expected a planner tool pose in world, got {pose.frame_id!r}.")
    if not np.isfinite(pose.position.to_tuple()).all() or not math.isclose(
        math.hypot(*pose.orientation.to_tuple()), 1.0, abs_tol=1e-5
    ):
        raise ValueError("Tool pose must have finite coordinates and a unit quaternion.")


class CalibrationArm:
    """Move one pose-capable group and pair each exposure with fresh joint feedback."""

    def __init__(self, arm: Arm, feedback: Transport[JointState]) -> None:
        self._arm = arm
        self.info = arm.info
        self.return_joints = (0.0,) * len(arm.info.joint_names)
        self.return_tolerance_rad = _ANGLE_TOLERANCE
        self._feedback = feedback
        self._required_joints = set(self.info.joint_names)
        self._reached_joints: np.ndarray | None = None

    def _positions(self, state: JointState) -> np.ndarray:
        # Connections may include gripper joints and use a different joint order.
        selected = filter_joint_state_to_selected_joints(state, self.info.joint_names)
        positions = np.asarray(selected.position, dtype=float)
        if not np.isfinite(positions).all():
            raise RuntimeError("Arm joint feedback contains non-finite positions.")
        return positions

    def _fresh_feedback(self) -> Iterator[JointState]:
        started, deadline = time(), monotonic() + 3.0
        while (remaining := deadline - monotonic()) > 0:
            try:
                state = self._feedback.get_next(timeout=remaining)
            except Exception:
                # get_next raises when its timeout expires.
                return
            if started <= state.ts <= time() and self._required_joints.issubset(state.name):
                yield state

    def read_joints(self) -> tuple[float, ...]:
        """Read fresh measured angles in planner joint order."""
        state = next(self._fresh_feedback(), None)
        if state is None:
            raise RuntimeError("No fresh arm joint feedback for the selected planning group.")
        return tuple(self._positions(state))

    def _wait_for_arrival(
        self,
        *,
        joints: Sequence[float] | None = None,
        target: PoseStamped | None = None,
        joint_tolerance_rad: float = _ANGLE_TOLERANCE,
    ) -> PoseStamped:
        reason = "No fresh arm joint feedback after the move."
        for raw in self._fresh_feedback():
            measured = self._positions(raw)
            if joints is not None and not np.allclose(
                measured, joints, rtol=0, atol=joint_tolerance_rad
            ):
                errors = np.abs(measured - joints)
                worst = int(np.argmax(errors))
                reason = (
                    f"Arm missed the requested joint positions. {self.info.joint_names[worst]} "
                    f"measured {measured[worst]:.4f} rad, requested {joints[worst]:.4f} rad, "
                    f"error {math.degrees(errors[worst]):.3f}° (limit {math.degrees(joint_tolerance_rad):.4g}°)."
                )
                continue
            snapshot = self._arm.state()
            pose = snapshot.end_effector_pose
            if (
                pose is None
                or snapshot.joints is None
                or not abs(pose.ts - raw.ts) <= TF_TOLERANCE_SEC
                or not np.allclose(
                    self._positions(snapshot.joints), measured, rtol=0, atol=_ANGLE_TOLERANCE
                )
            ):
                reason = "Planner tool pose does not match fresh measured arm feedback."
                continue
            _check_world_pose(pose)
            if target is not None:
                distance = pose.position.distance(target.position)
                angle = pose.orientation.angle_to(target.orientation)
                if distance > _POSITION_TOLERANCE_M or angle > _ANGLE_TOLERANCE:
                    reason = (
                        f"Arm missed the requested tool pose. Last measured position error "
                        f"{distance * 1000:.3f} mm (limit {_POSITION_TOLERANCE_M * 1000:g} mm), "
                        f"rotation error {math.degrees(angle):.3f}° "
                        f"(limit {math.degrees(_ANGLE_TOLERANCE):g}°)."
                    )
                    continue
            self._reached_joints = measured
            return pose
        raise RuntimeError(reason)

    @contextmanager
    def _planned_move(self) -> Iterator[tuple[float, ...]]:
        self._reached_joints = None
        measured = self.read_joints()
        try:
            yield measured
        except CancelledError:
            # Cancellation at the SDK's pre-execution check has no motion to stop.
            raise
        except (Exception, KeyboardInterrupt) as exc:
            # A plan or dispatch rejection leaves other clients' work alone.
            if isinstance(exc, MotionError) and (
                isinstance(exc.result, PlanResult)
                or (
                    isinstance(exc.result, ExecutionResult)
                    and exc.result.status in (ExecutionStatus.NO_PLAN, ExecutionStatus.REJECTED)
                )
            ):
                raise
            try:
                self._arm.cancel()
            except Exception as cancel_error:
                raise RuntimeError(
                    f"{exc}. Could not confirm the arm stopped: {cancel_error}"
                ) from exc
            raise

    def move_joints(self, joints: Sequence[float], *, returning: bool = False) -> PoseStamped:
        """Verify joint arrival, using the arm's parking tolerance for the final return."""
        if len(joints) != len(self.info.joint_names) or not all(math.isfinite(q) for q in joints):
            raise ValueError(
                f"Expected {len(self.info.joint_names)} finite joint positions "
                f"in planner order: {', '.join(self.info.joint_names)}."
            )
        with self._planned_move() as measured:
            if not np.allclose(measured, joints, rtol=0, atol=1e-3):
                self._arm.move_joints(joints, speed_scale=0.15, timeout=60.0)
            tolerance = self.return_tolerance_rad if returning else _ANGLE_TOLERANCE
            return self._wait_for_arrival(joints=joints, joint_tolerance_rad=tolerance)

    def move_pose(self, target: PoseStamped) -> PoseStamped:
        """Plan to a tool pose, then verify the measured result before capture."""
        _check_world_pose(target)
        with self._planned_move():
            self._arm.move_pose(
                target.position.to_tuple(),
                orientation=target.orientation.to_tuple(),
                speed_scale=0.15,
                timeout=60.0,
            )
            return self._wait_for_arrival(target=target)

    def capture_at_view(
        self, capture: Callable[[], tuple[Image, Transform]]
    ) -> tuple[Image, Transform]:
        """Verify the arm stayed near the reached posture at image time.

        Coordinator TF may restamp the last known positions if hardware feedback stops.
        Buffer direct readings while stationary capture waits for an image/TF pair.
        """
        if self._reached_joints is None:
            raise RuntimeError("Move to a calibration view before capturing.")
        readings: InMemoryStore[JointState] = InMemoryStore()
        lock = Lock()

        def observe(state: JointState) -> None:
            if math.isfinite(state.ts) and self._required_joints.issubset(state.name):
                with lock:
                    readings.save(state)
                    readings.prune_old(time() - MAX_FRAME_AGE_SEC - TF_TOLERANCE_SEC)

        with Disposable(self._feedback.subscribe(observe)):
            image, pose = capture()
        with lock:
            closest = readings.find_closest(image.ts, TF_TOLERANCE_SEC)
        if closest is None:
            raise RuntimeError("No fresh arm joint feedback within 150 ms of the image.")
        if not np.allclose(
            self._positions(closest), self._reached_joints, rtol=0, atol=_ANGLE_TOLERANCE
        ):
            raise RuntimeError("The measured arm was outside the 2 degree tolerance at image time.")
        return image, pose


@contextmanager
def connect_calibration_arm(
    *, check_cancelled: Callable[[], None] | None = None
) -> Iterator[CalibrationArm]:
    """Attach to the unique pose-capable group and its measured joint-state stream."""
    app = Dimos.connect()
    try:
        arm = Arm.from_app(app, before_execute=check_cancelled)
        sources = [
            stream.module_name
            for stream in app.list_streams()
            if stream.direction == "out"
            and stream.name == "joint_state"
            and stream.type_name == "JointState"
            and stream.module_name
        ]
        if len(sources) != 1:
            raise ValueError(
                f"Calibration needs one measured joint_state output on this arm stack. Found {sources}."
            )
        source = app.get_module(sources[0])
        connected = CalibrationArm(arm, source.get_stream("joint_state").transport)
        publisher = app.describe(source)
        if (
            isinstance(publisher, ModuleInfo)
            and publisher.qualified_path == "dimos.robot.manipulators.a1z.connection.A1ZConnection"
        ):
            connected.return_tolerance_rad = A1Z_HOME_TOLERANCE_RAD
            # Zero is J2's lower limit and J3's upper limit on the A1Z.
            offsets = {"arm_joint2": 0.05, "arm_joint3": -0.05}
            connected.return_joints = tuple(
                offsets.get(name.rsplit("/", 1)[-1], 0.0) for name in arm.info.joint_names
            )
        yield connected
    finally:
        app.stop()
