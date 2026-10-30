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

"""Device-sized adapter owned by a robot connection, using existing DimOS streams."""

from collections.abc import Callable

from dimos.control.feedback import ControlFeedback, MotorControl
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.sim2.types import Frame, JointTargets, Layout, WorldState


class SimRobotClient:
    def __init__(
        self,
        joints: tuple[str, ...],
        send: Callable[[JointTargets], None],
        *,
        imu_frame: str | None = None,
        root_frame: str | None = None,
    ) -> None:
        self.joints = joints
        self._imu_frame, self._root_frame = imu_frame, root_frame
        self._imu_index: int | None = None
        self._imu_body: int | None = None
        self._root_index: int | None = None
        self._feedback: ControlFeedback | None = None
        self._send = send
        self._state: WorldState | None = None
        self._layout: Layout | None = None
        self._indices: list[int] = []
        self._retired_epochs: set[str] = set()

    def bind(self, layout: Layout) -> None:
        """Resolve joint indices once for this world's layout."""
        self._indices = [layout.joints.index(name) for name in self.joints]
        self._layout = layout
        if self._imu_frame is not None:
            self._imu_index = layout.imu_frames.index(self._imu_frame)
            self._imu_body = layout.bodies.index(self._imu_frame)
        if self._root_frame is not None:
            self._root_index = layout.bodies.index(self._root_frame)

    def feedback(self, frame: Frame) -> ControlFeedback | None:
        """Select this robot's measurements; clock and sample identity come from physics."""
        layout, state = self._layout, frame.state
        if (
            layout is None
            or state.revision != layout.revision
            or state.epoch in self._retired_epochs
        ):
            return None
        dt = frame.dt
        if self._state is not None:
            if state.epoch == self._state.epoch:
                if state.tick < self._state.tick:
                    return None
                if state.tick == self._state.tick:
                    return self._feedback
                dt = state.sim_time - self._state.sim_time
            else:
                self._retired_epochs.add(self._state.epoch)
        imu = None
        if self._imu_index is not None and self._imu_body is not None:
            assert self._imu_frame is not None
            sample = frame.imu[self._imu_index]
            imu = Imu(
                ts=state.wall_time,
                frame_id=self._imu_frame,
                orientation=Quaternion(*state.poses[self._imu_body, 3:]),
                linear_acceleration=Vector3(*sample[:3]),
                angular_velocity=Vector3(*sample[3:]),
            )
        self.observe(state)
        self._feedback = ControlFeedback(
            state.epoch,
            state.tick,
            state.sim_time,
            dt,
            JointState(
                ts=state.wall_time,
                name=list(self.joints),
                position=state.position[self._indices].tolist(),
                velocity=state.velocity[self._indices].tolist(),
                effort=state.effort[self._indices].tolist(),
            ),
            imu,
        )
        return self._feedback

    def odometry(self) -> Odometry | None:
        """Truth at the configured root, with body-relative twist for the standard message."""
        state = self._state
        if state is None or self._root_index is None:
            return None
        assert self._root_frame is not None
        pose, velocity = state.poses[self._root_index], state.twists[self._root_index]
        orientation = Quaternion(*pose[3:])
        inverse = orientation.conjugate()
        return Odometry(
            ts=state.wall_time,
            frame_id="world",
            child_frame_id=self._root_frame,
            pose=Pose(position=Vector3(*pose[:3]), orientation=orientation),
            twist=Twist(
                linear=inverse.rotate_vector(Vector3(*velocity[:3])),
                angular=inverse.rotate_vector(Vector3(*velocity[3:])),
            ),
        )

    def write_motors(self, command: MotorControl) -> bool:
        """Forward all servo fields with the identity of the computed sample.

        Late replies are discarded here and also checked by the physics owner.
        A reset while inference is running must never relabel an old command.
        """
        state = self._state
        if state is None or (command.epoch, command.tick) != (state.epoch, state.tick):
            return False
        if len(set(command.joints)) != len(command.joints) or set(command.joints) != set(
            self.joints
        ):
            raise ValueError("motor command must address this robot's joints exactly")
        motors = command.motors
        if len(command.joints) != motors.num_joints:
            raise ValueError("motor command names and values differ in length")
        self._send(
            JointTargets(
                command.epoch,
                command.joints,
                tuple(motors.q),
                tick=command.tick,
                velocities=tuple(motors.dq),
                kp=tuple(motors.kp),
                kd=tuple(motors.kd),
                feedforward=tuple(motors.tau),
            )
        )
        return True

    def observe(self, state: WorldState) -> None:
        self._state = state

    def write(self, positions: dict[str, float], ttl: float = 0.25, *, paced: bool = False) -> None:
        if self._state is None:
            raise RuntimeError("waiting for simulation feedback")
        if not set(positions) <= set(self.joints):
            raise ValueError("command addresses another robot")
        self._send(
            JointTargets(
                self._state.epoch,
                tuple(positions),
                tuple(positions.values()),
                ttl,
                self._state.tick if paced else None,
            )
        )
