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
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.hardware.whole_body.spec import IMUState
from dimos.sim2.types import Frame, JointTargets, Layout, WorldState


class SimRobotClient:
    def __init__(self, joints: tuple[str, ...], send: Callable[[JointTargets], None]) -> None:
        self.joints = joints
        self._send = send
        self._state: WorldState | None = None
        self._layout: Layout | None = None
        self._indices: list[int] = []
        self._retired_epochs: set[str] = set()

    def bind(self, layout: Layout) -> None:
        """Resolve joint indices once for this world's layout."""
        self._indices = [layout.joints.index(name) for name in self.joints]
        self._layout = layout

    def feedback(
        self, frame: Frame, robot: str, imu_frame: str, dt: float
    ) -> ControlFeedback | None:
        """Map simulator arrays to the existing task state, without running control."""
        layout, state = self._layout, frame.state
        if (
            layout is None
            or state.revision != layout.revision
            or state.epoch in self._retired_epochs
        ):
            return None
        if self._state is not None:
            if state.epoch == self._state.epoch and state.tick < self._state.tick:
                return None
            if state.epoch != self._state.epoch:
                self._retired_epochs.add(self._state.epoch)
        self.observe(state)
        quat = state.poses[layout.bodies.index(imu_frame), 3:]
        sample = frame.imu[layout.imu_frames.index(imu_frame)]
        return ControlFeedback(
            state.epoch,
            state.tick,
            CoordinatorState(
                joints=JointStateSnapshot(
                    joint_positions=dict(
                        zip(self.joints, state.position[self._indices].tolist(), strict=True)
                    ),
                    joint_velocities=dict(
                        zip(self.joints, state.velocity[self._indices].tolist(), strict=True)
                    ),
                    joint_efforts=dict(
                        zip(self.joints, state.effort[self._indices].tolist(), strict=True)
                    ),
                    timestamp=state.wall_time,
                ),
                imu={
                    robot: IMUState(
                        quaternion=(float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2])),
                        gyroscope=tuple(sample[3:]),
                        accelerometer=tuple(sample[:3]),
                    )
                },
                t_now=state.sim_time,
                dt=dt,
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
