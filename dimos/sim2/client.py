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

"""Internal motor IO for a robot connection or policy host, not a DimOS Module."""

from __future__ import annotations

from collections.abc import Sequence
import math
import threading
import time
from typing import Any

import numpy as np

from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.messages import FrameMetadata, StateFrame, StreamDescriptor
from dimos.sim2.streams import SimStream


class SimRobotClient:
    """Attach using the world's descriptor; readers cannot accidentally command a robot.

    One writable client owns the complete actuator set. Commands name the observation
    they were computed from, so reset invalidates in-flight policy results as well as
    held targets. Joint order is descriptor.joint_names; IMU quaternions are WXYZ.
    """

    def __init__(
        self,
        descriptor: StreamDescriptor,
        *,
        writable: bool = False,
        feedback_timeout: float = 0.25,
        command_timeout: float = 0.25,
    ) -> None:
        if descriptor.dof == 0:
            raise ValueError("SimRobotClient requires a robot channel, not a world snapshot")
        if any(not math.isfinite(v) or v <= 0 for v in (feedback_timeout, command_timeout)):
            raise ValueError("client timeouts must be finite and positive")
        self.descriptor = descriptor
        self.feedback_timeout = feedback_timeout
        self.command_timeout = command_timeout
        self._channel: SimStream | None = None
        self._last_observation: StateFrame | None = None
        self._lock = threading.RLock()
        self._writable = writable

    def connect(self, timeout: float = 30.0) -> None:
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("connect timeout must be finite and nonnegative")
        with self._lock:
            if self._channel is not None:
                raise RuntimeError("sim2 client is already connected")
            try:
                self._channel = SimStream.attach(
                    self.descriptor, writable=self._writable, timeout=timeout
                )
                self.read()
            except BaseException:
                self.close()
                raise

    def _require_channel(self) -> SimStream:
        if self._channel is None:
            raise RuntimeError("sim2 client is disconnected")
        if self._channel.lifecycle != "ready":
            raise RuntimeError(f"sim2 robot is {self._channel.lifecycle}")
        return self._channel

    def _validate_observation(self, frame: StateFrame, channel: SimStream) -> None:
        if frame.metadata.episode_id != channel.episode_id:
            raise RuntimeError("sim2 robot was reset; read a new observation before commanding")
        age = time.monotonic() - float(frame.values["monotonic_time"][0])
        if not 0 <= age <= self.feedback_timeout:
            raise TimeoutError("sim2 robot feedback is stale")

    def read(self) -> StateFrame:
        """Read one coherent joint/IMU/base sample; never mix separate physics ticks."""
        with self._lock:
            channel = self._require_channel()
            frame = channel.read_observation()
            if frame is None:
                raise RuntimeError("sim2 robot has no observation")
            self._validate_observation(frame, channel)
            self._last_observation = frame
            return frame

    def write_motors(self, command: MotorCommandArray, *, observed: StateFrame) -> int:
        """Write complete PD/feedforward targets in descriptor joint order."""
        values = np.asarray([command.q, command.dq, command.kp, command.kd, command.tau])
        if values.shape != (5, self.descriptor.dof) or not np.isfinite(values).all():
            raise ValueError(f"expected {self.descriptor.dof} finite q/dq/kp/kd/tau values")
        if np.any(values[2:4] < 0):
            raise ValueError("motor gains must be nonnegative")
        position_servos = np.array([mode == "position" for mode in self.descriptor.actuator_modes])
        if np.any(values[1:, position_servos] != 0):
            raise ValueError(
                "native position servos accept positions, not velocity/gain/effort overrides"
            )
        q, dq, kp, kd, effort = values
        return self._write(
            dict(position=q, velocity=dq, kp=kp, kd=kd, effort=effort),
            observed,
            enabled=True,
            source_ts=command.timestamp,
        )

    def write_positions(
        self, positions: Sequence[float], *, observed: StateFrame, source_ts: float | None = None
    ) -> int:
        """Write native joint targets, including grippers in their declared units."""
        q = np.asarray(positions, dtype=np.float64)
        if q.shape != (self.descriptor.dof,) or not np.isfinite(q).all():
            raise ValueError(f"expected {self.descriptor.dof} finite joint positions")
        if any(
            mode == "effort" and gains[0] == 0
            for mode, gains in zip(
                self.descriptor.actuator_modes, self.descriptor.position_gains, strict=True
            )
        ):
            raise ValueError("position requests require a configured servo; send PD motor commands")
        kp, kd = np.asarray(self.descriptor.position_gains).T
        return self._write(
            dict(position=q, velocity=np.zeros_like(q), kp=kp, kd=kd, effort=np.zeros_like(q)),
            observed,
            enabled=True,
            source_ts=source_ts,
        )

    def disable(self, *, observed: StateFrame) -> int:
        """Disable effort outputs; native position servos hold their current position.

        This is device control, not a promise that a floating robot remains upright.
        """
        values = dict(
            position=observed.values["position"],
            velocity=np.zeros(self.descriptor.dof),
            effort=np.zeros(self.descriptor.dof),
        )
        values.update(kp=np.zeros(self.descriptor.dof), kd=np.zeros(self.descriptor.dof))
        return self._write(values, observed, enabled=False)

    def _write(
        self,
        values: dict[str, Any],
        observed: StateFrame,
        *,
        enabled: bool,
        source_ts: float | None = None,
    ) -> int:
        with self._lock:
            channel = self._require_channel()
            if not self._writable:
                raise PermissionError("sim2 client is read-only")
            if observed is not self._last_observation:
                raise ValueError("command must use this client's latest observation")
            self._validate_observation(observed, channel)
            issued = time.time() if source_ts is None else source_ts
            if not math.isfinite(issued) or issued < observed.reset_at:
                raise RuntimeError("sim2 command predates reset")
            remaining = self.command_timeout - max(0.0, time.time() - issued)
            if remaining <= 0:
                raise TimeoutError("sim2 command is stale")
            values.update(enabled=[int(enabled)], expires_at=[time.monotonic() + remaining])
            return channel.publish_action(
                values,
                FrameMetadata(
                    0,
                    observed.metadata.episode_id,
                    observed.metadata.physics_tick,
                    0,
                    observed.metadata.sim_time,
                ),
                source_ts=issued,
            )

    def close(self) -> None:
        with self._lock:
            if self._channel is not None:
                self._channel.close()
                self._channel = None
                self._last_observation = None

    def __enter__(self) -> SimRobotClient:
        self.connect()
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
