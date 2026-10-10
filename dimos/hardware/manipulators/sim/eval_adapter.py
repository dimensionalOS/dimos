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

"""ManipulatorAdapter for the isolated evaluation runtime; never attaches SHM."""

from __future__ import annotations

import threading
import time
from typing import Any

import zenoh

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorInfo
from dimos.hardware.spec import JointLimits
from dimos.protocol.mujoco_eval import MAX_COMMAND_BYTES, Command, State, key, session_config


class MujocoEvalAdapter:
    def __init__(
        self, dof: int, address: str, run: str, episode: str, timeout_s: float = 30.0, **_: Any
    ) -> None:
        self._dof = dof
        self._endpoint, self._run, self._episode = address, run, episode
        self._timeout = timeout_s
        self._session: zenoh.Session | None = None
        self._subscriber: zenoh.Subscriber[None] | None = None
        self._state: State | None = None
        self._received = 0.0
        self._sequence = 0
        self._mode = ControlMode.POSITION
        self._condition = threading.Condition()

    def connect(self) -> bool:
        self._session = zenoh.open(
            session_config(self._endpoint, trusted=False, run=self._run, episode=self._episode)
        )
        self._subscriber = self._session.declare_subscriber(
            key(self._run, self._episode, "state"), self._on_state
        )
        with self._condition:
            ready = self._condition.wait_for(lambda: self._state is not None, self._timeout)
        if not ready:
            self.disconnect()
        return ready

    def _on_state(self, sample: zenoh.Sample) -> None:
        raw = sample.payload.to_bytes()
        if len(raw) > MAX_COMMAND_BYTES:
            return
        try:
            state = State.model_validate_json(raw)
        except ValueError:
            return
        if (state.run, state.episode) != (self._run, self._episode):
            return
        if any(
            len(v) != self._dof
            for v in (
                state.position,
                state.velocity,
                state.effort,
                state.lower,
                state.upper,
                state.velocity_max,
            )
        ):
            return
        with self._condition:
            if self._state is not None and state.sequence <= self._state.sequence:
                return
            self._state, self._received = state, time.monotonic()
            self._condition.notify_all()

    def disconnect(self) -> None:
        if self._session is not None:
            self._session.close()
        self._session = None
        self._subscriber = None
        self._state = None

    def is_connected(self) -> bool:
        return (
            self._session is not None
            and self._state is not None
            and time.monotonic() - self._received < 1.0
        )

    def has_live_state(self) -> bool:
        return self.is_connected()

    def _live_state(self) -> State:
        if not self.is_connected() or self._state is None:
            raise RuntimeError("No fresh trusted actuator state")
        return self._state

    def _send(self, kind: str, values: list[float] | None = None) -> bool:
        with self._condition:
            if not self.is_connected() or self._session is None:
                return False
            self._sequence += 1
            packet = Command.model_validate(
                dict(
                    run=self._run,
                    episode=self._episode,
                    sequence=self._sequence,
                    sent_at=time.time(),
                    kind=kind,
                    values=values or [],
                )
            )
            self._session.put(key(self._run, self._episode, "command"), packet.model_dump_json())
            return True

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        return self.write_enable(False)

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(vendor="Simulation", model="Isolated MuJoCo", dof=self._dof)

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        state = self._live_state()
        return JointLimits(
            position_lower=state.lower, position_upper=state.upper, velocity_max=state.velocity_max
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            return False
        self._mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._mode

    def read_joint_positions(self) -> list[float]:
        return self._live_state().position.copy()

    def read_joint_velocities(self) -> list[float]:
        return self._live_state().velocity.copy()

    def read_joint_efforts(self) -> list[float]:
        return self._live_state().effort.copy()

    def read_state(self) -> dict[str, int]:
        return {
            "state": int(any(abs(v) > 1e-4 for v in self.read_joint_velocities())),
            "mode": list(ControlMode).index(self._mode),
        }

    def read_error(self) -> tuple[int, str]:
        return (0, "") if self.is_connected() else (1, "No fresh trusted actuator state")

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        return self._send("position", positions)

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        return False

    def write_joint_efforts(self, efforts: list[float]) -> bool:
        return False

    def write_stop(self) -> bool:
        return self._send("stop")

    def write_enable(self, enable: bool) -> bool:
        return self._send("enable" if enable else "disable")

    def read_enabled(self) -> bool:
        return self._live_state().enabled

    def write_clear_errors(self) -> bool:
        return self.is_connected()

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None
