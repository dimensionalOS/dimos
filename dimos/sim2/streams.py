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

"""Motor/state streams over DimOS Zenoh transport, with bounded local caches.

Only attachment and command ownership use RPC. Every physics tick and motor
command uses pub/sub; physics never waits for a consumer. A stopped publisher
is detected by feedback age, even if its final lifecycle message was lost.
"""

from __future__ import annotations

from collections import deque
from dataclasses import replace
import threading
import time
from typing import Any
from uuid import uuid4

import numpy as np

from dimos.core.transport import pZenohTransport
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.sim2.messages import FrameMetadata, Lifecycle, MotorFrame, StateFrame, StreamDescriptor


class SimStream:
    def __init__(self, descriptor: StreamDescriptor, *, owner: bool) -> None:
        self.descriptor = descriptor
        self._owner = owner
        self._lock = threading.RLock()
        self._history: deque[StateFrame] = deque(maxlen=descriptor.observation_slots)
        self._action: MotorFrame | None = None
        self._lifecycle: Lifecycle = "starting"
        self._episode = 0
        self._reset_at = 0.0
        self._sequence = 0
        self._action_sequence = 0
        self._writer: str | None = None
        self._writer_until = 0.0
        self._token = uuid4().hex
        self._closed = False
        self._state: pZenohTransport[StateFrame] = pZenohTransport(
            Topic(f"{descriptor.topic}/state", qos=QOS_LATEST_WINS)
        )
        self._commands: pZenohTransport[MotorFrame] = pZenohTransport(
            Topic(f"{descriptor.topic}/command", qos=QOS_LATEST_WINS)
        )
        self._rpc = ZenohRPC(default_rpc_timeout=2.0)
        try:
            self._rpc.start()
            self._state.start()
            self._commands.start()
            if owner:
                self._commands.subscribe(self._accept_command)
                self._rpc.serve_rpc(self._attach, f"{descriptor.topic}/attach")
                self._rpc.serve_rpc(self._release, f"{descriptor.topic}/release")
            else:
                self._state.subscribe(self._receive)
        except BaseException:
            self.close()
            raise

    @classmethod
    def create(cls, descriptor: StreamDescriptor) -> SimStream:
        return cls(descriptor, owner=True)

    @classmethod
    def attach(
        cls, descriptor: StreamDescriptor, *, writable: bool = False, timeout: float = 30
    ) -> SimStream:
        stream = cls(descriptor, owner=False)
        try:
            frames, unsub = stream._rpc.call_sync(
                f"{descriptor.topic}/attach",
                ([descriptor.fingerprint, stream._token if writable else None], {}),
                rpc_timeout=timeout,
            )
            unsub()
            for frame in frames:
                stream._receive(frame)
            stream._writer = stream._token if writable else None
            return stream
        except BaseException:
            stream.close()
            raise

    def _attach(self, fingerprint: str, writer: str | None) -> tuple[StateFrame, ...]:
        with self._lock:
            if fingerprint != self.descriptor.fingerprint:
                raise ValueError("sim2 stream descriptor mismatch")
            if self._lifecycle != "ready":
                raise RuntimeError(f"sim2 world is {self._lifecycle}")
            if writer is not None:
                if self._writer not in (None, writer) and time.monotonic() < self._writer_until:
                    raise RuntimeError(
                        f"robot {self.descriptor.robot_id!r} already has a command owner"
                    )
                if self._writer != writer:
                    self._action = None
                self._writer = writer
                # A client may load a policy after connecting, before its first command.
                self._writer_until = time.monotonic() + 30.0
            return tuple(self._history)

    def _release(self, writer: str) -> None:
        with self._lock:
            if self._writer == writer:
                self._writer = None
                self._action = None

    @property
    def lifecycle(self) -> Lifecycle:
        return self._lifecycle

    @property
    def episode_id(self) -> int:
        return self._episode

    def set_episode(self, episode_id: int) -> None:
        with self._lock:
            self._episode = episode_id
            self._reset_at = time.time()
            self._history.clear()
            self._action = None

    def set_lifecycle(self, state: Lifecycle) -> None:
        with self._lock:
            self._lifecycle = state
            frame = self._history[-1] if self._history else None
        if frame is not None:
            self.publish_observation(frame.values, frame.metadata)

    def _receive(self, frame: StateFrame) -> None:
        with self._lock:
            if self._history and frame.metadata.sequence <= self._history[-1].metadata.sequence:
                return
            if frame.metadata.episode_id != self._episode:
                self._history.clear()
            self._episode = frame.metadata.episode_id
            self._lifecycle = frame.lifecycle
            self._reset_at = frame.reset_at
            # Paused-world heartbeats are not additional motion samples.
            if (
                self._history
                and frame.metadata.physics_tick == self._history[-1].metadata.physics_tick
            ):
                self._history.pop()
            self._history.append(frame)

    def _accept_command(self, frame: MotorFrame) -> None:
        with self._lock:
            if (
                self._closed
                or self._lifecycle != "ready"
                or frame.writer != self._writer
                or frame.metadata.episode_id != self._episode
                or frame.source_ts < self._reset_at
                or not np.isfinite(frame.source_ts)
            ):
                return
            if (
                self._action is not None
                and frame.metadata.sequence <= self._action.metadata.sequence
            ):
                return
            values = frame.values
            for name in ("position", "velocity", "kp", "kd", "effort"):
                array = values.get(name)
                if (
                    array is None
                    or array.shape != (self.descriptor.dof,)
                    or not np.isfinite(array).all()
                ):
                    return
            if np.any(values["kp"] < 0) or np.any(values["kd"] < 0):
                return
            expires = values.get("expires_at")
            if expires is None or expires.shape != (1,) or not np.isfinite(expires).all():
                return
            if time.monotonic() > float(expires[0]):
                return
            if values.get("enabled", np.array([])).shape != (1,):
                return
            self._action = frame
            self._writer_until = float(expires[0])

    def publish_action(
        self, values: dict[str, Any], metadata: FrameMetadata, *, source_ts: float
    ) -> int:
        self._action_sequence += 1
        frame = MotorFrame(
            replace(metadata, sequence=self._action_sequence),
            {k: np.asarray(v, dtype=np.float64).copy() for k, v in values.items()},
            self._token,
            source_ts,
        )
        self._commands.broadcast(None, frame)
        return self._action_sequence

    def publish_observation(self, values: dict[str, Any], metadata: FrameMetadata) -> int:
        with self._lock:
            self._sequence += 1
            frame = StateFrame(
                replace(metadata, sequence=self._sequence),
                {k: np.asarray(v, dtype=np.float64).copy() for k, v in values.items()},
                self._lifecycle,
                self._reset_at,
            )
            self._receive(frame)
        self._state.broadcast(None, frame)
        return self._sequence

    def read_action(self) -> MotorFrame | None:
        with self._lock:
            return self._action

    def read_observation(self) -> StateFrame | None:
        with self._lock:
            return self._history[-1] if self._history else None

    def read_observations(self) -> tuple[StateFrame, ...]:
        with self._lock:
            return tuple(self._history)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if not self._owner and self._writer is not None:
            try:
                _, unsub = self._rpc.call_sync(
                    f"{self.descriptor.topic}/release", ([self._token], {}), rpc_timeout=0.5
                )
                unsub()
            except TimeoutError:
                pass  # A dead publisher's lease and last command expire independently.
        self._commands.stop()
        self._state.stop()
        self._rpc.stop()
