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

"""Single owner, fixed simulation time, bounded input queue; no transport or renderer.

Only the owner calls advance/reset/place. Producers enqueue; no sensor or network
callback ever locks native dynamics. The module publishes copied snapshots later.
"""

import math
from queue import Empty, Queue
from typing import TypeAlias
import uuid

from dimos.sim2.types import Frame, JointEfforts, JointTargets, PhysicsBackend, WorldState, Wrench

Command: TypeAlias = JointTargets | JointEfforts | Wrench


class Runtime:
    def __init__(self, backend: PhysicsBackend, control_joints: tuple[str, ...] = ()) -> None:
        self.backend = backend
        self._control_joints = frozenset(control_joints)
        self.epoch = uuid.uuid4().hex
        self.tick = 0
        self._inbox: Queue[Command] = Queue(maxsize=256)
        self._expires: dict[str, float] = {}
        self._wrenches: list[tuple[float, Wrench]] = []
        self.rejected = 0
        self.last_error = ""
        self._command_ticks: dict[str, int] = {}

    def submit(self, command: Command) -> None:
        """Nonblocking ingress. Full queue is explicit backpressure, not silent loss."""
        self._inbox.put_nowait(command)

    def advance(self, steps: int = 1) -> None:
        for _ in range(steps):
            self.prepare()
            now = self.tick * self.backend.dt
            expired = tuple(name for name, until in self._expires.items() if until <= now)
            if expired:
                self.backend.hold(expired)
                for name in expired:
                    del self._expires[name]
            self._wrenches = [(until, w) for until, w in self._wrenches if until > now]
            self.backend.step(tuple(w for _, w in self._wrenches))
            self.tick += 1

    def prepare(self) -> None:
        """Apply queued commands without advancing time; used at control barriers."""
        now = self.tick * self.backend.dt
        for _ in range(256):
            try:
                command = self._inbox.get_nowait()
            except Empty:
                break
            try:
                if command.epoch != self.epoch:
                    raise ValueError("stale simulation epoch")
                if isinstance(command, JointTargets | JointEfforts):
                    if not math.isfinite(command.ttl) or not 0 < command.ttl <= 10:
                        raise ValueError("target ttl must be in (0, 10] seconds")
                    if command.tick is None and self._control_joints.intersection(command.joints):
                        raise ValueError("paced joints require a feedback tick")
                    if command.tick is not None and command.tick != self.tick:
                        raise ValueError("stale control tick")
                    if isinstance(command, JointTargets):
                        self.backend.targets(command)
                    else:
                        self.backend.efforts(command)
                    if command.tick is not None:
                        self._command_ticks.update((n, self.tick) for n in command.joints)
                    self._expires.update((name, now + command.ttl) for name in command.joints)
                else:
                    if not math.isfinite(command.duration) or not 0 < command.duration <= 10:
                        raise ValueError("wrench duration must be in (0, 10] seconds")
                    self.backend.validate_wrench(command)
                    if len(self._wrenches) >= 256:
                        raise ValueError("too many active wrenches")
                    self._wrenches.append((now + command.duration, command))
            except ValueError as error:
                self.rejected += 1
                self.last_error = str(error)

    def controlled(self, joints: tuple[str, ...]) -> bool:
        return all(self._command_ticks.get(name) == self.tick for name in joints)

    def snapshot(self) -> WorldState:
        return self.capture().state

    def capture(self) -> Frame:
        return self.backend.snapshot(self.epoch, self.tick)

    def reset(self) -> WorldState:
        self.backend.reset()
        self._invalidate()
        return self.snapshot()

    def place(self, body: str, pose: tuple[float, ...]) -> WorldState:
        self.backend.place(body, pose)
        self._invalidate()  # scene discontinuities invalidate commands and sensor histories
        return self.snapshot()

    def _invalidate(self) -> None:
        self.epoch = uuid.uuid4().hex
        self.tick = 0
        self._command_ticks.clear()
        self._expires.clear()
        self._wrenches.clear()
        self.backend.hold(self.backend.layout.drives)
        # Queued commands retain their old epoch and will be rejected on the next tick.
