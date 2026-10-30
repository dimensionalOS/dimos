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

from dimos.sim2.types import JointTargets, WorldState


class SimRobotClient:
    def __init__(self, joints: tuple[str, ...], send: Callable[[JointTargets], None]) -> None:
        self.joints = joints
        self._send = send
        self._state: WorldState | None = None

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
