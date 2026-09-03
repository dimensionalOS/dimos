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

"""A complete recorded PoseStamped stream exposed as one timed Path."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path as FilePath
from typing import TYPE_CHECKING, ClassVar

from dimos.evals.types import Agent, RunningEnvironment

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


@dataclass
class PoseTrajectoryDataset:
    """Preserve a full odometry window as one agent-encodable path."""

    name: str
    stream: str = "odom"
    start_s: float | None = None
    stop_s: float | None = None

    artifacts: ClassVar[tuple[str, ...]] = ("recording",)
    has_robot: ClassVar[bool] = False
    _recording: Store | None = field(default=None, init=False, repr=False, compare=False)

    def preflight(self, agent: Agent) -> None:
        if agent.modules:
            raise RuntimeError(
                f"PoseTrajectoryDataset({self.name!r}) launches nothing; "
                f"{type(agent).__name__} adds modules {agent.modules!r}"
            )
        from dimos.memory.cli.dataset import open_dataset

        source = open_dataset(self.name)
        try:
            source.streams[self.stream].range_time(self.start_s, self.stop_s)
        finally:
            source.stop()

    def start(self, modules: str, trace_dir: FilePath | None = None) -> RunningEnvironment:
        from dimos.memory.cli.dataset import open_dataset, resolve_dataset
        from dimos.memory.store.memory import MemoryStore
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.nav_msgs.Path import Path

        source = open_dataset(self.name)
        try:
            observations = list(
                source.streams[self.stream].range_time(self.start_s, self.stop_s)
            )
            poses = [obs.data for obs in observations]
        finally:
            source.stop()
        if not poses:
            raise RuntimeError(f"pose stream {self.stream!r} has no observations in the window")
        if any(not isinstance(pose, PoseStamped) for pose in poses):
            raise TypeError(f"pose stream {self.stream!r} must contain PoseStamped messages")

        frames = {pose.frame_id for pose in poses}
        if len(frames) != 1:
            raise ValueError(f"pose stream {self.stream!r} changes frame: {sorted(frames)}")
        frame_id = frames.pop()
        trajectory = Path(ts=poses[0].ts, frame_id=frame_id, poses=poses)
        self._recording = MemoryStore()
        self._recording.stream("trajectory", Path).append(trajectory, ts=trajectory.ts)
        return RunningEnvironment(
            mcp_url="",
            recording=self._recording,
            artifacts={"recording": resolve_dataset(self.name)},
        )

    def settle(self, budget_s: float) -> None:
        return None

    def stop(self) -> None:
        if self._recording is not None:
            self._recording.stop()
            self._recording = None
