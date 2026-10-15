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

"""A simulator's ground truth, written to a file only the grader reads.

The sandboxed agent sees the robot through the router; this log stays inside the robot
container's case mount, so grading can use the true pose and every contact without
the agent ever being able to read them.
"""

from __future__ import annotations

import json
from pathlib import Path
import threading
from typing import IO

from dim_go2_sim import Contacts
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.evals.sandbox.case import robot_case_dir
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class GroundTruthLogConfig(ModuleConfig):
    path: str = Field(default_factory=lambda: f"{robot_case_dir()}/ground_truth.jsonl")
    pose_hz: float = 10.0


class GroundTruthLog(Module):
    """JSON lines of the true base pose (at pose_hz) and of every change in contacts."""

    config: GroundTruthLogConfig

    ground_truth: In[PoseStamped]
    contacts: In[Contacts]

    _file: IO[str] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        Path(self.config.path).parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.config.path, "a", buffering=1)
        self._lock = threading.Lock()
        self._next_pose_ts = 0.0
        self.register_disposable(Disposable(self.ground_truth.subscribe(self._on_pose)))
        self.register_disposable(Disposable(self.contacts.subscribe(self._on_contacts)))

    @rpc
    def stop(self) -> None:
        super().stop()
        with self._lock:
            if self._file is not None:
                self._file.close()
                self._file = None

    def _write(self, record: dict[str, object]) -> None:
        with self._lock:
            if self._file is not None:
                self._file.write(json.dumps(record) + "\n")

    def _on_pose(self, pose: PoseStamped) -> None:
        if pose.ts < self._next_pose_ts:
            return
        self._next_pose_ts = pose.ts + 1.0 / self.config.pose_hz
        p, q = pose.position, pose.orientation
        self._write({"t": pose.ts, "pose": [p.x, p.y, p.z, q.x, q.y, q.z, q.w]})

    def _on_contacts(self, contacts: Contacts) -> None:
        self._write({"t": contacts.ts, "contacts": [[c.part, c.kind] for c in contacts.contacts]})
