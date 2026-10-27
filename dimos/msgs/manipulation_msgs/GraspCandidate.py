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

from __future__ import annotations

import math
import pickle

from dimos.msgs.geometry_msgs.Pose import Pose


class GraspCandidate:
    """A robot TCP pose, its generator-local ranking score and an optional note on why it was
    proposed or what about it is uncertain."""

    msg_name = "manipulation_msgs.GraspCandidate"

    def __init__(self, pose: Pose | None = None, score: float = 0.0, note: str = "") -> None:
        self.pose = pose if pose is not None else Pose(0.0, 0.0, 0.0)
        self.score = float(score)
        self.note = note
        if not math.isfinite(self.score):
            raise ValueError("GraspCandidate.score must be finite")

    def encode(self) -> bytes:
        return pickle.dumps({"pose": self.pose, "score": self.score, "note": self.note})

    @classmethod
    def decode(cls, data: bytes) -> GraspCandidate:
        value = pickle.loads(data)
        return cls(value["pose"], value["score"], value.get("note", ""))
