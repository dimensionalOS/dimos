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

from collections.abc import Iterator
import pickle
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.std_msgs.Header import Header

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

# A parallel jaw drawn in the TCP frame: approach along +Z, jaws closing along Y,
# origin at the fingertip plane. Sized for the xArm gripper.
_JAW_HALF_WIDTH = 0.0445
_FINGER_LENGTH = 0.052
_PALM_DEPTH = 0.04
# Drawing every proposal buries the ranking; the leaders are what matters.
_MAX_DRAWN = 20


class GraspCandidateArray:
    """Ordered grasp proposals sharing one input-cloud header."""

    msg_name = "manipulation_msgs.GraspCandidateArray"

    def __init__(
        self, header: Header | None = None, candidates: list[GraspCandidate] | None = None
    ) -> None:
        self.header = header if header is not None else Header(0.0)
        self.candidates = candidates if candidates is not None else []

    def __len__(self) -> int:
        return len(self.candidates)

    def __iter__(self) -> Iterator[GraspCandidate]:
        return iter(self.candidates)

    def encode(self) -> bytes:
        """Encode using the repository's pickle transport convention."""
        return pickle.dumps({"header": self.header, "candidates": self.candidates})

    @classmethod
    def decode(cls, data: bytes) -> GraspCandidateArray:
        value = pickle.loads(data)
        return cls(value["header"], value["candidates"])

    @staticmethod
    def _wireframe(candidate: GraspCandidate) -> list[list[float]]:
        """Five segments tracing wrist, palm and both fingers, in world."""
        w, fl, pd = _JAW_HALF_WIDTH, _FINGER_LENGTH, _PALM_DEPTH
        local = np.array(
            [
                [0.0, 0.0, -(pd + fl)],  # wrist
                [0.0, 0.0, -fl],  # palm centre
                [0.0, -w, -fl],  # right jaw root
                [0.0, w, -fl],  # left jaw root
                [0.0, -w, 0.25 * fl],  # right fingertip
                [0.0, w, 0.25 * fl],  # left fingertip
            ],
            dtype=np.float32,
        )
        pose = candidate.pose
        rotation = np.asarray(pose.orientation.to_rotation_matrix(), dtype=np.float32)
        origin = np.asarray([pose.position.x, pose.position.y, pose.position.z], dtype=np.float32)
        points = local @ rotation.T + origin
        strips = [(0, 1), (1, 2), (1, 3), (2, 4), (3, 5)]
        return [points[list(pair)].tolist() for pair in strips]

    @staticmethod
    def _rank_color(index: int, total: int) -> list[int]:
        """Best rank green, worst orange, so the ordering reads at a glance."""
        t = 0.0 if total <= 1 else index / (total - 1)
        return [int(60 + 195 * t), int(220 - 60 * t), 60]

    def to_rerun(self, **_: object) -> Archetype:
        """Draw the ranked proposals as gripper wireframes.

        Scores are only meaningful against each other, so rank drives the colour
        rather than the absolute value.
        """
        import rerun as rr

        drawn = self.candidates[:_MAX_DRAWN]
        strips: list[Any] = []
        colors: list[list[int]] = []
        labels: list[str] = []
        for index, candidate in enumerate(drawn):
            segments = self._wireframe(candidate)
            color = self._rank_color(index, len(drawn))
            strips.extend(segments)
            colors.extend([color] * len(segments))
            labels.extend([f"#{index} {candidate.score:.3f}"] + [""] * (len(segments) - 1))
        return rr.LineStrips3D(strips, colors=colors, labels=labels, radii=0.0015)
