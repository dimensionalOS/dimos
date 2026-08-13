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

from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.std_msgs.Header import Header


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
