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

"""Typed object manipulation RPCs shared by Python clients and agent tools."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum, auto
import json
from typing import Any, Protocol

from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.spec.utils import Spec


class PickPlaceStatus(Enum):
    """Outcome of a blocking scan, pick, or place operation."""

    SUCCEEDED = auto()
    INVALID_INPUT = auto()
    INVALID_STATE = auto()
    OBJECT_NOT_DETECTED = auto()
    PERCEPTION_FAILED = auto()
    GRASP_GENERATION_FAILED = auto()
    GRASP_FRAME_MISMATCH = auto()
    ROBOT_NOT_FOUND = auto()
    PLANNING_FAILED = auto()
    EXECUTION_FAILED = auto()
    GRIPPER_FAILED = auto()
    GRASP_VERIFICATION_FAILED = auto()


@dataclass(frozen=True)
class DetectedObject:
    """An object identified by the latest scan; its ID selects a pick target."""

    object_id: str
    name: str


@dataclass(frozen=True)
class PickPlaceResult:
    """Common outcome of an object-manipulation operation or internal step."""

    status: PickPlaceStatus
    message: str = ""

    @property
    def succeeded(self) -> bool:
        return self.status is PickPlaceStatus.SUCCEEDED

    def agent_encode(self) -> list[dict[str, Any]]:
        """Format the same typed result for MCP without changing the RPC value."""
        payload = asdict(self)
        payload["status"] = self.status.name
        payload["succeeded"] = self.succeeded
        return [{"type": "text", "text": json.dumps(payload)}]


@dataclass(frozen=True)
class ScanResult(PickPlaceResult):
    """Detected objects and the normalized prompts used to find them."""

    objects: tuple[DetectedObject, ...] = ()
    prompts: tuple[str, ...] = ()


@dataclass(frozen=True)
class PickResult(PickPlaceResult):
    """Pick outcome, including whether the workflow still holds an object.

    ``object_id`` is the requested target. A failed retract can leave
    ``holding_object`` true. Candidate diagnostics are absent until selected.
    """

    object_id: str = ""
    holding_object: bool = False
    rank: int | None = None
    score: float | None = None
    candidates: int = 0


@dataclass(frozen=True)
class PlaceResult(PickPlaceResult):
    """Place outcome; a failed retract after release leaves holding_object false."""

    holding_object: bool = False


class PickAndPlaceSpec(Spec, Protocol):
    """Blocking object manipulation on a deployed pick/place Module.

    Object IDs belong to the latest scan. Place coordinates are in the
    Module's configured planning frame, in metres. Transport errors raise;
    an RPC timeout does not cancel the operation.
    """

    def scan_objects(self, prompts: list[str]) -> ScanResult: ...

    def get_object(self, object_id: str) -> DetectedObject | None: ...

    def pick_object(
        self, object_id: str, planning_group: PlanningGroupID | None = None
    ) -> PickResult: ...

    def get_grasp_candidates(self) -> GraspCandidateArray: ...

    def place_at(
        self,
        x: float,
        y: float,
        z: float,
        planning_group: PlanningGroupID | None = None,
    ) -> PlaceResult: ...
