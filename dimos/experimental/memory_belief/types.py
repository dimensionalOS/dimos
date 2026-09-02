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

"""One grounded judgement about one target at one moment.

``visibility`` is three-way on purpose: ``absent`` is negative evidence -- the
sensor looked through the spot -- while ``occluded`` and ``out_of_view`` are no
evidence at all. Collapsing them turns "I did not see it" into "it is not there".

Capture pose is not ``target_pose``: the first lives on the enclosing
Observation, the second is None without geometric evidence. Keeping them apart
is what stops "the robot was in the kitchen" becoming "X is in the kitchen".

``valid_ts`` is when the fact held, ``observed_ts`` when the system worked it
out. Detection, association and attribute ``confidence`` fail independently and
cannot be one blended score.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from pydantic import ConfigDict

SCHEMA_VERSION = "1"
STREAM_NAME = "belief_observation"

Visibility = Literal["present", "absent", "occluded", "out_of_view"]
IdentityStatus = Literal["none", "tentative", "confirmed"]


@dataclass(frozen=True, slots=True)
class EvidenceRef:
    """A pointer back to one raw observation.

    ``stream`` plus ``observation_id`` is the only durable way to re-read the
    frame; ``ts`` is carried so a reader can order evidence without opening the
    store.
    """

    __pydantic_config__ = ConfigDict(extra="forbid")

    stream: str
    observation_id: int
    ts: float


@dataclass(frozen=True, slots=True)
class Confidence:
    """Separate confidences for separately-failing steps. None means N/A."""

    __pydantic_config__ = ConfigDict(extra="forbid")

    detection: float | None = None
    association: float | None = None
    attribute: float | None = None


@dataclass(frozen=True, slots=True)
class BeliefObservation:
    """What one look established about one target."""

    __pydantic_config__ = ConfigDict(extra="forbid")

    schema_version: str
    target_ref: str
    visibility: Visibility
    valid_ts: float
    observed_ts: float
    source: str
    evidence: tuple[EvidenceRef, ...]
    label: str | None = None
    #: Where the *target* is. Requires positional evidence -- depth, a fixed
    #: camera's known footprint, a presence sensor. A plain 2D detection cannot
    #: fill this in, and must not.
    place_ref: str | None = None
    #: Where the *sensor* was. Always knowable from odometry, and never a claim
    #: about the target: "the robot was in the kitchen when it saw a bottle" is
    #: not "there is a bottle in the kitchen".
    capture_place_ref: str | None = None
    target_pose: tuple[float, float, float] | None = None
    frame_id: str | None = None
    bbox: tuple[float, float, float, float] | None = None
    """Pixel box ``(x1, y1, x2, y2)`` in the frame ``evidence`` points at.

    Without it ``evidence`` names the frame a sighting came from but not where in
    the frame, which is enough to re-open the image and not enough to show anyone
    what was seen. Every claim this layer makes about the world traces back to a
    rectangle some detector drew; keeping the rectangle is what lets a person
    check the claim instead of trusting it."""
    identity_status: IdentityStatus | None = None
    identity_basis: str | None = None
    vocabulary: tuple[str, ...] | None = None
    confidence: Confidence = field(default_factory=Confidence)

    def to_rerun(self):  # type: ignore[no-untyped-def]
        """One sighting as a point, or nothing when it has no position.

        Replayed on a timeline these accumulate into the trail an object left
        behind. A sighting the locator refused shows as nothing rather than as a
        point at the sensor: drawing it at the camera would put every unplaced
        detection on the robot's own path, which is the exact conflation
        ``capture_place_ref`` exists to avoid.
        """
        import rerun as rr

        if self.target_pose is None:
            return rr.Clear(recursive=False)
        conf = float(self.confidence.detection or 0.0)
        # Faint at low confidence so a dense trail of weak detections reads as
        # weak rather than as a confident track.
        shade = int(60 + 195 * min(max(conf, 0.0), 1.0))
        return rr.Points3D(
            positions=[list(self.target_pose)],
            colors=[(shade, shade // 2, 255 - shade)],
            radii=[0.04],
            labels=[self.label or "?"],
        )

    def __post_init__(self) -> None:
        if not self.target_ref:
            raise ValueError("target_ref must be non-empty: a belief is about something")
        if not self.source:
            raise ValueError("source must be non-empty: oracle grade must stay attached")
        if not self.evidence:
            raise ValueError(
                f"{self.target_ref}: belief needs evidence. A record that cannot be traced "
                "back to a raw observation is an assertion, not a belief."
            )
        if (self.target_pose is None) != (self.frame_id is None):
            raise ValueError(
                f"{self.target_ref}: target_pose and frame_id go together. Coordinates "
                "without a declared frame cannot be compared with anything."
            )


def belief_tags(record: BeliefObservation) -> dict[str, Any]:
    """The indexed projection of a record.

    SQLite indexes every tag key it sees, so these are the fields worth pushing
    a query down on. The payload stays authoritative -- tags are a redundant
    copy, which is why callers never supply them by hand (see
    :func:`~dimos.experimental.memory_belief.write.append_belief`): a tag that disagreed with
    its payload would be undetectable at query time.
    """
    tags: dict[str, Any] = {
        "schema_version": record.schema_version,
        "target_ref": record.target_ref,
        "visibility": record.visibility,
        "source": record.source,
    }
    if record.place_ref is not None:
        tags["place_ref"] = record.place_ref
    if record.capture_place_ref is not None:
        tags["capture_place_ref"] = record.capture_place_ref
    if record.label is not None:
        tags["label"] = record.label
    if record.identity_status is not None:
        tags["identity_status"] = record.identity_status
    return tags
