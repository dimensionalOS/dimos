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

"""RGB/state snapshots with explicit acquisition or delivery timing for sonic78."""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from typing import Literal

import numpy as np
from numpy.typing import NDArray

ObservationTimeBasis = Literal["acquisition", "delivery"]
DELIVERY_COMPONENTS = ("camera", "body", "left_hand", "right_hand")


@dataclass(frozen=True)
class SonicDeliveryStamp:
    """Original same-host receiver time and opaque source delivery marker.

    Markers must advance within one source lifetime. DDS publication stamps
    and camera stamps are delivery evidence, not sensor acquisition identity.
    """

    source_id: str
    source_marker: int | float
    received_at: float

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if not isinstance(self.source_id, str) or not 0 < len(self.source_id.strip()) <= 512:
            raise ValueError("delivery source identity required")
        for value in (self.source_marker, self.received_at):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("finite delivery marker and receipt time required")


_ARM_NAMES = (
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
)
POLICY_JOINT_NAMES = (
    *(f"{side}_{joint}_joint" for side in ("left", "right") for joint in _ARM_NAMES),
    *(
        f"left_hand_{joint}_joint"
        for joint in ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
    ),
    *(
        f"right_hand_{joint}_joint"
        for joint in ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1")
    ),
)


def policy_joint_state(positions: Mapping[str, float]) -> NDArray[np.float32]:
    """Order named measured joints as the WBT dataset, including Dex3 fingers."""
    try:
        values = np.asarray([positions[name] for name in POLICY_JOINT_NAMES], dtype=np.float64)
    except KeyError as exc:
        raise ValueError(f"missing policy joint: {exc.args[0]}") from exc
    if not np.isfinite(values).all() or np.max(np.abs(values)) > np.finfo(np.float32).max:
        raise ValueError("joint values must be finite float32-compatible numbers")
    return values.astype(np.float32)


@dataclass(frozen=True)
class SonicObservation:
    """One snapshot; observed_at uses the session's local monotonic clock.

    Acquisition mode preserves paired source capture time. Delivery mode
    preserves the oldest of four original receiver times and requires its own
    service/execution profile; sensor acquisition age and skew remain unknown.
    Later queues or inference must never refresh either basis. Cross-host
    monotonic clock mapping and multi-frame histories are outside this profile.
    """

    sequence: int
    observed_at: float
    state: NDArray[np.float32]
    rgb: NDArray[np.uint8]
    time_basis: ObservationTimeBasis = "acquisition"
    delivery_stamps: tuple[SonicDeliveryStamp, ...] | None = None

    def __post_init__(self) -> None:
        if (
            type(self.sequence) is not int
            or self.sequence < 0
            or not math.isfinite(self.observed_at)
        ):
            raise ValueError("invalid observation sequence or source timestamp")
        if self.time_basis not in ("acquisition", "delivery"):
            raise ValueError("invalid observation time basis")
        if self.time_basis == "acquisition":
            if self.delivery_stamps is not None:
                raise ValueError("delivery provenance cannot be labeled acquisition")
        else:
            if not isinstance(self.delivery_stamps, tuple) or len(self.delivery_stamps) != 4:
                raise ValueError("delivery observations require four component receipts")
            for stamp in self.delivery_stamps:
                if not isinstance(stamp, SonicDeliveryStamp):
                    raise ValueError("invalid component receipt")
                stamp.validate()
            if self.observed_at != min(s.received_at for s in self.delivery_stamps):
                raise ValueError("observed_at must preserve the oldest component receipt")
        state, rgb = np.asarray(self.state), np.asarray(self.rgb)
        if (
            state.shape != (28,)
            or state.dtype.kind not in "fiu"
            or not np.isfinite(state).all()
            or np.max(np.abs(state)) > np.finfo(np.float32).max
        ):
            raise ValueError("state must be 28 finite float32-compatible measured joints")
        if rgb.shape != (480, 640, 3) or rgb.dtype != np.uint8:
            raise ValueError("RGB must be uint8 [480, 640, 3]")
        state, rgb = state.astype(np.float32, copy=True), rgb.copy()
        state.setflags(write=False)
        rgb.setflags(write=False)
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "rgb", rgb)

    def require_fresh(
        self,
        *,
        now: float,
        after_sequence: int = -1,
        after_observed_at: float = -math.inf,
        max_age: float = 0.2,
    ) -> None:
        """Reject stale/future samples and replayed sequence numbers/timestamps."""
        if not math.isfinite(now) or not math.isfinite(max_age) or max_age <= 0:
            raise ValueError("finite monotonic time and positive age bound required")
        if not 0 <= now - self.observed_at < max_age:
            raise ValueError("observation stale or from the future")
        if self.delivery_stamps is not None and any(
            not 0 <= now - stamp.received_at < max_age for stamp in self.delivery_stamps
        ):
            raise ValueError("component receipt stale or from the future")
        if self.sequence <= after_sequence or self.observed_at <= after_observed_at:
            raise ValueError("observation must be newer than the previous snapshot")
