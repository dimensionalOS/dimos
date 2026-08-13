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

"""Typed object-target observations for mobile-manipulation tasks.

The task contract is deliberately more explicit than a bare ``PoseStamped``:
consumers know which object was observed, where the measurement came from,
when it was observed, and how trustworthy it is. Providers remain replaceable
(sim ground truth today, localized perception on hardware later).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, TypeAlias

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

TargetObservationSource: TypeAlias = Literal["sim_ground_truth", "perception"]


@dataclass(frozen=True)
class TargetObservation:
    """One observation of a manipulation target.

    ``pose`` is required to be expressed in a stable world frame before it
    reaches a task. This keeps localization/perception transforms in the provider
    layer instead of leaking source-specific frame logic into task behavior.
    """

    object_id: str
    label: str
    pose: PoseStamped
    source: TargetObservationSource
    observed_at: float
    confidence: float = 1.0


def copy_pose_stamped(pose: PoseStamped) -> PoseStamped:
    """Copy a stamped pose without losing its frame id."""
    return PoseStamped(
        ts=float(pose.ts),
        frame_id=pose.frame_id,
        position=list(pose.position),
        orientation=list(pose.orientation),
    )
