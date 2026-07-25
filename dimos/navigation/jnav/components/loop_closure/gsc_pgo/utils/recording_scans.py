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

"""Recording-specific stream + odom-edge resolution."""

from __future__ import annotations

from typing import Any

# (odom stream, lidar-fallback candidates). First pair whose odom stream exists wins, so a
# recording never mixes rigs (e.g. fastlio + pointlio). Ordered mid360 rig -> go2 -> generic.
STREAM_PAIRS: list[tuple[str, list[str]]] = [
    ("pointlio_odometry", ["pointlio_lidar"]),
    ("fastlio_odometry", ["fastlio_lidar"]),
    ("go2_odom", ["go2_lidar", "l1_lidar", "lidar"]),
    ("odom", ["lidar"]),
]


def resolve_streams(
    available: set[str] | list[str], odom: str = "", lidar: str = ""
) -> tuple[str, str]:
    """``(odom_stream, lidar_stream)`` defaults from what a recording actually has."""
    if not odom:
        odom = next((name for name, _ in STREAM_PAIRS if name in available), "odom")
    if not lidar:
        candidates = next((ls for name, ls in STREAM_PAIRS if name == odom), ["lidar"])
        lidar = next((name for name in candidates if name in available), candidates[0])
    return odom, lidar


def default_odom_edge(store: Any, odom_stream: str) -> str:
    """``"parent:child"`` from the odom stream's own header, or ``""`` if it has no child frame
    (e.g. ``PoseStamped`` odometry)."""
    observation = next(iter(store.stream(odom_stream)), None)
    if observation is None:
        return ""
    child_frame = getattr(observation.data, "child_frame_id", "")
    if not child_frame:
        return ""
    return f"{observation.data.frame_id}:{child_frame}"
