# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise


@dataclass(frozen=True)
class PreviewFrame:
    """One timestamped local robot preview frame."""

    time_from_start: float
    positions: tuple[float, ...]


@dataclass(frozen=True)
class PreviewTrack:
    """One fixed-baseline local robot track in a group-native preview."""

    robot_id: str
    joint_names: tuple[str, ...]
    frames: tuple[PreviewFrame, ...]


@dataclass(frozen=True)
class GroupPreviewAnimation:
    """Validated collection of robot tracks sharing one preview transaction."""

    tracks: tuple[PreviewTrack, ...]


def scaled_frame_delays(frames: Sequence[PreviewFrame], duration: float) -> tuple[float, ...]:
    """Return stored inter-frame delays, optionally scaled to a requested duration."""
    if len(frames) < 2:
        return ()
    original_duration = max(float(frames[-1].time_from_start), 0.0)
    scale = duration / original_duration if duration > 0.0 and original_duration > 0.0 else 1.0
    return tuple(
        max(float(next_frame.time_from_start) - float(frame.time_from_start), 0.0) * scale
        for frame, next_frame in pairwise(frames)
    )


def preview_tick_times(preview: GroupPreviewAnimation) -> tuple[float, ...]:
    """Union all stored track timestamps without synthesizing extra samples."""
    return tuple(
        sorted({float(frame.time_from_start) for track in preview.tracks for frame in track.frames})
    )
