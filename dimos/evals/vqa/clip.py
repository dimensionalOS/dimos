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

"""Short dense color_image clips for memory-backed VQA.

Each clip is small enough that :class:`~dimos.evals.agents.question_answer.QuestionAnswer`
encodes **every** frame (``len(frames) ≤ frames_per_stream``). Questions state the
clip's relative ``[0, duration_s]`` window so the agent is not left to search a
long video that was sparsely subsampled.
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field

from dimos.msgs.sensor_msgs.Image import Image

# Match QuestionAnswerConfig.frames_per_stream default: never exceed this per clip.
DEFAULT_CLIP_MAX_FRAMES = 8
# Half-width around an anchor when building a short sighting/history clip.
DEFAULT_CLIP_RADIUS_S = 1.0


class ClipFrameEntry(BaseModel):
    """One frame file inside a clip directory."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    file: str
    ts: float


class ClipManifest(BaseModel):
    """On-disk clip index next to ``frame-*.png`` files."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    frames: tuple[ClipFrameEntry, ...]
    anchors: tuple[float, ...] = ()
    # Absolute recording times of the first/last kept frame (for audit).
    t_start: float = Field(description="absolute ts of first frame")
    t_end: float = Field(description="absolute ts of last frame")
    duration_s: float = Field(description="t_end - t_start (relative span the agent sees)")


def clip_duration_s(frames: Sequence[tuple[float, Image]]) -> float:
    if not frames:
        return 0.0
    return float(frames[-1][0] - frames[0][0])


def format_clip_span(duration_s: float) -> str:
    """Human span for prompts: relative seconds from the first shown frame."""
    return f"t=0.0s to t={duration_s:.1f}s"


def dense_clip_around(
    window_frames: Sequence[tuple[float, Image]],
    anchor_ts: float,
    *,
    radius_s: float = DEFAULT_CLIP_RADIUS_S,
    max_frames: int = DEFAULT_CLIP_MAX_FRAMES,
) -> tuple[tuple[tuple[float, Image], ...], float]:
    """Consecutive frames near ``anchor_ts``, capped so QA shows all of them.

    Picks every frame in ``[anchor - radius, anchor + radius]`` that exists in
    ``window_frames``. If that set is larger than ``max_frames``, keeps a
    consecutive run centered on the nearest-to-anchor frame (no long-range
    even subsample).
    """
    if max_frames < 1:
        raise ValueError("max_frames must be >= 1")
    if not window_frames:
        raise ValueError("clip window contains no frames")
    lo, hi = anchor_ts - radius_s, anchor_ts + radius_s
    in_radius = [(ts, img) for ts, img in window_frames if lo <= ts <= hi]
    if not in_radius:
        # Fall back to the single nearest frame in the provided window.
        nearest = min(window_frames, key=lambda row: abs(row[0] - anchor_ts))
        in_radius = [nearest]
    if len(in_radius) > max_frames:
        anchor_i = min(range(len(in_radius)), key=lambda i: abs(in_radius[i][0] - anchor_ts))
        half = max_frames // 2
        start = max(0, min(anchor_i - half, len(in_radius) - max_frames))
        in_radius = in_radius[start : start + max_frames]
    picked = tuple(in_radius)
    return picked, float(picked[0][0])


def ego_memory_clip(
    window_frames: Sequence[tuple[float, Image]],
    *,
    first_spot_ts: float,
    query_ts: float,
    max_frames: int = DEFAULT_CLIP_MAX_FRAMES,
) -> tuple[tuple[tuple[float, Image], ...], float]:
    """Frames covering first sighting of the referent through the question time.

    The agent must be able to have observed the object at least once before
    answering an ego-spatial question, so the clip spans
    ``[first_spot_ts, query_ts]`` (not a fixed pad ending at query alone).

    If the span has more frames than ``max_frames``, keeps both endpoints
    (nearest frames to first spot and query) and fills evenly between — never
    drops the first-spot or query anchors.
    """
    if max_frames < 1:
        raise ValueError("max_frames must be >= 1")
    if not window_frames:
        raise ValueError("clip window contains no frames")
    if query_ts < first_spot_ts:
        raise ValueError("query_ts must be >= first_spot_ts for ego memory clips")
    in_window = [
        (ts, img) for ts, img in window_frames if first_spot_ts - 1e-9 <= ts <= query_ts + 1e-9
    ]
    if not in_window:
        raise ValueError("no frames between first spot and query time")
    timestamps = [ts for ts, _ in in_window]
    if len(in_window) <= max_frames:
        picked = tuple(in_window)
        return picked, float(picked[0][0])
    # Bound length but never drop first-spot / query anchors.
    indices = _pick_with_anchors(
        timestamps,
        anchors=(first_spot_ts, query_ts),
        max_frames=max_frames,
    )
    picked = tuple(in_window[i] for i in indices)
    return picked, float(picked[0][0])


def _pick_with_anchors(
    timestamps: Sequence[float],
    anchors: Sequence[float],
    *,
    max_frames: int,
) -> tuple[int, ...]:
    """Nearest index per anchor, then even fill; result length ≤ max_frames."""
    n = len(timestamps)
    chosen: set[int] = set()
    for anchor in anchors:
        chosen.add(min(range(n), key=lambda i: abs(timestamps[i] - anchor)))
    if len(chosen) > max_frames:
        raise ValueError(f"ego clip needs {len(chosen)} anchor frames but max_frames={max_frames}")
    target = min(max_frames, n)
    if len(chosen) < target:
        even = [0] if target == 1 else [round(i * (n - 1) / (target - 1)) for i in range(target)]
        for idx in even:
            chosen.add(idx)
            if len(chosen) >= target:
                break
        if len(chosen) < target:
            for idx in range(n):
                chosen.add(idx)
                if len(chosen) >= target:
                    break
    return tuple(sorted(chosen))


def write_image_clip(
    clip_dir: Path,
    frames: Sequence[tuple[float, Image]],
    anchors: Sequence[float],
) -> ClipManifest:
    """Write ``manifest.json`` + PNGs under ``clip_dir``. Returns the manifest."""
    if not frames:
        raise ValueError("cannot write an empty clip")
    clip_dir.mkdir(parents=True, exist_ok=True)
    entries: list[ClipFrameEntry] = []
    for index, (ts, image) in enumerate(frames):
        name = f"frame-{index:03d}.png"
        path = clip_dir / name
        if not image.save(str(path)):
            raise RuntimeError(f"failed to write clip frame: {path}")
        entries.append(ClipFrameEntry(file=name, ts=float(ts)))
    t_start = float(entries[0].ts)
    t_end = float(entries[-1].ts)
    manifest = ClipManifest(
        frames=tuple(entries),
        anchors=tuple(float(a) for a in anchors),
        t_start=t_start,
        t_end=t_end,
        duration_s=t_end - t_start,
    )
    (clip_dir / "manifest.json").write_text(
        json.dumps(manifest.model_dump(mode="json"), indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def load_clip_manifest(clip_dir: Path) -> ClipManifest:
    """Load and validate ``clip_dir/manifest.json``."""
    path = clip_dir / "manifest.json"
    if not path.is_file():
        raise FileNotFoundError(f"clip manifest missing: {path}")
    return ClipManifest.model_validate_json(path.read_text(encoding="utf-8"))
