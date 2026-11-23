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

"""Write identity tracks next to a recording, same 1 Hz ticks as captions."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

from dimos.memory.query import CAPTION_STREAM, COLOR_STREAM, MANIFEST_FILE, ManifestRow
from dimos.memory.store.base import Store
from dimos.memory.type.observation import Observation
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox

TRACKS_DIR = "tracks"
TRACKS_FILE = "tracks.jsonl"

Update = Callable[[Image], Sequence[Detection2DBBox]]


def sidecar_tracks_jsonl(recording: Path) -> Path:
    """Precomputed tracks next to a recording: ``<parent>/tracks/<stem>/tracks.jsonl``."""
    path = Path(recording)
    return path.parent / TRACKS_DIR / path.stem / TRACKS_FILE


def _utc_date(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).date().isoformat()


def _origin_ts(store: Store) -> float:
    if COLOR_STREAM not in store.streams:
        raise LookupError(
            f"recording has no {COLOR_STREAM} stream; available: {store.list_streams()}"
        )
    return store.streams.color_image.first().ts


def dump_times(store: Store, origin: float, hz: float) -> list[float]:
    """Caption timestamps when present, otherwise one tick per *hz*."""
    if hz <= 0:
        raise ValueError("hz must be > 0")
    if CAPTION_STREAM in store.streams and store.streams.color_image_caption.count() > 0:
        return [float(obs.ts) - origin for obs in store.streams.color_image_caption]
    last = float(store.streams.color_image.order_by("ts", desc=True).first().ts) - origin
    n = int(last * hz) + 1
    return [i / hz for i in range(n)]


def _nearest_obs(
    prev: Observation[Any] | None, nxt: Observation[Any] | None, abs_t: float
) -> Observation[Any] | None:
    if prev is None:
        return nxt
    if nxt is None:
        return prev
    if abs(float(nxt.ts) - abs_t) < abs(float(prev.ts) - abs_t):
        return nxt
    return prev


def _append_row(
    grouped: dict[str, list[dict[str, Any]]],
    order: list[str],
    origin: float,
    t: float,
    detections: Sequence[Detection2DBBox],
) -> None:
    day = _utc_date(origin + t)
    if day not in grouped:
        grouped[day] = []
        order.append(day)
    grouped[day].append(track_row(t, detections))


def track_row(t: float, detections: Sequence[Detection2DBBox]) -> dict[str, Any]:
    tracks = [
        {
            "id": int(det.track_id),
            "name": det.name,
            "xyxy": [round(float(v), 1) for v in det.bbox],
        }
        for det in detections
        if det.track_id >= 0
    ]
    return {"t": round(float(t), 3), "tracks": tracks}


def dump_track_jsonl(
    store: Store,
    dest: Path,
    update: Update,
    *,
    hz: float = 1.0,
    until: float | None = None,
) -> list[ManifestRow]:
    """Run *update* once per caption/1 Hz tick on the nearest color frame."""
    origin = _origin_ts(store)
    targets = dump_times(store, origin, hz)
    if until is not None:
        targets = [t for t in targets if t <= until + 1e-6]
    if not targets:
        return []
    grouped: dict[str, list[dict[str, Any]]] = {}
    order: list[str] = []
    frames = iter(store.streams.color_image)
    current = next(frames, None)
    prev: Observation[Any] | None = None
    for t in targets:
        abs_t = origin + t
        while current is not None and float(current.ts) <= abs_t:
            prev = current
            current = next(frames, None)
        obs = _nearest_obs(prev, current, abs_t)
        image = obs.data if obs is not None else None
        detections: Sequence[Detection2DBBox] = (
            list(update(image)) if isinstance(image, Image) else []
        )
        _append_row(grouped, order, origin, t, detections)
    if not order:
        return []
    return _write_daily(dest, grouped, order)


def _write_daily(
    dest: Path, grouped: dict[str, list[dict[str, Any]]], order: list[str]
) -> list[ManifestRow]:
    tracks_dir = dest / TRACKS_DIR
    tracks_dir.mkdir(parents=True, exist_ok=True)
    manifest: list[ManifestRow] = []
    for day in order:
        rows = grouped[day]
        rel = f"{TRACKS_DIR}/{day}.jsonl"
        path = dest / rel
        path.write_text(
            "".join(
                json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n" for row in rows
            ),
            encoding="utf-8",
        )
        manifest.append(
            {
                "date": day,
                "t0": rows[0]["t"],
                "t1": rows[-1]["t"],
                "n": len(rows),
                "path": rel,
            }
        )
    (tracks_dir / MANIFEST_FILE).write_text(
        "".join(
            json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n" for row in manifest
        ),
        encoding="utf-8",
    )
    if len(manifest) == 1:
        (dest / TRACKS_FILE).write_bytes((dest / manifest[0]["path"]).read_bytes())
    elif (dest / TRACKS_FILE).is_file():
        (dest / TRACKS_FILE).unlink()
    return manifest
