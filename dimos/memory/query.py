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

"""Read a recording store for the agent ``memory`` tool.

Hits report ``t`` in seconds from the first ``color_image`` frame, not Unix time.
``inspect`` opens the color frame only. Caption search is a JSONL dump plus
grep/head/tail, not a memory op, and inspect does not attach the 1 Hz caption.
Pass one ``t`` or up to four; several times become one labeled strip or 2x2 grid.
"""

from __future__ import annotations

from collections.abc import Sequence
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
from PIL import Image as PILImage
from PIL import ImageDraw

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

COLOR_STREAM = "color_image"
CAPTION_STREAM = "color_image_caption"
EMBEDDED_STREAM = "color_image_embedded"
CAPTIONS_DIR = "captions"
CAPTIONS_FILE = "captions.jsonl"
MANIFEST_FILE = "manifest.jsonl"
MAX_INSPECT_FRAMES = 4
_MONTAGE_CELL_H = 360
_MONTAGE_PAD = 4
_MONTAGE_LABEL_H = 28

Hit = dict[str, Any]
Result = dict[str, Any]
ManifestRow = dict[str, Any]


def _origin_ts(store: SqliteStore) -> float:
    if COLOR_STREAM in store.streams:
        return store.streams.color_image.first().ts
    if CAPTION_STREAM in store.streams:
        return store.streams.color_image_caption.first().ts
    raise LookupError(
        f"recording has no {COLOR_STREAM} or {CAPTION_STREAM} stream; "
        f"available: {store.list_streams()}"
    )


def _require_captions(store: SqliteStore) -> None:
    if CAPTION_STREAM not in store.streams:
        raise LookupError(
            f"recording has no {CAPTION_STREAM} stream; run `dimos mem caption` on this store"
        )


def _caption_text(obs: Observation[Any]) -> str:
    data = obs.data
    if isinstance(data, str):
        return data
    return json.dumps(data, ensure_ascii=False)


def _utc_date(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).date().isoformat()


def _caption_object(text: str) -> dict[str, Any]:
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        return {"caption": text}
    if isinstance(parsed, dict):
        return parsed
    return {"caption": text}


def _dump_row(t: float, caption: str) -> dict[str, Any]:
    body = {k: v for k, v in _caption_object(caption).items() if k != "t"}
    return {"t": round(float(t), 3), **body}


def _hit(
    obs: Observation[Any],
    origin: float,
    *,
    score: float | None,
    path: str | None,
) -> Hit:
    similarity = score
    if similarity is None:
        similarity = getattr(obs, "similarity", None)
    return {
        "t": round(float(obs.ts) - origin, 3),
        "id": int(obs.id),
        "score": None if similarity is None else round(float(similarity), 4),
        "path": path,
    }


def _save_frame(obs: Observation[Any], out_dir: Path, origin: float) -> str | None:
    data = obs.data
    if not isinstance(data, Image):
        return None
    out_dir.mkdir(parents=True, exist_ok=True)
    t = float(obs.ts) - origin
    path = out_dir / f"t{t:.3f}_id{obs.id}.jpg"
    if not data.save(str(path)):
        return None
    return str(path)


def _color_at(store: SqliteStore, t: float, *, tolerance: float = 1.0) -> Observation[Any] | None:
    if COLOR_STREAM not in store.streams:
        return None
    hits = store.streams.color_image.at_relative(t, tolerance=tolerance).to_list()
    return hits[-1] if hits else None


def _normalize_inspect_times(t: float | Sequence[float]) -> list[float]:
    if isinstance(t, (int, float)):
        return [float(t)]
    if isinstance(t, str) or not isinstance(t, Sequence):
        raise TypeError("t must be a number or a list of numbers")
    times = [float(x) for x in t]
    if not (1 <= len(times) <= MAX_INSPECT_FRAMES):
        raise ValueError(f"t must have 1..{MAX_INSPECT_FRAMES} values, got {len(times)}")
    return times


def _rgb_array(obs: Observation[Any]) -> np.ndarray:
    data = obs.data
    if not isinstance(data, Image):
        raise TypeError(f"color frame {obs.id} is not an Image")
    rgb = data if data.format == ImageFormat.RGB else data.to_rgb()
    arr = np.asarray(rgb.data)
    if arr.ndim != 3 or arr.shape[2] < 3:
        raise TypeError(f"color frame {obs.id} is not HxWx3")
    return np.ascontiguousarray(arr[:, :, :3])


def _montage_cell(rgb: np.ndarray, label: str) -> PILImage.Image:
    src = PILImage.fromarray(rgb.astype(np.uint8), mode="RGB")
    w, h = src.size
    scale = _MONTAGE_CELL_H / max(h, 1)
    cell_w = max(1, int(round(w * scale)))
    resized = src.resize((cell_w, _MONTAGE_CELL_H), PILImage.Resampling.BILINEAR)
    canvas = PILImage.new(
        "RGB", (cell_w, _MONTAGE_CELL_H + _MONTAGE_LABEL_H), color=(20, 20, 20)
    )
    canvas.paste(resized, (0, _MONTAGE_LABEL_H))
    draw = ImageDraw.Draw(canvas)
    draw.text((6, 6), label, fill=(255, 230, 80))
    return canvas


def _compose_montage(
    panels: Sequence[tuple[float, np.ndarray]], out_path: Path
) -> str:
    """Write a labeled strip (2–3 frames) or 2x2 grid (4 frames). Returns the path."""
    cells = [_montage_cell(rgb, f"t={t:.3f}") for t, rgb in panels]
    n = len(cells)
    if n <= 3:
        width = sum(c.size[0] for c in cells) + _MONTAGE_PAD * (n + 1)
        height = cells[0].size[1] + 2 * _MONTAGE_PAD
        sheet = PILImage.new("RGB", (width, height), color=(10, 10, 10))
        x = _MONTAGE_PAD
        for cell in cells:
            sheet.paste(cell, (x, _MONTAGE_PAD))
            x += cell.size[0] + _MONTAGE_PAD
    else:
        # 2x2
        cell_w = max(c.size[0] for c in cells)
        cell_h = cells[0].size[1]
        width = 2 * cell_w + 3 * _MONTAGE_PAD
        height = 2 * cell_h + 3 * _MONTAGE_PAD
        sheet = PILImage.new("RGB", (width, height), color=(10, 10, 10))
        for i, cell in enumerate(cells):
            row, col = divmod(i, 2)
            # center narrower cells in their slot
            x = _MONTAGE_PAD + col * (cell_w + _MONTAGE_PAD) + (cell_w - cell.size[0]) // 2
            y = _MONTAGE_PAD + row * (cell_h + _MONTAGE_PAD)
            sheet.paste(cell, (x, y))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path, format="JPEG", quality=90)
    return str(out_path)


def inspect(
    store: SqliteStore,
    t: float | Sequence[float],
    *,
    out_dir: Path | None = None,
    tolerance: float = 1.0,
) -> Result:
    """Color frame(s) nearest relative second ``t``.

    One ``t`` returns that frame. Two to four times return each hit plus a
    labeled montage (horizontal strip for 2–3, 2x2 for 4) under ``montage``.
    """
    if COLOR_STREAM not in store.streams:
        raise LookupError(f"recording has no {COLOR_STREAM} stream")
    times = _normalize_inspect_times(t)
    origin = _origin_ts(store)
    hits: list[Hit] = []
    panels: list[tuple[float, np.ndarray]] = []
    for want in times:
        obs = _color_at(store, want, tolerance=tolerance)
        if obs is None:
            raise LookupError(f"no {COLOR_STREAM} near t={want}")
        path = _save_frame(obs, out_dir, origin) if out_dir is not None else None
        hit = _hit(obs, origin, score=None, path=path)
        hits.append(hit)
        if out_dir is not None and len(times) > 1:
            panels.append((float(hit["t"]), _rgb_array(obs)))
    result: Result = {
        "ok": True,
        "op": "inspect",
        "origin_ts": origin,
        "stream": COLOR_STREAM,
        "hits": hits,
    }
    if out_dir is not None and len(panels) > 1:
        stamp = "_".join(f"{t:.3f}" for t, _ in panels)
        montage_path = out_dir / f"montage_{stamp}.jpg"
        result["montage"] = _compose_montage(panels, montage_path)
    return result


def dump_caption_jsonl(store: SqliteStore, dest: Path) -> list[ManifestRow]:
    """Write one JSONL file per UTC day under *dest*.

    ``captions/YYYY-MM-DD.jsonl`` always. ``captions/manifest.jsonl`` lists
    ``{date, t0, t1, n, path}``. A recording that stays on one UTC day also
    gets ``captions.jsonl`` at *dest* (same bytes as that day's file).
    """
    _require_captions(store)
    origin = _origin_ts(store)
    grouped: dict[str, list[dict[str, Any]]] = {}
    order: list[str] = []
    for obs in store.streams.color_image_caption:
        ts = float(obs.ts)
        day = _utc_date(ts)
        if day not in grouped:
            grouped[day] = []
            order.append(day)
        grouped[day].append(_dump_row(ts - origin, _caption_text(obs)))
    if not order:
        return []

    captions_dir = dest / CAPTIONS_DIR
    captions_dir.mkdir(parents=True, exist_ok=True)
    manifest: list[ManifestRow] = []
    for day in order:
        rows = grouped[day]
        rel = f"{CAPTIONS_DIR}/{day}.jsonl"
        path = dest / rel
        path.write_text(
            "".join(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n" for row in rows),
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
    (captions_dir / MANIFEST_FILE).write_text(
        "".join(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n" for row in manifest),
        encoding="utf-8",
    )
    if len(manifest) == 1:
        (dest / CAPTIONS_FILE).write_bytes((dest / manifest[0]["path"]).read_bytes())
    elif (dest / CAPTIONS_FILE).is_file():
        (dest / CAPTIONS_FILE).unlink()
    return manifest


def write_caption_stream(
    store: SqliteStore,
    rows: Sequence[tuple[float, str]],
    *,
    dest: str = CAPTION_STREAM,
) -> int:
    """Write ``(relative_t, caption)`` rows onto *dest*. ``relative_t`` is seconds from the first color frame."""
    if COLOR_STREAM not in store.streams:
        raise LookupError(f"recording has no {COLOR_STREAM} stream")
    if dest in store.streams and store.streams[dest].count() > 0:
        raise ValueError(f"{dest} already has observations")
    origin = _origin_ts(store)
    target = store.stream(dest, str)
    written = 0
    for t, caption in sorted(rows, key=lambda row: float(row[0])):
        target.append(caption, ts=origin + float(t))
        written += 1
    return written


def captions_from_jsonl(path: Path) -> list[tuple[float, str]]:
    """Load ``{t, caption|parsed|raw}`` lines into ``(relative_t, json_text)`` rows."""
    rows: list[tuple[float, str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        obj = json.loads(line)
        t = float(obj["t"])
        caption = _caption_from_row(obj)
        rows.append((t, caption))
    return rows


def _caption_from_row(obj: dict[str, Any]) -> str:
    if isinstance(obj.get("caption"), str) and obj["caption"].strip():
        return obj["caption"]
    parsed = obj.get("parsed")
    if parsed is not None:
        return parsed if isinstance(parsed, str) else json.dumps(parsed, ensure_ascii=False)
    raw = obj.get("raw")
    if isinstance(raw, str) and raw.strip():
        return raw
    body = {k: v for k, v in obj.items() if k != "t"}
    if body:
        return json.dumps(body, ensure_ascii=False)
    raise ValueError("jsonl row needs caption, parsed, raw, or caption fields")


def embed_color_stream(
    store: SqliteStore,
    model: Any,
    *,
    hz: float = 2.0,
    stream: str = COLOR_STREAM,
    dest: str = EMBEDDED_STREAM,
) -> int:
    """Write CLIP (or any EmbeddingModel) vectors for *stream* into *dest*. Returns frames written."""
    from dimos.memory.embed import EmbedImages
    from dimos.memory.transform import throttle

    if hz <= 0:
        raise ValueError("hz must be > 0")
    if stream not in store.streams:
        raise LookupError(f"no stream {stream!r}; available: {store.list_streams()}")
    if dest in store.streams and store.streams[dest].count() > 0:
        raise ValueError(f"{dest} already has observations")
    target = store.stream(dest, store.streams[stream].data_type)
    (
        store.streams[stream]
        .transform(throttle(1.0 / hz))
        .transform(EmbedImages(model))
        .save(target)
        .drain()
    )
    return target.count()


def handle_request(request: dict[str, Any]) -> Result:
    """Dispatch one JSON request from the harness worker."""
    op = request.get("op")
    path = request.get("path")
    if not isinstance(op, str) or not isinstance(path, str):
        raise TypeError("request needs string op and path")
    out = request.get("out_dir")
    out_dir = Path(out) if isinstance(out, str) and out else None
    with SqliteStore(path=path, must_exist=True) as store:
        if op == "inspect":
            if "t" not in request:
                raise TypeError("inspect needs t")
            return inspect(store, request["t"], out_dir=out_dir)
        if op == "dump":
            dest = Path(request["dest"]) if request.get("dest") else Path(path).parent
            return {"ok": True, "op": "dump", "files": dump_caption_jsonl(store, dest)}
    raise ValueError(f"unknown op {op!r}")


def main() -> None:
    """JSONL worker: one request object per stdin line, one result per stdout line."""
    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        try:
            result = handle_request(json.loads(line))
        except Exception as exc:
            result = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        sys.stdout.write(json.dumps(result) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
