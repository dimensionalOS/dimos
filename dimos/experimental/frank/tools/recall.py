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

"""What FRANK has seen and where he has been: the run's recording, queried.

    uv run python dimos/experimental/frank/tools/recall.py streams                 # what is recorded, since when
    uv run python dimos/experimental/frank/tools/recall.py recent --seconds 60     # the last minute as one picture: a frame per heading you faced
    uv run python dimos/experimental/frank/tools/recall.py near 2.0 -1.5 --radius 2  # what you saw when you were at that spot (last 30 min)
    uv run python dimos/experimental/frank/tools/recall.py at 15:09:20             # the frame and pose at that wall-clock time
    uv run python dimos/experimental/frank/tools/recall.py trail --minutes 10      # where you have been, drawn on the map
    uv run python dimos/experimental/frank/tools/recall.py find "a staircase"      # CLIP search over the last minutes of frames

Every picture lands in cache/recall.jpg (or --out) with a caption per tile: time, world pose, and
the compass direction the camera faced. The text printed says the same in words, so read both.

The stack records its streams to `recordings/<run-id>/memory.db` (DimOS `--record`; up.py turns it
on). This reads the newest one, or `--db PATH`. It is the DimOS memory API, `dimos.memory`:
`SqliteStore(path).streams.<name>` with `.after(t)`, `.at(t, tolerance)`, `.limit(k)`,
`.to_list()`, `.first()`, `.last()`; `obs.ts`, `obs.data` (Image, PoseStamped, PointCloud2).
Frames carry no pose of their own; odometry at the same timestamp gives it. Read-only.
"""

from __future__ import annotations

import argparse
import bisect
import datetime as dt
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import grid

from dimos.constants import RECORDINGS_DIR
from dimos.memory.store.sqlite import SqliteStore

CACHE = Path(__file__).resolve().parents[1] / "cache"
TILE = (426, 240)  # a 1280x720 frame at a third
COMPASS = ["east", "north-east", "north", "north-west", "west", "south-west", "south", "south-east"]
NOW: float | None = None  # set by --now to query an old recording as if it were live


def now() -> float:
    return NOW if NOW is not None else time.time()


# --- the recording -----------------------------------------------------------------


def newest_db() -> Path | None:
    dbs = sorted(RECORDINGS_DIR.glob("*/memory.db"), key=lambda p: p.stat().st_mtime)
    return dbs[-1] if dbs else None


def open_store(db: str | None = None) -> SqliteStore:
    path = Path(db) if db else newest_db()
    if path is None or not path.exists():
        raise SystemExit(
            "no recording: the stack is not running with --record (up.py turns it on), so there is no memory to query"
        )
    return SqliteStore(path=path)


def compass(yaw_deg: float) -> str:
    return COMPASS[int(((yaw_deg + 22.5) % 360.0) // 45.0)]


def clock(ts: float) -> str:
    return dt.datetime.fromtimestamp(ts).strftime("%H:%M:%S")


def _yaw(data: Any) -> float:
    yaw = float(data.yaw)
    return math.degrees(yaw) if abs(yaw) < 7 else yaw


class Track:
    """Odometry over a time window, for looking up where the robot was at any instant."""

    def __init__(self, store: SqliteStore, t0: float, t1: float | None = None) -> None:
        q = store.streams.odom.after(t0)
        if t1 is not None:
            q = q.before(t1)
        rows = q.to_list()
        self.ts = [o.ts for o in rows]
        self.xy = [(float(o.data.position.x), float(o.data.position.y)) for o in rows]
        self.yaw = [_yaw(o.data) for o in rows]

    def __len__(self) -> int:
        return len(self.ts)

    def at(self, ts: float) -> tuple[float, float, float] | None:
        """(x, y, yaw) nearest in time, or None if nothing within a second."""
        if not self.ts:
            return None
        i = bisect.bisect_left(self.ts, ts)
        best = min(
            (j for j in (i - 1, i) if 0 <= j < len(self.ts)), key=lambda j: abs(self.ts[j] - ts)
        )
        if abs(self.ts[best] - ts) > 1.0:
            return None
        return (*self.xy[best], self.yaw[best])

    def within(self, x: float, y: float, radius: float) -> list[tuple[float, float]]:
        """Time spans [t0, t1] during which the robot was within radius of (x, y)."""
        spans: list[tuple[float, float]] = []
        inside = False
        for ts, (px, py) in zip(self.ts, self.xy, strict=True):
            near = math.hypot(px - x, py - y) <= radius
            if near and not inside:
                spans.append((ts, ts))
            elif near:
                spans[-1] = (spans[-1][0], ts)
            inside = near
        return spans


# --- choosing and drawing frames ---------------------------------------------------


def pick(
    frames: list[Any],
    track: Track,
    n: int,
    yaw_step: float = 30.0,
    move_step: float = 0.5,
    every_s: float = 30.0,
) -> list[tuple[Any, tuple[float, float, float]]]:
    """Frames worth showing: a new one whenever the heading or the place has moved on (or every
    half minute while standing still), and always the latest one."""
    kept: list[tuple[Any, tuple[float, float, float]]] = []
    last: tuple[float, float, float, float] | None = None  # ts, x, y, yaw
    for i, f in enumerate(frames):
        p = track.at(f.ts)
        if p is None:
            continue
        x, y, yaw = p
        if last is not None and i < len(frames) - 1:
            dyaw = abs((yaw - last[3] + 180.0) % 360.0 - 180.0)
            if (
                dyaw < yaw_step
                and math.hypot(x - last[1], y - last[2]) < move_step
                and f.ts - last[0] < every_s
            ):
                continue
        kept.append((f, p))
        last = (f.ts, x, y, yaw)
    if len(kept) > n:
        idx = np.linspace(0, len(kept) - 1, n).round().astype(int)
        kept = [kept[i] for i in idx]
    return kept


def mosaic(tiles: list[tuple[np.ndarray, str]], out: str, cols: int = 3) -> str:
    """Thumbnails in a grid, each with its caption on a dark strip. Returns the path."""
    if not tiles:
        raise SystemExit("no frames to show")
    w, h = TILE
    strip = 22
    rows = math.ceil(len(tiles) / cols)
    canvas = np.full((rows * (h + strip), cols * w, 3), 30, dtype=np.uint8)
    for i, (img, caption) in enumerate(tiles):
        r, c = divmod(i, cols)
        y0, x0 = r * (h + strip), c * w
        canvas[y0 + strip : y0 + strip + h, x0 : x0 + w] = cv2.resize(img, (w, h))
        cv2.putText(
            canvas, caption, (x0 + 4, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1
        )
    cv2.imwrite(out, canvas)
    return out


def caption(i: int, ts: float, pose: tuple[float, float, float]) -> str:
    x, y, yaw = pose
    return f"{i} {clock(ts)} x={x:.1f} y={y:.1f} yaw={yaw:.0f} facing {compass(yaw)}"


def show(picked: list[tuple[Any, tuple[float, float, float]]], out: str, what: str) -> str:
    tiles, lines = [], []
    for i, (f, pose) in enumerate(picked, 1):
        cap = caption(i, f.ts, pose)
        tiles.append((f.data.to_opencv(), cap))
        lines.append(cap)
    mosaic(tiles, out)
    return "\n".join(
        [
            f"{out}: {len(tiles)} frames, {what}; captions are time, world pose, and the way the camera faced",
            *lines,
        ]
    )


# --- commands ----------------------------------------------------------------------


def streams(store: SqliteStore) -> str:
    lines = []
    for name in store.list_streams():
        s = store.streams[name]
        try:
            t0, t1 = s.get_time_range()
            lines.append(
                f"{name}: {s.count()} items, {clock(t0)} to {clock(t1)} ({(t1 - t0) / 60:.1f} min)"
            )
        except Exception:
            lines.append(f"{name}: empty")
    return "\n".join(lines) or "nothing recorded yet"


def recent(store: SqliteStore, seconds: float, out: str, n: int = 8) -> str:
    now_ = now()
    frames = store.streams.color_image.after(now_ - seconds).to_list()
    if not frames:
        raise SystemExit(f"no frames in the last {seconds:.0f} s; is the stack recording?")
    track = Track(store, now_ - seconds - 2)
    picked = pick(frames, track, n)
    return show(picked, out, f"the last {seconds:.0f} s, one per heading or place")


def near(
    store: SqliteStore, x: float, y: float, radius: float, minutes: float, out: str, n: int = 8
) -> str:
    now_ = now()
    track = Track(store, now_ - minutes * 60)
    spans = track.within(x, y, radius)
    if not spans:
        return f"you have not been within {radius:.1f} m of x={x:.1f} y={y:.1f} in the last {minutes:.0f} min"
    frames: list[Any] = []
    for t0, t1 in spans:
        frames += store.streams.color_image.after(t0 - 0.5).before(t1 + 0.5).to_list()
    picked = pick(frames, track, n)
    visits = ", ".join(f"{clock(a)}-{clock(b)}" for a, b in spans[-6:])
    return show(
        picked, out, f"seen within {radius:.1f} m of x={x:.1f} y={y:.1f}; there at {visits}"
    )


def at(store: SqliteStore, when: str, out: str) -> str:
    today = dt.datetime.fromtimestamp(now()).replace(second=0, microsecond=0)
    hh, mm, ss = (int(p) for p in when.split(":"))
    ts = today.replace(hour=hh, minute=mm, second=ss).timestamp()
    f = store.streams.color_image.at(ts, tolerance=2.0).first()
    if f is None:
        raise SystemExit(f"no frame within 2 s of {when}")
    pose = Track(store, ts - 2, ts + 2).at(f.ts)
    cap = caption(1, f.ts, pose) if pose else f"1 {clock(f.ts)} (no pose recorded)"
    mosaic([(f.data.to_opencv(), cap)], out, cols=1)
    return f"{out}: {cap}"


def trail(store: SqliteStore, minutes: float, out: str) -> str:
    now_ = now()
    track = Track(store, now_ - minutes * 60)
    if not track.ts:
        raise SystemExit(f"no odometry in the last {minutes:.0f} min")
    step = max(1, len(track) // 400)
    points = track.xy[::step]
    try:
        grid.load().render(out, trail=points)
        on = "on the map"
    except Exception:
        _trail_only(points, out)  # no live map on the bus: the path alone, on a grid
        on = "alone (no live map to draw it on)"
    marks = []
    for ts, (x, y), yaw in zip(
        track.ts[:: max(1, len(track) // 10)],
        track.xy[:: max(1, len(track) // 10)],
        track.yaw[:: max(1, len(track) // 10)],
        strict=False,
    ):
        marks.append(f"{clock(ts)} x={x:.1f} y={y:.1f} facing {compass(yaw)}")
    dist = sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(track.xy, track.xy[1:], strict=False)
    )
    return "\n".join(
        [
            f"{out}: your path over the last {minutes:.0f} min in blue {on}, {dist:.1f} m walked, oldest to newest:",
            *marks,
        ]
    )


def _trail_only(points: list[tuple[float, float]], out: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 7), dpi=120)
    ax.plot([p[0] for p in points], [p[1] for p in points], color="#2266cc", lw=1.4)
    ax.plot(points[0][0], points[0][1], "o", color="#2266cc", ms=6, label="start")
    ax.plot(points[-1][0], points[-1][1], "s", color="#ee7700", ms=7, label="now")
    ax.set_aspect("equal")
    ax.grid(color="#4488cc", lw=0.4, alpha=0.4)
    ax.set_xlabel("world x (m, east)")
    ax.set_ylabel("world y (m, north)")
    ax.legend()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)


def _vec(embedding: Any) -> np.ndarray:
    v = getattr(embedding, "vector", embedding)
    if hasattr(v, "detach"):
        v = v.detach().cpu().numpy()
    return np.asarray(v, dtype=np.float32).ravel()


def find(store: SqliteStore, text: str, minutes: float, out: str, k: int = 6) -> str:
    from dimos.models.embedding.clip import CLIPModel

    now_ = now()
    frames = store.streams.color_image.after(now_ - minutes * 60).to_list()
    if not frames:
        raise SystemExit(f"no frames in the last {minutes:.0f} min")
    step = max(1, int(len(frames) / (minutes * 30)))  # about one frame every 2 s
    frames = frames[::step]
    clip = CLIPModel()
    query = _vec(clip.embed_text(text))
    query /= np.linalg.norm(query) + 1e-9
    scores = []
    for i in range(0, len(frames), 16):
        batch = frames[i : i + 16]
        embs = clip.embed(*[f.data for f in batch])
        if not isinstance(embs, list):
            embs = [embs]
        for f, e in zip(batch, embs, strict=False):
            v = _vec(e)
            scores.append((float(v @ query / (np.linalg.norm(v) + 1e-9)), f))
    scores.sort(key=lambda s: -s[0])
    track = Track(store, now_ - minutes * 60 - 2)
    tiles, lines = [], []
    for i, (score, f) in enumerate(scores[:k], 1):
        pose = track.at(f.ts) or (float("nan"), float("nan"), 0.0)
        cap = f"{score:.2f} " + caption(i, f.ts, pose)
        tiles.append((f.data.to_opencv(), cap))
        lines.append(cap)
    mosaic(tiles, out)
    return "\n".join(
        [
            f"{out}: best {len(tiles)} of {len(frames)} frames from the last {minutes:.0f} min for '{text}', score first; look before you believe it",
            *lines,
        ]
    )


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--db", help="a memory.db; default the newest recording")
    p.add_argument("--out", default=str(CACHE / "recall.jpg"))
    p.add_argument("--json", action="store_true", help='print {"text", "image"}')
    p.add_argument("--now", type=float, help="pretend it is this epoch time (for old recordings)")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("streams")
    r = sub.add_parser("recent")
    r.add_argument("--seconds", type=float, default=60.0)
    r.add_argument("--n", type=int, default=8)
    nr = sub.add_parser("near")
    nr.add_argument("x", type=float)
    nr.add_argument("y", type=float)
    nr.add_argument("--radius", type=float, default=2.0)
    nr.add_argument("--minutes", type=float, default=30.0)
    a_ = sub.add_parser("at")
    a_.add_argument("when", help="HH:MM:SS today")
    tr = sub.add_parser("trail")
    tr.add_argument("--minutes", type=float, default=10.0)
    fd = sub.add_parser("find")
    fd.add_argument("text")
    fd.add_argument("--minutes", type=float, default=10.0)
    fd.add_argument("--k", type=int, default=6)
    a = p.parse_args(argv)
    CACHE.mkdir(exist_ok=True)
    store = open_store(a.db)
    if a.now:
        global NOW
        NOW = a.now
    if a.cmd == "streams":
        text, image = streams(store), None
    elif a.cmd == "recent":
        text, image = recent(store, a.seconds, a.out, a.n), a.out
    elif a.cmd == "near":
        text = near(store, a.x, a.y, a.radius, a.minutes, a.out)
        image = a.out if text.startswith(a.out) else None
    elif a.cmd == "at":
        text, image = at(store, a.when, a.out), a.out
    elif a.cmd == "trail":
        out = a.out if a.out.endswith(".png") else str(Path(a.out).with_suffix(".png"))
        text, image = trail(store, a.minutes, out), out
    else:
        text, image = find(store, a.text, a.minutes, a.out, a.k), a.out
    if a.json:
        print(json.dumps({"text": text, "image": image}))
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
