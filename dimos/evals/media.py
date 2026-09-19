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

"""Captioned and tiled videos from eval runs, with ffmpeg."""

from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
TILE_W, TILE_H = 960, 540


def arm_label(run_dir: Path) -> str:
    """``pi gpt-6-astra no-dimOS`` from the run's manifest."""
    manifest = json.loads((run_dir / "manifest.json").read_text())
    agent = manifest.get("agent") or {}
    module = str(agent.get("module", "")).rsplit(".", 1)[-1] or "agent"
    kwargs = agent.get("kwargs") or {}
    parts = [module, str(kwargs.get("model", ""))]
    if kwargs.get("no_dimos"):
        parts.append("no-dimOS")
    if kwargs.get("modules"):
        parts.append(",".join(kwargs["modules"]))
    return " ".join(p for p in parts if p)


def _escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")


def caption(src: Path, dst: Path, lines: list[str]) -> Path:
    """Burn ``lines`` into the top-left corner of ``src``."""
    filters = [
        f"drawtext=fontfile={FONT}:text='{_escape(line)}':x=24:y={24 + 44 * i}:fontsize=36:"
        "fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=10"
        for i, line in enumerate(lines)
    ]
    dst.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-y", "-i", str(src), "-vf", ",".join(filters),
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", "-an", str(dst)],
        check=True,
    )  # fmt: skip
    return dst


def caption_runs(runs: list[Path], out: Path) -> dict[tuple[str, str], Path]:
    """One captioned video per (case, run): arm, case, score and time to object."""
    done: dict[tuple[str, str], Path] = {}
    for run in runs:
        label = arm_label(run)
        for row in (run / "results.jsonl").read_text().splitlines():
            r = json.loads(row)
            src = run / r["case_id"] / "viewer.mp4"
            if not src.exists():
                continue
            metrics_path = run / r["case_id"] / "nav_metrics.json"
            lines = [label, r["case_id"], f"score {r['score']:.2f}"]
            if metrics_path.exists():
                m = json.loads(metrics_path.read_text())
                lines[-1] += (
                    f"  reached {m['reached']}  t {m['time_to_object_s']:.0f}s  bumps {m['bumps']}"
                )
            slug = label.replace(" ", "_").replace("/", "_")
            done[(r["case_id"], label)] = caption(src, out / f"{r['case_id']}-{slug}.mp4", lines)
    return done


def tile(videos: list[Path], dst: Path) -> Path:
    """Tile the videos in a grid, each scaled to a 960x540 tile, padded to the longest."""
    n = len(videos)
    cols = math.ceil(math.sqrt(n))
    inputs = [arg for v in videos for arg in ("-i", str(v))]
    scaled = "".join(
        f"[{i}:v]scale={TILE_W}:{TILE_H}:force_original_aspect_ratio=decrease,"
        f"pad={TILE_W}:{TILE_H}:(ow-iw)/2:(oh-ih)/2,tpad=stop_mode=clone:stop_duration=3600[v{i}];"
        for i in range(n)
    )
    layout = "|".join(f"{(i % cols) * TILE_W}_{(i // cols) * TILE_H}" for i in range(n))
    chain = (
        scaled
        + "".join(f"[v{i}]" for i in range(n))
        + f"xstack=inputs={n}:layout={layout}:fill=black[out]"
    )
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-y", *inputs, "-filter_complex", chain, "-map", "[out]",
         "-shortest", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", str(dst)],
        check=True,
    )  # fmt: skip
    return dst
