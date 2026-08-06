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

"""Pull RTAB-Map trajectories out of the Plotly page the RTAB-Map runs are shared as.

    python -m dimos.mapping.import_rtabmap_html <all_maps.html>

The page holds one ``Plotly.newPlot("map-N", [...])`` per run, ordered to match the
section headings, and each run carries an offline (pose-graph optimised) and an online
(live) trace. Only x/y are plotted -- there are no timestamps -- so the saved
trajectories keep the existing RTAB-Map convention of a sample index in the time column
and stay unscorable against Point-LIO. They are still worth plotting: shape and extent
are exactly what the stereo/IMU rerun changed.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any

import numpy as np

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DATASETS = Path.home() / "datasets" / "d455"
SECTION = re.compile(r"(\w+) — (stereo infrared \+ 400 Hz IMU|colour \+ hardware depth)</a>")
# Which stored method each (variant, trace group) pair becomes.
STEREO_VARIANT = "stereo infrared + 400 Hz IMU"
METHOD_FOR = {
    (STEREO_VARIANT, "offline"): "rtabmap_stereo_imu",
    (STEREO_VARIANT, "online"): "rtabmap_stereo_imu_online",
    ("colour + hardware depth", "offline"): "rtabmap",
    ("colour + hardware depth", "online"): "rtabmap_online",
}


def plot_traces(text: str, index: int) -> list[dict[str, Any]]:
    key = f'"map-{index}",'
    start = text.index(key) + len(key)
    while text[start] in " \n\t":
        start += 1
    depth = 0
    for position in range(start, len(text)):
        if text[position] == "[":
            depth += 1
        elif text[position] == "]":
            depth -= 1
            if depth == 0:
                loaded: list[dict[str, Any]] = json.loads(text[start : position + 1])
                return loaded
    raise RuntimeError(f"map-{index} payload is not terminated")


def import_page(html_path: Path, write: bool = True) -> list[tuple[str, str, int]]:
    text = html_path.read_text(errors="ignore")
    sections = SECTION.findall(text)
    if not sections:
        raise RuntimeError("no run sections found; is this the RTAB-Map page?")

    written: list[tuple[str, str, int]] = []
    for index, (recording, variant) in enumerate(sections):
        traces = plot_traces(text, index)
        for group, keyword in (("offline", "offline"), ("online", "online")):
            method = METHOD_FOR.get((variant, group))
            if method is None:
                continue
            # Each group is drawn as several traces; the first carries every point.
            match = next(
                (
                    trace
                    for trace in traces
                    if keyword in str(trace.get("legendgroup", "")) and len(trace.get("x", [])) > 2
                ),
                None,
            )
            if match is None:
                logger.warning("%s %s: no %s trace", recording, variant, group)
                continue
            x = np.asarray(match["x"], dtype=float)
            y = np.asarray(match["y"], dtype=float)
            keep = np.isfinite(x) & np.isfinite(y)
            x, y = x[keep], y[keep]
            # No timestamps in the page; index stands in, as the existing RTAB-Map
            # trajectories already do, which is why these stay unscorable.
            trajectory = np.column_stack([np.arange(len(x), dtype=float), x, y, np.zeros(len(x))])
            destination = DATASETS / recording / f"{method}_traj.npy"
            if write:
                if not destination.parent.exists():
                    logger.warning("%s has no dataset directory, skipping", recording)
                    continue
                np.save(destination, trajectory)
            written.append((recording, method, len(trajectory)))
            logger.info("%s -> %s (%d poses)", recording, destination.name, len(trajectory))
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("html", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    written = import_page(args.html, write=not args.dry_run)
    print(json.dumps([{"recording": r, "method": m, "poses": n} for r, m, n in written], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
