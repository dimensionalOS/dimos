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

"""Odom memory-row generators for the VQA / memory-eval pipeline.

``displacement`` and ``path_length`` families compute ground truth analytically
from privileged odom; emitted rows quiz a lossily-encoded surface. Rows are
pure data — suite modules (for example ``go2_vqa``) map them onto
:class:`~dimos.evals.types.EvalCase` values. Numeric band scoring stays outside
the image-choice VQA families until that protocol grows a numeric answer path.
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

from dimos.memory.cli.dataset import open_dataset

Row = dict[str, object]


def displacement_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """Straight-line displacement over each window (odom is the privileged truth;
    the case quizzes the encoded odom summary). Sampling-invariant ground truth."""
    store = open_dataset(dataset)
    try:
        rows: list[Row] = []
        for t1, t2 in windows:
            obs = store.streams.odom.range_time(t1, t2).to_list()
            if len(obs) < 2:
                continue
            d = (obs[-1].data.position - obs[0].data.position).length()
            rows.append(
                {
                    "id": f"{dataset}_disp_{t1:g}_{t2:g}",
                    "family": "displacement",
                    "q": "How far in a straight line is your final position from your "
                    "position at the first shown observation, in meters?",
                    "a": round(d, 1),
                    "band": max(1.0, d * 0.4),
                    "stream": "odom",
                    "window": [t1, t2],
                    "dataset": dataset,
                }
            )
        return rows
    finally:
        store.stop()


def path_length_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """Integrated path length per window. Deliberately hard on a downsampled
    encoding — expect partial credit; that gap is the finding."""
    store = open_dataset(dataset)
    try:
        rows: list[Row] = []
        for t1, t2 in windows:
            path, prev = 0.0, None
            for obs in store.streams.odom.range_time(t1, t2):
                p = obs.data.position
                if prev is not None:
                    path += (p - prev).length()
                prev = p
            if prev is None:
                continue
            rows.append(
                {
                    "id": f"{dataset}_path_{t1:g}_{t2:g}",
                    "family": "path_length",
                    "q": "Roughly how many meters did you travel in total over these "
                    "observations (path length, not displacement)?",
                    "a": round(path, 1),
                    "band": max(2.0, path * 0.5),
                    "stream": "odom",
                    "window": [t1, t2],
                    "dataset": dataset,
                }
            )
        return rows
    finally:
        store.stop()


def memory_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """Displacement and path-length rows for each window in *dataset*."""
    return displacement_rows(dataset, windows) + path_length_rows(dataset, windows)


def write_rows(path: Path, rows: Sequence[Row]) -> None:
    path.write_text(json.dumps(list(rows), indent=2) + "\n", encoding="utf-8")
