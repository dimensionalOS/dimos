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

"""Export selected sensor observations without a DimOS runtime or derived answers."""

from __future__ import annotations

from collections.abc import Sequence
import hashlib
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from PIL import Image as PILImage

from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.memory.stream import Stream


def plain_recording(streams: Sequence[Stream[Any, Any]], directory: Path) -> Path:
    """Write lossless PNGs, XYZ/RGB CSVs and JSON primitives, with a manifest.

    Only explicitly supported payloads are exported. No pickle, Python repr,
    semantic tags, source paths or agent_encode summaries cross this boundary.
    """
    directory.mkdir(parents=True, exist_ok=False)
    records: list[dict[str, Any]] = []
    for stream_index, stream in enumerate(streams):
        for index, obs in enumerate(stream):
            data = obs.data
            stem = f"{stream_index:03d}-{index:06d}"
            record: dict[str, Any] = {
                "stream": stream.name,
                "timestamp": obs.ts,
                "frame_id": getattr(data, "frame_id", None),
            }
            if isinstance(data, PointCloud2):
                points, colors = data.as_numpy()
                values = points if colors is None else np.column_stack((points, colors))
                path = directory / f"{stem}.csv"
                np.savetxt(
                    path,
                    values,
                    delimiter=",",
                    fmt="%.17g",
                    comments="",
                    header="x,y,z" if colors is None else "x,y,z,r,g,b",
                )
                record["format"] = "XYZ in source coordinates; optional RGB in [0,1]"
            elif isinstance(data, Image):
                path = directory / f"{stem}.png"
                PILImage.fromarray(data.to_rgb().data).save(path)
                record["format"] = "RGB PNG"
            elif data is None or isinstance(data, (bool, int, float, str, list, dict)):
                path = directory / f"{stem}.json"
                path.write_text(json.dumps(data, allow_nan=False))
                record["format"] = "JSON"
            else:
                raise TypeError(f"No plain recording export for {type(data).__name__}")
            record.update(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            records.append(record)
    if not records:
        raise ValueError("No selected observations to export")
    manifest = directory / "manifest.json"
    manifest.write_text(json.dumps({"version": 1, "observations": records}, indent=2))
    return manifest
