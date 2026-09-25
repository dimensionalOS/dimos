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

"""Where image files go and what they are called."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import numpy as np

from dimos.constants import STATE_DIR
from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Drawable
from dimos.experimental.agent_encode.pointcloud.queries.base import Query
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import finite_points
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def output_dir(out_dir: Path | None) -> Path:
    """The absolute directory image files go in: ``out_dir`` when given, else the
    current run's log directory, else the dimos state directory."""
    if out_dir is not None:
        selected = Path(out_dir)
    elif run_dir := os.environ.get("DIMOS_RUN_LOG_DIR"):
        selected = Path(run_dir) / "agent_encode"
    else:
        selected = STATE_DIR / "agent_encode"
    return selected.expanduser().resolve()


def digest_stem(*parts: bytes) -> str:
    """A file name prefix that is the same exactly when ``parts`` are."""
    digest = hashlib.sha256()
    for part in parts:
        digest.update(hashlib.sha256(part).digest())
    return f"pointcloud_{digest.hexdigest()[:24]}"


def cloud_bytes(cloud: PointCloud2) -> bytes:
    """What identifies a cloud: its finite returns, frame and timestamp."""
    points = np.ascontiguousarray(finite_points(cloud), dtype="<f4").tobytes()
    return points + repr((cloud.frame_id, cloud.ts)).encode()


def draw_bytes(draw: tuple[Drawable | PointCloud2, ...]) -> list[bytes]:
    """What identifies each ``draw=`` item; a cloud by its returns, not its repr."""
    return [
        cloud_bytes(item) if isinstance(item, PointCloud2) else repr(item).encode() for item in draw
    ]


def file_stem(
    query: Query[object], cloud: PointCloud2, draw: tuple[Drawable | PointCloud2, ...]
) -> str:
    """A file name prefix that is the same for the same query on the same cloud with the
    same items drawn over it."""
    return digest_stem(repr(query).encode(), cloud_bytes(cloud), *draw_bytes(draw))
