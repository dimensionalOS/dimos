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

"""An MJCF with named frames (sensor mounts) added as sites, so a sim sensor and the URDF made from it name the same frame."""

from __future__ import annotations

from collections.abc import Mapping
import hashlib
from pathlib import Path
import tempfile

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

Vector = tuple[float, float, float]
# name -> (parent, xyz, rpy): the parent is a body of the model or a frame listed before it.
Frames = Mapping[str, tuple[str, Vector, Vector]]

_CACHE = Path(tempfile.gettempdir()) / "dimos-sim2-mjcf"


def mjcf_with_frames(mjcf: Path, frames: Frames, out_dir: Path | None = None) -> Path:
    """A copy of `mjcf` with each frame as a site on the body it is fixed to; reused while nothing changes."""
    mjcf = Path(mjcf).resolve()
    key = hashlib.sha256(f"{mjcf}\0{mjcf.stat().st_mtime_ns}\0{sorted(frames.items())}".encode())
    out = (out_dir or _CACHE) / f"{mjcf.stem}-{key.hexdigest()[:16]}.xml"
    if not out.exists():
        out.parent.mkdir(parents=True, exist_ok=True)
        partial = out.with_suffix(".partial")
        partial.write_text(_with_frames(mjcf, frames))
        partial.replace(out)
    return out


def _with_frames(mjcf: Path, frames: Frames) -> str:
    spec = mujoco.MjSpec.from_file(str(mjcf))
    # The copy lives elsewhere, so the files the model names are found by full path.
    for kind in ("meshdir", "texturedir"):
        setattr(spec, kind, str((mjcf.parent / (getattr(spec, kind) or "")).resolve()))
    placed: dict[str, tuple[str, NDArray[np.float64]]] = {}
    for name, (parent, xyz, rpy) in frames.items():
        on_parent = np.eye(4)
        on_parent[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        on_parent[:3, 3] = xyz
        if parent in placed:
            parent, above = placed[parent]
            on_parent = above @ on_parent
        body = spec.body(parent)
        if body is None:
            raise ValueError(f"frame {name!r}: no body or earlier frame {parent!r} in {mjcf}")
        placed[name] = (parent, on_parent)
        quat = Rotation.from_matrix(on_parent[:3, :3]).as_quat(scalar_first=True)
        body.add_site(
            name=name, pos=on_parent[:3, 3], quat=quat, size=(0.005, 0.005, 0.005), group=5
        )
    return str(spec.to_xml())
