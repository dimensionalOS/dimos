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

from collections.abc import Mapping, Sequence
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


def mjcf_with_frames(
    mjcf: Path,
    frames: Frames,
    out_dir: Path | None = None,
    collide_drawn: Sequence[str] = (),
) -> Path:
    """A copy of `mjcf` with each frame as a site on the body it is fixed to; reused while nothing changes.

    ``collide_drawn`` names bodies that are drawn but cannot be hit, and gives each drawn
    shape a collider of its own; see ``_collide_drawn``.
    """
    mjcf = Path(mjcf).resolve()
    key = hashlib.sha256(
        f"{mjcf}\0{mjcf.stat().st_mtime_ns}\0{sorted(frames.items())}"
        f"\0{sorted(collide_drawn)}".encode()
    )
    out = (out_dir or _CACHE) / f"{mjcf.stem}-{key.hexdigest()[:16]}.xml"
    if not out.exists():
        out.parent.mkdir(parents=True, exist_ok=True)
        partial = out.with_suffix(".partial")
        partial.write_text(_with_frames(mjcf, frames, collide_drawn))
        partial.replace(out)
    return out


# Which geoms are drawn and which are collided with. MuJoCo says a geom collides when its
# contype or conaffinity is set, and the convention these models follow is that the drawn
# meshes are the ones that do neither.
def _is_collision(spec_geom) -> bool:
    return bool(spec_geom.contype) or bool(spec_geom.conaffinity)


def _collide_drawn(spec, body_name: str) -> None:
    """Give every drawn shape on a body a collider, where it has none of its own.

    A shape that is drawn but cannot be hit is invisible to anything that senses by contact or
    by ray -- Menagerie's G1 draws a rubber hand on each wrist and collides only the wrist, so
    the robot's own lidar sees its shoulders and its elbows and looks straight through its
    hands. Reusing the drawn mesh means the collider is the shape, not a guess at it; MuJoCo
    takes its convex hull, which a hand is close to anyway.

    Mass is left at zero: these bodies carry an explicit inertial, and a collider is being added
    to say where the surface is, not to add a hand the robot was not already carrying.
    """
    body = spec.body(body_name)
    if body is None:
        raise ValueError(f"no body named {body_name!r} to give colliders to")
    drawn = [g for g in body.geoms if not _is_collision(g)]
    for geom in list(drawn):
        if any(_is_collision(g) and g.meshname == geom.meshname for g in body.geoms):
            continue
        body.add_geom(
            name=f"{geom.name or geom.meshname}_collision",
            type=geom.type,
            meshname=geom.meshname,
            pos=geom.pos,
            quat=geom.quat,
            size=geom.size,
            group=_COLLISION_GROUP,
            contype=1,
            conaffinity=1,
            mass=0.0,
        )


# Where this family of models keeps collision geometry, and so where a lidar is told to look.
_COLLISION_GROUP = 3


def _with_frames(
    mjcf: Path,
    frames: Frames,
    collide_drawn: Sequence[str] = (),
) -> str:
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
    for body_name in collide_drawn:
        _collide_drawn(spec, body_name)
    return str(spec.to_xml())
