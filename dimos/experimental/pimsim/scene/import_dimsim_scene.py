# Copyright 2025-2026 Dimensional Inc.
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

"""Compose a DimSim authored scene into one cook-ready GLB.

A DimSim scene (``misc/DimSim/scenes/<name>/``) is a static shell
(``structure.glb``) plus a manifest of furniture, each piece a separate GLB
placed by a TRS transform (``objects/manifest.json``). The PimSim cooker
ingests a single source asset with an object hierarchy, so this tool replays
DimSim's placement (``engine.js`` ``instantiateAsset`` glbUrl path: plain
position / Euler-XYZ-radians rotation / scale on the GLB root, no pivot
re-center) and fuses everything into one GLB whose nodes are named by furniture
slug. Those node names become ``objects.json`` landmarks at cook time, so
``findAsset("bed")`` works without any sidecar; a sidecar then only needs to
carve out the few pieces we want genuinely dynamic.

Coordinates stay in glTF/three.js Y-up; the cooker's ``y_up`` alignment maps
them to the Dimos world frame. We never round-trip through a Z-up tool.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _euler_xyz_radians(rx: float, ry: float, rz: float) -> np.ndarray:
    """three.js ``Euler`` order 'XYZ' rotation matrix (intrinsic X→Y→Z).

    Matches ``Matrix4.makeRotationFromEuler`` so composed furniture lands
    exactly where DimSim renders it.
    """
    c1, s1 = np.cos(rx), np.sin(rx)
    c2, s2 = np.cos(ry), np.sin(ry)
    c3, s3 = np.cos(rz), np.sin(rz)
    r = np.eye(4)
    r[0, 0] = c2 * c3
    r[0, 1] = -c2 * s3
    r[0, 2] = s2
    r[1, 0] = c1 * s3 + c3 * s1 * s2
    r[1, 1] = c1 * c3 - s1 * s2 * s3
    r[1, 2] = -c2 * s1
    r[2, 0] = s1 * s3 - c1 * c3 * s2
    r[2, 1] = c3 * s1 + c1 * s2 * s3
    r[2, 2] = c1 * c2
    return r


def _trs_matrix(transform: dict) -> np.ndarray:
    """Compose ``T · R · S`` from a manifest transform (three.js convention)."""
    pos = transform.get("position", {}) or {}
    rot = transform.get("rotation", {}) or {}
    scl = transform.get("scale", {}) or {}

    t = np.eye(4)
    t[:3, 3] = [pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)]

    r = _euler_xyz_radians(rot.get("x", 0.0), rot.get("y", 0.0), rot.get("z", 0.0))

    s = np.eye(4)
    s[0, 0] = scl.get("x", 1.0)
    s[1, 1] = scl.get("y", 1.0)
    s[2, 2] = scl.get("z", 1.0)

    return t @ r @ s


def _state_file(asset: dict) -> str | None:
    """Resolve the asset's current (or first) state GLB, relative to objects/."""
    states = asset.get("states") or []
    if not states:
        return None
    current = asset.get("currentStateId")
    for state in states:
        if state.get("id") == current:
            return state.get("file")
    return states[0].get("file")


def _slug(state_file: str) -> str:
    """The furniture directory is already a clean title slug."""
    return state_file.split("/", 1)[0]


def _add_asset(
    scene: trimesh.Scene,
    glb_path: Path,
    matrix: np.ndarray,
    node_name: str,
) -> int:
    """Bake the asset GLB's internal node graph, then place it under ``matrix``.

    Per-geometry materials are preserved (no concatenation) so textures
    survive for the VLM-perception demos.
    """
    loaded = trimesh.load(glb_path, process=False)
    geometries = loaded.dump(concatenate=False) if isinstance(loaded, trimesh.Scene) else [loaded]
    added = 0
    for index, geom in enumerate(geometries):
        if getattr(geom, "vertices", None) is None or len(geom.vertices) == 0:
            continue
        name = node_name if len(geometries) == 1 else f"{node_name}.{index}"
        scene.add_geometry(geom, node_name=name, geom_name=name, transform=matrix)
        added += 1
    return added


def import_dimsim_scene(scene_dir: str | Path, output: str | Path) -> Path:
    """Fuse ``structure.glb`` + manifest furniture into one cook-ready GLB."""
    scene_dir = Path(scene_dir).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    structure = scene_dir / "structure.glb"
    manifest_path = scene_dir / "objects" / "manifest.json"
    if not structure.exists():
        raise FileNotFoundError(f"missing static shell: {structure}")
    if not manifest_path.exists():
        raise FileNotFoundError(f"missing furniture manifest: {manifest_path}")

    combined = trimesh.Scene()
    _add_asset(combined, structure, np.eye(4), "structure")

    manifest = json.loads(manifest_path.read_text())
    used: dict[str, int] = {}
    placed = missing = 0
    for asset in manifest:
        state_file = _state_file(asset)
        if not state_file:
            continue
        glb_path = scene_dir / "objects" / state_file
        if not glb_path.exists():
            logger.warning("furniture GLB missing, skipping: %s", glb_path)
            missing += 1
            continue
        slug = _slug(state_file)
        count = used.get(slug, 0)
        used[slug] = count + 1
        node_name = slug if count == 0 else f"{slug}-{count}"
        if _add_asset(combined, glb_path, _trs_matrix(asset.get("transform", {})), node_name):
            placed += 1

    combined.export(output)
    logger.info(
        "composed DimSim scene %s -> %s (%d placed, %d missing)",
        scene_dir.name,
        output,
        placed,
        missing,
    )
    return output


def cli_main() -> None:
    parser = argparse.ArgumentParser(
        description="Compose a DimSim authored scene into one cook-ready GLB.",
    )
    parser.add_argument("scene_dir", type=Path, help="misc/DimSim/scenes/<name>/")
    parser.add_argument("output", type=Path, help="destination .glb")
    args = parser.parse_args()
    print(import_dimsim_scene(args.scene_dir, args.output))


if __name__ == "__main__":
    cli_main()


__all__ = ["import_dimsim_scene"]
