#!/usr/bin/env python3
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

"""A vendor MJCF as the USD dimos serves. Run once by hand; the result is committed.

Why two steps. Google DeepMind's converter (``mujoco-usd`` on PyPI) is the only thing that reads
an MJCF into USD with its physics intact -- it writes MjcActuator, MjcJointAPI and MjcCollisionAPI
prims, so MuJoCo can still simulate from the layer and nothing is smuggled through customData.
But it **bakes** every mesh into the layer: the M20 comes out 4.8 MB of binary crate, 19 MB as
text, with no reference arcs at all. dimos hands a scene over as one string and serves each mesh
separately by content hash, so baked geometry is both too big and the wrong shape.

So the converter runs first, in its own virtualenv, and this re-points the baked meshes at the
files they came from:

    python3 -m venv /tmp/mjcfusd
    /tmp/mjcfusd/bin/pip install mujoco-usd-converter
    # its own warning: usd-exchange ships OpenUSD and will not share with usd-core
    /tmp/mjcfusd/bin/pip uninstall -y usd-core
    /tmp/mjcfusd/bin/pip install --force-reinstall usd-exchange
    /tmp/mjcfusd/bin/python -c "
    import mujoco_usd_converter
    mujoco_usd_converter.Converter().convert('<robot>.xml', '/tmp/converted')"

    # then, in dimos's own venv:
    python scripts/mjcf_to_usd.py --mjcf <robot>.xml --converted /tmp/converted/<Robot>.usda \\
        --meshes <robot>/meshes --out dimos/robot/<company>/<robot>/assets/<robot>.usda

Each mesh then ends up named twice, on purpose, because two kinds of consumer want two
different things from it:

* a **reference arc** to a ``.usdc`` sidecar this writes beside the original. A reference
  composes a USD layer and only a USD layer, so this is what makes the result valid USD --
  usdview, Isaac and MuJoCo 4 all compose it and get real triangles.
* ``assetInfo["identifier"]``, naming the **original mesh file**, which is what dimos serves
  over its own bus and what dim-controller fetches and draws. It never composes the stage.

Dropping either one costs a consumer its geometry, so both are authored.

**Never install mujoco-usd into dimos's virtualenv.** It pulls usd-exchange, which bundles its
own OpenUSD and declares usd-core incompatible, and mujoco-core, which shadows mujoco. Removing
them afterwards takes usd-core's modules with it.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import sys
from typing import Any
import xml.etree.ElementTree as ET

from dimos.robot.assets.usd import hash_of_file
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The geometry a referenced mesh file supplies for itself.
_BAKED = frozenset({"points", "faceVertexIndices", "faceVertexCounts", "normals", "extent"})
# What a sidecar is written as: binary, because nothing hands these around as text -- they are
# fetched as bytes, one file per mesh, exactly like the STL beside them.
_SIDECAR_SUFFIX = ".usdc"


def write_sidecar(source: Path, pxr: Any) -> Path:
    """``source`` as a one-prim USD layer beside it, so a reference to it is valid USD.

    Returns the layer's path. Rewritten every run rather than cached: it is derived from the
    mesh, and a stale sidecar is a robot whose geometry silently does not match its meshes.
    """
    import trimesh

    out = source.with_suffix(_SIDECAR_SUFFIX)
    loaded = trimesh.load_mesh(str(source), process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.to_geometry()

    stage = pxr.Usd.Stage.CreateInMemory(out.name)
    mesh = pxr.UsdGeom.Mesh.Define(stage, "/mesh")
    mesh.CreatePointsAttr([pxr.Gf.Vec3f(*point) for point in loaded.vertices.tolist()])
    mesh.CreateFaceVertexIndicesAttr([int(i) for i in loaded.faces.reshape(-1).tolist()])
    mesh.CreateFaceVertexCountsAttr([3] * len(loaded.faces))
    # Without an extent a consumer has to load every point to frame the robot. A degenerate
    # mesh has no bounds to state, and saying nothing is better than saying something wrong.
    bounds = loaded.bounds
    if bounds is not None:
        mesh.CreateExtentAttr(
            [pxr.Gf.Vec3f(*bounds[0].tolist()), pxr.Gf.Vec3f(*bounds[1].tolist())]
        )
    stage.SetDefaultPrim(mesh.GetPrim())
    stage.GetRootLayer().Export(str(out))
    return out


def mesh_files(mjcf: Path) -> dict[str, str]:
    """Every name a baked prim might carry -> the mesh file behind it.

    Two spellings, because the converter names a mesh prim after whichever the MJCF gave it:

    * the **mesh asset's** own name, which is what a robot MJCF uses -- its geoms are unnamed
      and the prim inherits `<mesh name=...>`;
    * the **geom's** name, which is what a scene MJCF uses -- office_555 names every geom
      (`drawer_cabinet_5_g29`) and has them reference meshes called `asset_0`, `asset_1`, ...

    Indexing both means one lookup covers both, and a robot and a room go through the same
    script. Without the geom spelling the office scene re-points nothing: 3547 meshes stay
    baked and the layer comes out 35 MB.
    """
    root = ET.fromstring(mjcf.read_text())
    files = {}
    for mesh in root.iter("mesh"):
        name, file = mesh.get("name"), mesh.get("file")
        if name and file:
            files[name] = file
    for geom in root.iter("geom"):
        name, mesh_name = geom.get("name"), geom.get("mesh")
        if name and mesh_name and mesh_name in files:
            files[name] = files[mesh_name]
    return files


def _mesh_of(prim_name: str, files: dict[str, str]) -> str | None:
    """The mesh file a prim's name refers to, allowing for the converter's de-duplication.

    Two geoms on one body that use the same mesh become two prims, and the converter names the
    second ``<mesh>_1``. Both are the same file. Without this every such prim stays baked, which
    on the G1 is 23 of 58.
    """
    if prim_name in files:
        return files[prim_name]
    stripped = re.sub(r"_\d+$", "", prim_name)
    return files.get(stripped) if stripped != prim_name else None


def rereference(
    stage: Any, files: dict[str, str], meshes: Path, out_dir: Path, pxr: Any
) -> tuple[int, int]:
    """Swap each baked mesh for a reference to its file, with the file's hash beside it.

    Returns how many were re-pointed and how many were left alone. A mesh the MJCF does not
    name is left baked rather than dropped: it came from somewhere this cannot resolve, and a
    hole in the robot is worse than a large layer.

    References are written **relative to the layer**, because the layer is committed and the
    absolute path it was converted at means nothing on anyone else's machine.
    """
    done = skipped = 0
    for prim in list(stage.Traverse()):
        if not prim.IsA(pxr.UsdGeom.Mesh):
            continue
        file = _mesh_of(prim.GetName(), files)
        if file is None:
            logger.warning(f"{prim.GetPath()} has no mesh of that name in the MJCF; left baked")
            skipped += 1
            continue
        # --meshes is authoritative, so a `file=` that carries a directory (because the MJCF
        # set meshdir, or names the vendor checkout by relative path) is matched by its
        # basename rather than pasted onto the directory a second time.
        path = next(
            (
                candidate
                for candidate in ((meshes / file).resolve(), (meshes / Path(file).name).resolve())
                if candidate.is_file()
            ),
            None,
        )
        if path is None:
            logger.warning(f"no {file} under {meshes}; {prim.GetPath()} left baked")
            skipped += 1
            continue
        # Everything per-vertex goes: the reference supplies it, and it is what made the layer
        # large. primvars included -- the converter bakes normals there, which alone is most of
        # the size. Transform ops and material bindings stay, since the file carries neither.
        for property in list(prim.GetPropertyNames()):
            if property.startswith("xformOp") or property.startswith("material:"):
                continue
            if property.startswith("primvars:") or property in _BAKED:
                prim.RemoveProperty(property)
        sidecar = write_sidecar(path, pxr)
        relative = os.path.relpath(path, out_dir)
        prim.GetReferences().ClearReferences()
        # The sidecar, not the mesh file: a reference composes a USD layer and nothing else.
        prim.GetReferences().AddReference(os.path.relpath(sidecar, out_dir))
        digest = hash_of_file(str(path))
        info = prim.GetAssetInfo() or {}
        info["identifier"] = pxr.Sdf.AssetPath(relative)
        if digest:
            info["version"] = digest
        prim.SetAssetInfo(info)
        done += 1
    return done, skipped


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mjcf", required=True, type=Path, help="the vendor MJCF, for its mesh names"
    )
    parser.add_argument("--converted", required=True, type=Path, help="the converter's stage")
    parser.add_argument(
        "--meshes", required=True, type=Path, help="the directory holding the mesh files"
    )
    parser.add_argument("--out", required=True, type=Path, help="the .usda to write")
    args = parser.parse_args()

    from pxr import Gf, Sdf, Usd, UsdGeom

    pxr = argparse.Namespace(Gf=Gf, Sdf=Sdf, Usd=Usd, UsdGeom=UsdGeom)

    for path in (args.mjcf, args.converted):
        if not path.is_file():
            print(f"ERROR: no file at {path}", file=sys.stderr)
            return 1

    files = mesh_files(args.mjcf)
    if not files:
        print(f"ERROR: {args.mjcf} names no meshes", file=sys.stderr)
        return 1

    # Flattened, because the converter writes a root that payloads several binary layers and a
    # scene has to leave here as one text layer.
    stage = Usd.Stage.Open(str(args.converted))
    flat = Usd.Stage.Open(stage.Flatten())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    done, skipped = rereference(flat, files, args.meshes, args.out.parent.resolve(), pxr)

    # The ROOT LAYER, not the stage: Stage.Export() flattens composition, which resolves the
    # reference arcs we just authored back into baked geometry -- the opposite of the point.
    flat.GetRootLayer().Export(str(args.out))
    size = args.out.stat().st_size
    print(f"{done} mesh(es) re-pointed with a .usdc sidecar, {skipped} left baked")
    print(f"wrote {args.out} ({size / 1e6:.2f} MB)")
    if skipped:
        print("note: a baked mesh keeps the layer large; see the warnings above for why")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
