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
an MJCF into USD with its physics intact -- MjcActuator, MjcJointAPI and MjcCollisionAPI prims,
so MuJoCo can still simulate from the layer. But it **bakes** every mesh into the layer (the M20
comes out 19 MB as text, with no reference arcs), and dimos serves each mesh separately by
content hash.

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
  composes a USD layer and only a USD layer, so this is what makes the result valid USD.
* ``assetInfo["identifier"]``, naming the **original mesh file**, which is what dimos serves
  over its own bus. It never composes the stage.

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

from dimos.robot.assets.usd_assets import hash_of_file
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_BAKED = frozenset({"points", "faceVertexIndices", "faceVertexCounts", "normals", "extent"})
_KEPT_PRIMVARS = frozenset({"primvars:displayColor", "primvars:displayOpacity"})
_SIDECAR_SUFFIX = ".usdc"


def write_sidecar(source: Path, pxr: Any) -> Path:
    import trimesh

    out = source.with_suffix(_SIDECAR_SUFFIX)
    loaded = trimesh.load_mesh(str(source), process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.to_geometry()
    if len(loaded.faces) == 0:
        raise ValueError(
            f"{source} has no faces. If it is {source.stat().st_size} bytes it is a git-lfs "
            f"pointer: run `git lfs checkout` on its directory first."
        )

    stage = pxr.Usd.Stage.CreateInMemory(out.name)
    mesh = pxr.UsdGeom.Mesh.Define(stage, "/mesh")
    mesh.CreatePointsAttr([pxr.Gf.Vec3f(*point) for point in loaded.vertices.tolist()])
    mesh.CreateFaceVertexIndicesAttr([int(i) for i in loaded.faces.reshape(-1).tolist()])
    mesh.CreateFaceVertexCountsAttr([3] * len(loaded.faces))
    bounds = loaded.bounds
    if bounds is not None:
        mesh.CreateExtentAttr(
            [pxr.Gf.Vec3f(*bounds[0].tolist()), pxr.Gf.Vec3f(*bounds[1].tolist())]
        )
    stage.SetDefaultPrim(mesh.GetPrim())
    stage.GetRootLayer().Export(str(out))
    return out


def mesh_files(mjcf: Path) -> dict[str, str]:
    root = ET.fromstring(mjcf.read_text())
    files = {}
    for mesh in root.iter("mesh"):
        file = mesh.get("file")
        name = mesh.get("name") or (Path(file).stem if file else None)
        if name and file:
            files[name] = file
    for geom in root.iter("geom"):
        name, mesh_name = geom.get("name"), geom.get("mesh")
        if name and mesh_name and mesh_name in files:
            files[name] = files[mesh_name]
    return files


def _mesh_of(prim_name: str, files: dict[str, str]) -> str | None:
    if prim_name in files:
        return files[prim_name]
    stripped = re.sub(r"_\d+$", "", prim_name)
    return files.get(stripped) if stripped != prim_name else None


def rereference(
    stage: Any, files: dict[str, str], meshes: Path, out_dir: Path, pxr: Any
) -> tuple[int, int]:
    done = skipped = 0
    for prim in list(stage.Traverse()):
        if not prim.IsA(pxr.UsdGeom.Mesh):
            continue
        file = _mesh_of(prim.GetName(), files)
        if file is None:
            logger.warning(f"{prim.GetPath()} has no mesh of that name in the MJCF; left baked")
            skipped += 1
            continue
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
        for property in list(prim.GetPropertyNames()):
            if property.startswith(("xformOp", "material:")) or property in _KEPT_PRIMVARS:
                continue
            if property.startswith("primvars:") or property in _BAKED:
                prim.RemoveProperty(property)
        sidecar = write_sidecar(path, pxr)
        relative = os.path.relpath(path, out_dir)
        prim.GetReferences().ClearReferences()
        prim.GetReferences().AddReference(os.path.relpath(sidecar, out_dir))
        digest = hash_of_file(str(path))
        info = prim.GetAssetInfo() or {}
        info["identifier"] = pxr.Sdf.AssetPath(relative)
        if digest:
            info["version"] = digest
        prim.SetAssetInfo(info)
        done += 1
    return done, skipped


def paint_from_materials(stage: Any, pxr: Any) -> int:
    """State a mesh's real colour where the simplest consumer looks for it.

    The converter binds a UsdShade Material per MJCF material -- a G1's black parts and its
    silver ones are stated there -- and then writes ``primvars:displayColor = (0.5, 0.5, 0.5)``
    on every visual mesh as well. USD says the bound material wins, but displayColor is what a
    reader with no shading graph reads, and ours takes it first: the whole robot came out in
    that placeholder grey. So where a mesh binds a material that states a colour, the mesh says
    the same colour, and no consumer has to choose.
    """
    painted = 0
    for prim in list(stage.Traverse()):
        if not prim.IsA(pxr.UsdGeom.Mesh):
            continue
        bound = prim.GetRelationship("material:binding")
        targets = bound.GetTargets() if bound else []
        if not targets:
            continue
        material = stage.GetPrimAtPath(targets[0])
        color = material.GetAttribute("inputs:color").Get() if material else None
        if color is None:
            continue
        pxr.UsdGeom.Gprim(prim).CreateDisplayColorAttr([pxr.Gf.Vec3f(*color)])
        painted += 1
    return painted


def name_joints(stage: Any, pxr: Any) -> int:
    renamed = 0
    for prim in list(stage.Traverse()):
        if not prim.IsA(pxr.UsdPhysics.Joint) or prim.GetName() != prim.GetTypeName():
            continue
        targets = prim.GetRelationship("physics:body1").GetTargets()
        if not targets:
            logger.warning(f"{prim.GetPath()} is unnamed and moves nothing; left as it is")
            continue
        child = str(targets[0]).rsplit("/", 1)[-1]
        edit = pxr.Sdf.NamespaceEdit.Rename(prim.GetPath(), f"{child}_joint")
        batch = pxr.Sdf.BatchNamespaceEdit()
        batch.Add(edit)
        if stage.GetRootLayer().Apply(batch):
            renamed += 1
        else:
            logger.warning(f"could not rename {prim.GetPath()} to {child}_joint")
    return renamed


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

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    pxr = argparse.Namespace(Gf=Gf, Sdf=Sdf, Usd=Usd, UsdGeom=UsdGeom, UsdPhysics=UsdPhysics)

    for path in (args.mjcf, args.converted):
        if not path.is_file():
            print(f"ERROR: no file at {path}", file=sys.stderr)
            return 1

    files = mesh_files(args.mjcf)

    stage = Usd.Stage.Open(str(args.converted))
    flat = Usd.Stage.Open(stage.Flatten())
    baked = any(prim.IsA(UsdGeom.Mesh) for prim in flat.Traverse())
    if baked and not files:
        print(
            f"ERROR: {args.converted} has meshes and {args.mjcf} names none; "
            f"is --mjcf the file it was converted from?",
            file=sys.stderr,
        )
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    named = name_joints(flat, pxr)
    painted = paint_from_materials(flat, pxr)
    done, skipped = rereference(flat, files, args.meshes, args.out.parent.resolve(), pxr)

    flat.GetRootLayer().Export(str(args.out))
    size = args.out.stat().st_size
    print(f"{done} mesh(es) re-pointed with a .usdc sidecar, {skipped} left baked")
    print(f"{named} unnamed joint(s) named after the body they move")
    print(f"{painted} mesh(es) given the colour of the material they bind")
    print(f"wrote {args.out} ({size / 1e6:.2f} MB)")
    if skipped:
        print("note: a baked mesh keeps the layer large; see the warnings above for why")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
