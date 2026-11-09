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

from dimos.robot.assets.usd import hash_of_file
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The geometry a referenced mesh file supplies for itself.
_BAKED = frozenset({"points", "faceVertexIndices", "faceVertexCounts", "normals", "extent"})
# The two primvars that are the mesh's appearance rather than its geometry. One value per prim,
# not per vertex, so keeping them costs nothing -- and dropping them left every piece of
# office_555's furniture at MuJoCo's default grey, because the MJCF gives those geoms an rgba
# the exporter writes here and the sidecar does not carry.
_KEPT_PRIMVARS = frozenset({"primvars:displayColor", "primvars:displayOpacity"})
# What a sidecar is written as: binary, because nothing hands these around as text -- they are
# fetched as bytes, one file per mesh, exactly like the STL beside them.
_SIDECAR_SUFFIX = ".usdc"


def write_sidecar(source: Path, pxr: Any) -> Path:
    """``source`` as a one-prim USD layer beside it, so a reference to it is valid USD.

    Rewritten every run rather than cached: a stale sidecar is a robot whose geometry silently
    does not match its meshes.
    """
    import trimesh

    out = source.with_suffix(_SIDECAR_SUFFIX)
    loaded = trimesh.load_mesh(str(source), process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.to_geometry()
    if len(loaded.faces) == 0:
        # Refusing beats writing an empty sidecar. A git-lfs **pointer** parses as a mesh with no
        # faces, so running this against an unmaterialised worktree produced 427 identical 737-byte
        # layers and the scene died much later inside MuJoCo as "at least 4 vertices required".
        raise ValueError(
            f"{source} has no faces. If it is {source.stat().st_size} bytes it is a git-lfs "
            f"pointer: run `git lfs checkout` on its directory first."
        )

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

    Three spellings, because the converter names a mesh prim after whichever the MJCF gave it:

    * the **mesh asset's** own name, which is what a robot MJCF uses;
    * the **file's stem**, for a `<mesh file=...>` with no name -- MuJoCo's rule, and how the Go2
      declares all sixteen of its meshes. Reading only the explicit spelling found one;
    * the **geom's** name, which is what a scene MJCF uses (office_555 names every geom and has
      them reference meshes called `asset_0`, `asset_1`, ...).

    Without the geom spelling the office scene re-points nothing: 3547 meshes stay baked and the
    layer comes out 35 MB.
    """
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
    """The mesh file a prim's name refers to, allowing for the converter's de-duplication: two
    geoms on one body using the same mesh become two prims, the second named ``<mesh>_1``. Without
    this every such prim stays baked, which on the G1 is 23 of 58.
    """
    if prim_name in files:
        return files[prim_name]
    stripped = re.sub(r"_\d+$", "", prim_name)
    return files.get(stripped) if stripped != prim_name else None


def rereference(
    stage: Any, files: dict[str, str], meshes: Path, out_dir: Path, pxr: Any
) -> tuple[int, int]:
    """Swap each baked mesh for a reference to its file, with the file's hash beside it. Returns
    how many were re-pointed and how many were left alone.

    A mesh the MJCF does not name is left baked rather than dropped: a hole in the robot is worse
    than a large layer. References are written **relative to the layer**, because the layer is
    committed and the path it was converted at means nothing on anyone else's machine.
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
        # --meshes is authoritative, so a `file=` carrying a directory is matched by its basename
        # rather than pasted onto the directory a second time.
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
        # Everything per-vertex goes: the reference supplies it, and it is what made the layer large.
        # primvars included -- the converter bakes normals there. Transform ops, material bindings and
        # the mesh's own colour stay, since the file carries none of them.
        for property in list(prim.GetPropertyNames()):
            if property.startswith(("xformOp", "material:")) or property in _KEPT_PRIMVARS:
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


def name_joints(stage: Any, pxr: Any) -> int:
    """Give every unnamed joint prim the name of the body it moves. Returns how many.

    The converter leaves a joint it was not given a name for as the bare schema name, so a scene of
    loose furniture comes out with **every** weld called `PhysicsFixedJoint` -- office_555 has 448
    of them. Legal USD, but useless as a frame source: `usd_frame_names` returns one name for 448
    edges, and a URDF converted from it has 448 joints of the same name and will not load.

    `<child>_joint` matches what the robot layers already carry (`left_ankle_roll_joint`).
    """
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

    # Flattened, because the converter writes a root that payloads several binary layers and a
    # scene has to leave here as one text layer.
    stage = Usd.Stage.Open(str(args.converted))
    flat = Usd.Stage.Open(stage.Flatten())
    # A model of boxes and spheres names no meshes, which is fine. A stage full of them whose MJCF
    # names none is the --mjcf being the wrong file, and re-pointing nothing would quietly write a
    # layer of baked geometry.
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
    done, skipped = rereference(flat, files, args.meshes, args.out.parent.resolve(), pxr)

    # The ROOT LAYER, not the stage: Stage.Export() flattens composition, which resolves the
    # reference arcs we just authored back into baked geometry -- the opposite of the point.
    flat.GetRootLayer().Export(str(args.out))
    size = args.out.stat().st_size
    print(f"{done} mesh(es) re-pointed with a .usdc sidecar, {skipped} left baked")
    print(f"{named} unnamed joint(s) named after the body they move")
    print(f"wrote {args.out} ({size / 1e6:.2f} MB)")
    if skipped:
        print("note: a baked mesh keeps the layer large; see the warnings above for why")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
