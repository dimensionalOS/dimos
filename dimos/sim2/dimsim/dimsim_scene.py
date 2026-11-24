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

"""A DimSim scene (misc/DimSim/scenes/<name>) as USD: its structure, and each object with a variant per state.

An object with actions carries its state machine in customData ``interactive``: ``states`` (state
id -> name) and ``actions`` (action id -> label, from, to); its variant set ``state`` is its state now.
One that can be picked up has customData ``pickable``; every object has its DimSim ``title``.
"""

import hashlib
import io
import json
import os
from pathlib import Path
import re
from typing import Any

import numpy as np

from dimos.constants import CACHE_DIR, DIMOS_PROJECT_ROOT

DIMSIM_SCENES = DIMOS_PROJECT_ROOT / "misc" / "DimSim" / "scenes"
INTERACTIVE = "interactive"
PICKABLE = "pickable"
TITLE = "title"
STATE = "state"
# Loaded as scene content, so it is this user's alone: see _private().
_MESHES = CACHE_DIR / "dimsim-meshes"
_LFS_POINTER = b"version https://git-lfs.github.com/spec/v1"
# DimSim's scenes are three.js, y up: this rotation (120 degrees about x+y+z) takes its x, y, z to our y, z, x.
_Y_UP_TO_Z_UP = (0.5, 0.5, 0.5, 0.5)


def dimsim_scene_usd(name: str) -> str:
    """The DimSim scene ``name`` as a z-up USD layer whose root prim is ``name``."""
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    pxr = load_pxr()
    scene = DIMSIM_SCENES / name
    if not (scene / "index.js").is_file():
        raise FileNotFoundError(f"no DimSim scene {name!r} at {scene}")
    stage = pxr.Usd.Stage.CreateInMemory()
    pxr.UsdGeom.SetStageUpAxis(stage, pxr.UsdGeom.Tokens.z)
    pxr.UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(pxr.UsdGeom.Xform.Define(stage, f"/{name}").GetPrim())
    y_up = f"/{name}/y_up"
    pxr.UsdGeom.Xform.Define(stage, y_up).AddOrientOp().Set(pxr.Gf.Quatf(*_Y_UP_TO_Z_UP))
    structure = scene / "structure.glb"
    if structure.is_file():
        _reference(pxr, stage, f"{y_up}/structure", _mesh_file(pxr, structure, name))
    manifest = scene / "objects" / "manifest.json"
    entries = json.loads(manifest.read_text()) if manifest.is_file() else []
    taken: set[str] = set()
    for entry in entries:
        _add_object(pxr, stage, f"{y_up}/objects/{_prim_name(entry['title'], taken)}", entry, name)
    return str(stage.GetRootLayer().ExportToString())


def _add_object(pxr: Any, stage: Any, path: str, entry: dict[str, Any], scene: str) -> None:
    placed = entry["transform"]
    xform = pxr.UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(pxr.Gf.Vec3d(*_xyz(placed["position"])))
    # three.js turns by its Euler "XYZ" order, which is USD's rotateZYX.
    xform.AddRotateZYXOp().Set(pxr.Gf.Vec3f(*np.degrees(_xyz(placed["rotation"])).tolist()))
    xform.AddScaleOp().Set(pxr.Gf.Vec3f(*_xyz(placed["scale"])))
    prim = xform.GetPrim()
    variants = prim.GetVariantSets().AddVariantSet(STATE)
    for state in entry["states"]:
        variants.AddVariant(state["id"])
        variants.SetVariantSelection(state["id"])
        glb = DIMSIM_SCENES / scene / "objects" / state["file"]
        with variants.GetVariantEditContext():
            _reference(pxr, stage, f"{path}/geometry", _mesh_file(pxr, glb, scene))
    variants.SetVariantSelection(entry["currentStateId"])
    prim.SetCustomDataByKey(TITLE, entry["title"])
    if entry.get("pickable"):
        prim.SetCustomDataByKey(PICKABLE, True)
    if entry.get("actions"):
        prim.SetCustomDataByKey(
            INTERACTIVE,
            {
                "states": {state["id"]: state["name"] for state in entry["states"]},
                "actions": {
                    action["id"]: {
                        "label": action["label"],
                        "from": action["from"],
                        "to": action["to"],
                    }
                    for action in entry["actions"]
                },
            },
        )


def _reference(pxr: Any, stage: Any, path: str, layer: Path) -> None:
    stage.DefinePrim(path, "Xform").GetReferences().AddReference(str(layer))


def _mesh_file(pxr: Any, glb: Path, scene: str) -> Path:
    """The GLB's meshes as a cached USD layer, one Mesh per GLB mesh node, each in the GLB's own frame and material."""
    data = glb.read_bytes()
    if data.startswith(_LFS_POINTER):
        raise FileNotFoundError(
            f"{glb} is a Git LFS pointer, not the model; fetch the scene with "
            f'git lfs pull --include="misc/DimSim/scenes/{scene}/**"'
        )
    out = _private(_MESHES) / f"{hashlib.sha256(data).hexdigest()[:16]}.usdc"
    if out.exists():
        return out
    from pxr import UsdShade
    import trimesh

    loaded = trimesh.load_scene(io.BytesIO(data), file_type="glb")
    stage = pxr.Usd.Stage.CreateInMemory()
    stage.SetDefaultPrim(pxr.UsdGeom.Xform.Define(stage, "/geometry").GetPrim())
    materials: dict[int, Any] = {}
    for index, node in enumerate(loaded.graph.nodes_geometry):
        transform, geometry = loaded.graph.get(node)
        mesh = loaded.geometry[geometry]
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            continue
        points = trimesh.transform_points(mesh.vertices, np.asarray(transform)).astype(np.float32)
        part = pxr.UsdGeom.Mesh.Define(stage, f"/geometry/part_{index}")
        part.GetPointsAttr().Set(pxr.Vt.Vec3fArray.FromNumpy(points))
        part.GetFaceVertexCountsAttr().Set(pxr.Vt.IntArray([3] * len(mesh.faces)))
        part.GetFaceVertexIndicesAttr().Set(
            pxr.Vt.IntArray.FromNumpy(mesh.faces.astype(np.int32).ravel())
        )
        _shell(pxr, part.GetPrim())
        visual = mesh.visual
        # A GLB's mesh has a glTF material: trimesh reads it as a PBRMaterial on TextureVisuals.
        if isinstance(visual, trimesh.visual.TextureVisuals) and visual.material is not None:
            if visual.uv is not None:
                uv = pxr.UsdGeom.PrimvarsAPI(part).CreatePrimvar(
                    "st", pxr.Sdf.ValueTypeNames.TexCoord2fArray, pxr.UsdGeom.Tokens.vertex
                )
                uv.Set(pxr.Vt.Vec2fArray.FromNumpy(visual.uv.astype(np.float32)))
            source = visual.material
            if id(source) not in materials:
                materials[id(source)] = _material(
                    pxr, stage, f"/geometry/material_{len(materials)}", source
                )
            UsdShade.MaterialBindingAPI.Apply(part.GetPrim()).Bind(materials[id(source)])
    partial = out.with_suffix(f".{os.getpid()}.usdc")
    stage.GetRootLayer().Export(str(partial))
    partial.replace(out)
    return out


def _material(pxr: Any, stage: Any, path: str, pbr: Any) -> Any:
    """The GLB's PBR material as a UsdPreviewSurface: its base color (by factor, or texture times factor) and alpha."""
    from pxr import UsdShade

    Shader, Types = UsdShade.Shader, pxr.Sdf.ValueTypeNames
    material = UsdShade.Material.Define(stage, path)
    surface = Shader.Define(stage, f"{path}/surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    # glTF's defaults, for what the GLB leaves out.
    factor = np.ones(4) if pbr.baseColorFactor is None else np.asarray(pbr.baseColorFactor) / 255
    metallic = 1.0 if pbr.metallicFactor is None else float(pbr.metallicFactor)
    roughness = 1.0 if pbr.roughnessFactor is None else float(pbr.roughnessFactor)
    surface.CreateInput("metallic", Types.Float).Set(metallic)
    surface.CreateInput("roughness", Types.Float).Set(roughness)
    surface.CreateInput("opacity", Types.Float).Set(float(factor[3]))
    color = surface.CreateInput("diffuseColor", Types.Color3f)
    if pbr.baseColorTexture is None:
        color.Set(pxr.Gf.Vec3f(*factor[:3].tolist()))
    else:
        reader = Shader.Define(stage, f"{path}/st")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Types.Token).Set("st")
        texture = Shader.Define(stage, f"{path}/texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Types.Asset).Set(_image_file(pbr.baseColorTexture).name)
        texture.CreateInput("scale", Types.Float4).Set(pxr.Gf.Vec4f(*factor.tolist()))
        texture.CreateInput("st", Types.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        color.ConnectToSource(texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    return material


def _image_file(image: Any) -> Path:
    """The texture as a PNG beside the mesh layers, which name it relative to themselves."""
    pixels = io.BytesIO()
    image.save(pixels, format="PNG")
    data = pixels.getvalue()
    out = _MESHES / f"{hashlib.sha256(data).hexdigest()[:16]}.png"
    if not out.exists():
        partial = out.with_suffix(f".{os.getpid()}.png")
        partial.write_bytes(data)
        partial.replace(out)
    return out


def _private(directory: Path) -> Path:
    """The directory, made if missing; refused if another user owns it or anyone else can write to it."""
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    held = directory.stat()
    if held.st_uid != os.getuid() or held.st_mode & 0o022:
        raise PermissionError(
            f"{directory} holds meshes loaded into the scene, and someone else could put their own"
            f" there: it must be yours and writable by you alone (chmod go-w), or removed"
        )
    return directory


def _shell(pxr: Any, prim: Any) -> None:
    """Weigh the mesh as a shell: MuJoCo (the sensors' renderer) can hull no flat part, and a room has many.

    It carries no collision API, so MuJoCo draws and ray-casts it without hulling it; DimSim collides with its triangles.
    """
    prim.AddAppliedSchema("NewtonMassAPI")
    prim.CreateAttribute(
        "newton:massModel", pxr.Sdf.ValueTypeNames.Token, variability=pxr.Sdf.VariabilityUniform
    ).Set("shell")


def _prim_name(title: str, taken: set[str]) -> str:
    base = re.sub(r"\W+", "_", title.lower()).strip("_") or "object"
    base = base if base[0].isalpha() else f"object_{base}"
    name, count = base, 0
    while name in taken:
        count += 1
        name = f"{base}_{count}"
    taken.add(name)
    return name


def _xyz(value: dict[str, float]) -> tuple[float, float, float]:
    return float(value["x"]), float(value["y"]), float(value["z"])
