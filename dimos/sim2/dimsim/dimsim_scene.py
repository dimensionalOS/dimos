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

"""A DimSim scene (misc/DimSim/scenes/<name>) as USD: its structure, and each object as it is in its current state."""

import hashlib
import io
import json
from pathlib import Path
import re
import struct
from typing import Any

import numpy as np

from dimos.constants import CACHE_DIR, DIMOS_PROJECT_ROOT
from dimos.robot.assets.usd_utils.assets import write_asset_info
from dimos.robot.assets.usd_utils.lfs import pull_lfs_files
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DIMSIM_SCENES = DIMOS_PROJECT_ROOT / "misc" / "DimSim" / "scenes"
_MESHES = CACHE_DIR / "dimsim-meshes"
# DimSim's scenes are three.js, y up: this rotation (120 degrees about x+y+z) takes its x, y, z to our y, z, x.
_Y_UP_TO_Z_UP = (0.5, 0.5, 0.5, 0.5)
# A SphereLight of this radius shows a disc of area 1, so its radiance is a three.js PointLight's intensity.
_POINT_RADIUS = 1 / np.sqrt(np.pi)
# The lights DimSim's engine adds to every scene (src/engine.js), as a scene's index.js would write them.
_ENGINE_LIGHTS = """
scene.add(new THREE.AmbientLight(0xffffff, 0.65));
scene.add(new THREE.HemisphereLight(0xffffff, 0x223344, 0.85));
const dir = new THREE.DirectionalLight(0xffffff, 1.6);
dir.position.set(8, 14, 6);
"""
_NUMBER = r"-?(?:\d+\.?\d*|\.\d+)"
_LIGHT = re.compile(
    r"(?:(?:const|let|var)\s+(\w+)\s*=\s*|scene\.add\()new THREE\.(\w+Light)\(([^;]*?)\)\)?;"
)


def dimsim_scene_usd(name: str, *, environment: float, sun_per_intensity: float) -> str:
    """The DimSim scene ``name`` as a z-up USD layer whose root prim is ``name``; see _light for the rest."""
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
    manifest = scene / "objects" / "manifest.json"
    entries = json.loads(manifest.read_text()) if manifest.is_file() else []
    # One pull for every model the scene uses that is still a Git LFS pointer, before any is read.
    pull_lfs_files(
        [
            structure,
            *(scene / "objects" / _current_file(entry) for entry in entries),
        ]
    )
    if structure.is_file():
        _add_geometry(pxr, stage, f"{y_up}/structure", structure)
    _light(pxr, stage, y_up, (scene / "index.js").read_text(), environment, sun_per_intensity)
    taken: set[str] = set()
    for entry in entries:
        _add_object(pxr, stage, f"{y_up}/objects/{_prim_name(entry['title'], taken)}", entry, scene)
    return str(stage.GetRootLayer().ExportToString())


def _light(
    pxr: Any, stage: Any, y_up: str, index_js: str, environment: float, sun_per_intensity: float
) -> None:
    """DimSim's lighting, its engine's and the scene's own (index.js), as UsdLux lights.

    The lights are DimSim's three.js ones in UsdLux units: a PointLight as a SphereLight of the same radiance, a
    DirectionalLight as a normalized DistantLight, the ambient lights as one DomeLight (a hemisphere light as an
    ambient one and a light from straight above, see _three_lights). Two things DimSim lights with have no USD
    light: its environment map, stood in for by ``environment`` radiance on the dome, and the headlamp on its camera,
    left out. A viewer reading UsdLux as Blender does multiplies a normalized DistantLight's intensity by
    ``sun_per_intensity``, so a DirectionalLight's is divided by it.
    Nothing casts a shadow unless the scene turns shadows on, as in DimSim.
    """
    from pxr import UsdLux

    shadows = "enableShadows()" in index_js
    ambient = np.full(3, environment * np.pi)
    for number, (kind, colour, intensity, at, toward, shadow) in enumerate(
        _three_lights(_ENGINE_LIGHTS + index_js)
    ):
        path = f"{y_up}/lights/{kind.removesuffix('Light').lower()}_{number}"
        if intensity <= 0:
            continue
        if kind == "AmbientLight":
            ambient += colour * intensity
            continue
        if kind == "PointLight":
            light = UsdLux.SphereLight.Define(stage, path)
            light.CreateRadiusAttr(float(_POINT_RADIUS))
            light.CreateIntensityAttr(float(intensity))
            light.AddTranslateOp().Set(pxr.Gf.Vec3d(*at))
        else:
            light = UsdLux.DistantLight.Define(stage, path)
            light.CreateNormalizeAttr(True)
            light.CreateIntensityAttr(float(intensity / sun_per_intensity))
            # a UsdLux light shines down its -Z
            turn = pxr.Gf.Rotation(
                pxr.Gf.Vec3d(0, 0, -1), pxr.Gf.Vec3d(*np.subtract(toward, at, dtype=float).tolist())
            )
            light.AddOrientOp().Set(pxr.Gf.Quatf(turn.GetQuat()))
        light.CreateColorAttr(pxr.Gf.Vec3f(*colour.tolist()))
        UsdLux.ShadowAPI.Apply(light.GetPrim()).CreateShadowEnableAttr(shadows and shadow)
    dome = UsdLux.DomeLight.Define(stage, f"{y_up}/lights/ambient")
    # a uniform dome of radiance L lights a surface as an ambient light of pi * L does
    dome.CreateIntensityAttr(float(ambient.max() / np.pi))
    dome.CreateColorAttr(pxr.Gf.Vec3f(*(ambient / ambient.max()).tolist()))


def _three_lights(source: str) -> list[tuple[str, Any, float, Any, Any, bool]]:
    """The three.js lights a scene's source makes with literal values: kind, linear colour, intensity, position, target, shadow.

    A light placed by a value worked out at run time (in a loop, say) is left out.
    """
    lights: list[tuple[str, Any, float, Any, Any, bool]] = []
    for name, kind, arguments in _LIGHT.findall(source):
        values = re.findall(rf"0x[0-9a-fA-F]{{6}}|['\"]#[0-9a-fA-F]{{6}}['\"]|{_NUMBER}", arguments)
        colours = [_linear(value) for value in values if not re.fullmatch(_NUMBER, value)]
        numbers = [float(value) for value in values if re.fullmatch(_NUMBER, value)]
        if not colours:
            colours = [np.ones(3)]
        intensity = numbers[0] if numbers else 1.0
        if kind == "HemisphereLight":
            # A face lit by sky and ground in turn as it tips up is lit by their mean from all round, plus half
            # their difference from straight above (exact for every face but those looking down).
            sky, ground = (colours + [np.ones(3)] * 2)[:2]
            lights.append(("AmbientLight", (sky + ground) / 2, intensity, None, None, False))
            above = np.clip(sky - ground, 0, None) / 2
            if above.max() > 0:
                lights.append(
                    (
                        "DirectionalLight",
                        above / above.max(),
                        above.max() * intensity,
                        (0, 1, 0),
                        (0, 0, 0),
                        False,
                    )
                )
            continue
        if kind == "AmbientLight":
            lights.append((kind, colours[0], intensity, None, None, False))
            continue
        at = _position(source, rf"{name}\.position") if name else None
        if at is None or kind not in ("PointLight", "DirectionalLight"):
            logger.debug(
                f"DimSim light {name or kind} has no literal position (or no USD kind): left out"
            )
            continue
        toward = _position(source, rf"{name}\.target\.position") or (0.0, 0.0, 0.0)
        shadow = re.search(rf"{name}\.castShadow\s*=\s*true", source) is not None
        lights.append((kind, colours[0], intensity, at, toward, shadow))
    return lights


def _position(source: str, what: str) -> tuple[float, float, float] | None:
    found = re.search(
        rf"{what}\.set\(\s*({_NUMBER})\s*,\s*({_NUMBER})\s*,\s*({_NUMBER})\s*\)", source
    )
    return None if found is None else (float(found[1]), float(found[2]), float(found[3]))


def _linear(colour: str) -> Any:
    """A three.js colour literal (0xRRGGBB or '#RRGGBB', sRGB) as linear RGB, as three.js lights with it."""
    return _srgb_to_linear(
        np.array([int(colour.strip("'\"#")[-6:][i : i + 2], 16) for i in (0, 2, 4)]) / 255
    )


def _srgb_to_linear(srgb: Any) -> Any:
    return np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)


def _current_file(entry: dict[str, Any]) -> str:
    return str(
        next(state["file"] for state in entry["states"] if state["id"] == entry["currentStateId"])
    )


def _add_object(pxr: Any, stage: Any, path: str, entry: dict[str, Any], scene: Path) -> None:
    placed = entry["transform"]
    xform = pxr.UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(pxr.Gf.Vec3d(*_xyz(placed["position"])))
    # three.js turns by its Euler "XYZ" order, which is USD's rotateZYX.
    xform.AddRotateZYXOp().Set(pxr.Gf.Vec3f(*np.degrees(_xyz(placed["rotation"])).tolist()))
    xform.AddScaleOp().Set(pxr.Gf.Vec3f(*_xyz(placed["scale"])))
    _add_geometry(pxr, stage, f"{path}/geometry", scene / "objects" / _current_file(entry))


def _reference(pxr: Any, stage: Any, path: str, layer: Path) -> Any:
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(str(layer))
    write_asset_info(pxr, prim, str(layer), None)
    return prim


def _add_geometry(pxr: Any, stage: Any, path: str, glb: Path) -> None:
    """The GLB at ``path``: a prim per material, referencing that material's meshes and bound to it here.

    The material is bound in the scene's own layer rather than in the mesh layer, because a viewer that draws
    a referenced layer as one mesh (Controller's) takes its colour from the referencing prim's binding.
    """
    from pxr import UsdShade

    pxr.UsdGeom.Xform.Define(stage, path)
    for index, (layer, look) in enumerate(_mesh_parts(pxr, glb)):
        prim = _reference(pxr, stage, f"{path}/part_{index}", layer)
        if look is not None:
            material = _material(pxr, stage, f"{path}/looks/look_{index}", look)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _mesh_parts(pxr: Any, glb: Path) -> list[tuple[Path, dict[str, Any] | None]]:
    """The GLB's meshes as cached USD layers, one per material, each mesh in the GLB's own frame; with each its material."""
    data = glb.read_bytes()
    key = hashlib.sha256(data).hexdigest()[:16]
    _MESHES.mkdir(parents=True, exist_ok=True)
    manifest = _MESHES / f"{key}.json"
    if manifest.exists():
        return [(_MESHES / name, look) for name, look in json.loads(manifest.read_text())]
    import trimesh

    loaded = trimesh.load_scene(io.BytesIO(data), file_type="glb")
    uv_transforms = _uv_transforms(data)
    # A GLB's mesh has a glTF material: trimesh reads it as a PBRMaterial on TextureVisuals.
    groups: dict[int, tuple[Any, list[tuple[Any, Any, Any]]]] = {}
    for node in loaded.graph.nodes_geometry:
        transform, geometry = loaded.graph.get(node)
        mesh = loaded.geometry[geometry]
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            continue
        visual = mesh.visual
        source = visual.material if isinstance(visual, trimesh.visual.TextureVisuals) else None
        groups.setdefault(id(source), (source, []))[1].append(
            (mesh, transform, uv_transforms.get(node))
        )
    parts: list[tuple[str, dict[str, Any] | None]] = []
    for index, (source, meshes) in enumerate(groups.values()):
        stage = pxr.Usd.Stage.CreateInMemory()
        stage.SetDefaultPrim(pxr.UsdGeom.Xform.Define(stage, "/geometry").GetPrim())
        for number, (mesh, transform, uv_transform) in enumerate(meshes):
            points = trimesh.transform_points(mesh.vertices, np.asarray(transform))
            faces = mesh.faces
            # glTF draws a double-sided material's back faces too; a viewer that culls them would lose an open sheet
            # (a floor whose faces point down), so an open mesh carries its back faces itself.
            if source is not None and source.doubleSided and not mesh.is_watertight:
                faces = np.concatenate([faces, faces[:, ::-1]])
            part = pxr.UsdGeom.Mesh.Define(stage, f"/geometry/mesh_{number}")
            part.GetPointsAttr().Set(pxr.Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
            part.GetFaceVertexCountsAttr().Set(pxr.Vt.IntArray([3] * len(faces)))
            part.GetFaceVertexIndicesAttr().Set(
                pxr.Vt.IntArray.FromNumpy(faces.astype(np.int32).ravel())
            )
            _shell(pxr, part.GetPrim())
            uv = getattr(mesh.visual, "uv", None)
            if uv is not None and uv_transform is not None:
                # trimesh flips v from glTF's top-left origin; the transform is in glTF's
                flipped = (
                    np.column_stack([uv[:, 0], 1 - uv[:, 1], np.ones(len(uv))]) @ uv_transform.T
                )
                uv = np.column_stack([flipped[:, 0], 1 - flipped[:, 1]])
            if source is not None and source.baseColorTexture is not None and uv is not None:
                pxr.UsdGeom.PrimvarsAPI(part).CreatePrimvar(
                    "st", pxr.Sdf.ValueTypeNames.TexCoord2fArray, pxr.UsdGeom.Tokens.vertex
                ).Set(pxr.Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
        # Binary is fine here: the layer names no other file (its material, and so its texture, is the scene's).
        name = f"{key}_{index}.usdc"
        stage.GetRootLayer().Export(str(_MESHES / name))
        parts.append((name, None if source is None else _look(source)))
    manifest.write_text(json.dumps(parts))
    return [(_MESHES / name, look) for name, look in parts]


def _uv_transforms(data: bytes) -> dict[str, Any]:
    """Each glTF node's base color KHR_texture_transform (which trimesh drops), as a 2x3 matrix on glTF's uv, by node name."""
    (length,) = struct.unpack_from("<I", data, 12)
    gltf = json.loads(data[20 : 20 + length])
    materials, meshes = gltf.get("materials", []), gltf.get("meshes", [])
    found: dict[str, Any] = {}
    for node in gltf.get("nodes", []):
        if "mesh" not in node or "name" not in node:
            continue
        material = meshes[node["mesh"]]["primitives"][0].get("material")
        if material is None:
            continue
        texture = materials[material].get("pbrMetallicRoughness", {}).get("baseColorTexture", {})
        moved = texture.get("extensions", {}).get("KHR_texture_transform")
        if moved is None:
            continue
        # glTF's order: translate, then rotate, then scale (applied to uv right to left)
        (u, v), turn, (su, sv) = (
            moved.get("offset", (0, 0)),
            moved.get("rotation", 0),
            moved.get("scale", (1, 1)),
        )
        cos, sin = np.cos(turn), np.sin(turn)
        found[node["name"]] = np.array(
            [[cos * su, sin * sv, u], [-sin * su, cos * sv, v]], dtype=np.float64
        )
    return found


def _look(pbr: Any) -> dict[str, Any]:
    """A glTF PBR material as UsdPreviewSurface inputs, glTF's defaults for what it leaves out."""
    factor = np.ones(4) if pbr.baseColorFactor is None else np.asarray(pbr.baseColorFactor) / 255
    texture = pbr.baseColorTexture
    return {
        # A texture's colour factor is baked into it, so a reader that ignores UsdUVTexture's scale still gets it.
        "color": factor[:3].tolist(),
        "texture": None if texture is None else _image_file(texture, factor[:3]).name,
        "opacity": float(factor[3]),
        "metallic": 1.0 if pbr.metallicFactor is None else float(pbr.metallicFactor),
        "roughness": 1.0 if pbr.roughnessFactor is None else float(pbr.roughnessFactor),
    }


def _material(pxr: Any, stage: Any, path: str, look: dict[str, Any]) -> Any:
    """``look`` (see _look) as a UsdPreviewSurface: its base color (by value, or by texture) and alpha."""
    from pxr import UsdShade

    Shader, Types = UsdShade.Shader, pxr.Sdf.ValueTypeNames
    material = UsdShade.Material.Define(stage, path)
    surface = Shader.Define(stage, f"{path}/surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("metallic", Types.Float).Set(look["metallic"])
    surface.CreateInput("roughness", Types.Float).Set(look["roughness"])
    surface.CreateInput("opacity", Types.Float).Set(look["opacity"])
    color = surface.CreateInput("diffuseColor", Types.Color3f)
    if look["texture"] is None:
        color.Set(pxr.Gf.Vec3f(*look["color"]))
    else:
        reader = Shader.Define(stage, f"{path}/st")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Types.Token).Set("st")
        texture = Shader.Define(stage, f"{path}/texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Types.Asset).Set(str(_MESHES / look["texture"]))
        write_asset_info(pxr, texture.GetPrim(), str(_MESHES / look["texture"]), None)
        texture.CreateInput("st", Types.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        color.ConnectToSource(texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    return material


def _image_file(image: Any, factor: Any) -> Path:
    """The texture times its linear colour factor, as a PNG in the mesh cache."""
    if not np.allclose(factor, 1):
        rgba = np.asarray(image.convert("RGBA"), dtype=np.float64) / 255
        linear = _srgb_to_linear(rgba[..., :3]) * np.asarray(factor)
        rgba[..., :3] = np.where(
            linear <= 0.0031308, linear * 12.92, 1.055 * linear ** (1 / 2.4) - 0.055
        )
        from PIL import Image

        image = Image.fromarray(np.round(np.clip(rgba, 0, 1) * 255).astype(np.uint8), "RGBA")
    pixels = io.BytesIO()
    image.save(pixels, format="PNG")
    data = pixels.getvalue()
    out = _MESHES / f"{hashlib.sha256(data).hexdigest()[:16]}.png"
    if not out.exists():
        out.write_bytes(data)
    return out


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
