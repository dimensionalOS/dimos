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

"""Author and export a small native-asset proof, not a general Unreal importer.

Run in an isolated DimosRenderer editor project using -ExecutePythonScript and
a real graphics backend, with the user's copied Konbini content at /Game/Konbini.
Use -RenderOffscreen when a visible editor is unwanted. Do not use -nullrhi:
UE 5.7.4 crashed inside its viewport hit-proxy code when spawning the first actor.
Set DIMOS_NATIVE_SCENE_OUTPUT to the generated scene directory. Optionally set
DIMOS_NATIVE_SOURCE_PROJECT to verify the original assets before and after.

Only /Game/DimosTests/KonbiniNativeProof is saved. Reruns replace its five known
DimOS-tagged demo actors and retain its lights. The native shelf's existing
solid outer collider is preserved: this demo does not make shelf openings usable.
Robot placement is a spawn reference; no robot model or controller is duplicated.
Mass comes from the authored DimOS tag; friction is an explicit demo default.
Neither is imported from Unreal physical materials.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import os
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import unreal

MAP_PATH = "/Game/DimosTests/KonbiniNativeProof"
SHELF_PATH = "/Game/Konbini/Assets/Shelves/SM_Shelf_01"
PRODUCT_PATH = "/Game/Konbini/Assets/StoreProducts/Cereals/SM_CerealBox_01"
CUBE_PATH = "/Engine/BasicShapes/Cube"


def _numbers(values: Any, count: int) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f"Expected {count} finite numbers: {values!r}")
    return result


def _quaternion(values: Any) -> list[float]:
    result = _numbers(values, 4)
    if abs(sum(value * value for value in result) - 1.0) > 1e-5:
        raise ValueError(f"Non-unit rotation is unsupported: {result}")
    return result


def _scale(values: Any) -> list[float]:
    result = _numbers(values, 3)
    if min(result) <= 0:
        raise ValueError("Reflected or zero scales are unsupported by this demo")
    return result


def _rotate(point: list[float], quaternion: list[float]) -> list[float]:
    x, y, z, w = quaternion
    px, py, pz = point
    tx, ty, tz = 2 * (y * pz - z * py), 2 * (z * px - x * pz), 2 * (x * py - y * px)
    return [
        px + w * tx + y * tz - z * ty,
        py + w * ty + z * tx - x * tz,
        pz + w * tz + x * ty - y * tx,
    ]


def _mj_position(values: list[float]) -> list[float]:
    return [values[0] / 100, -values[1] / 100, values[2] / 100]


def _mj_quaternion(values: list[float]) -> list[float]:
    # Reflection through Y maps the quaternion vector to (-x, y, -z).
    x, y, z, w = values
    return [w, -x, y, -z]


def _text(values: Any) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _transform(transform: Any) -> dict[str, list[float]]:
    translation, rotation, scale = transform.translation, transform.rotation, transform.scale3d
    return {
        "translation": _numbers([translation.x, translation.y, translation.z], 3),
        "rotation_xyzw": _quaternion([rotation.x, rotation.y, rotation.z, rotation.w]),
        "scale": _scale([scale.x, scale.y, scale.z]),
    }


def _collision_vertices(
    collider: dict[str, Any], component_scale: list[float]
) -> list[list[float]]:
    """Bake exact box/convex element transforms and scale into body-local metres."""
    if collider["type"] == "convex":
        points = [_numbers(point, 3) for point in collider["vertices"]]
    elif collider["type"] == "box":
        size = _scale(collider["size"])
        points = [
            [sign[axis] * size[axis] / 2 for axis in range(3)]
            for sign in itertools.product((-1, 1), repeat=3)
        ]
    else:
        raise ValueError(f"This bounded demo supports box/convex collision, got {collider['type']}")
    if len(points) < 4:
        raise ValueError("A collision hull needs at least four vertices")
    transform = collider["transform"]
    translation = _numbers(transform["translation"], 3)
    rotation = _quaternion(transform["rotation_xyzw"])
    element_scale = _scale(transform["scale"])
    result = []
    for point in points:
        rotated = _rotate([point[axis] * element_scale[axis] for axis in range(3)], rotation)
        result.append(
            _mj_position(
                [(rotated[axis] + translation[axis]) * component_scale[axis] for axis in range(3)]
            )
        )
    return result


def _tags(actor: Any) -> dict[str, str]:
    return dict(
        str(tag).split("=", 1)
        for tag in actor.get_editor_property("tags")
        if str(tag).startswith("dimos.") and "=" in str(tag)
    )


def _spawn_mesh(
    subsystem: Any,
    identity: str,
    asset_path: str,
    position: tuple[float, ...],
    scale: tuple[float, ...] = (1, 1, 1),
    yaw: float = 0,
    mass: float | None = None,
) -> Any:
    mesh = unreal.load_asset(asset_path)
    if mesh is None:
        raise RuntimeError(f"Missing source asset {asset_path}; mount copied Konbini content first")
    actor = subsystem.spawn_actor_from_class(
        unreal.StaticMeshActor, unreal.Vector(*position), unreal.Rotator(yaw=yaw)
    )
    actor.set_actor_label(f"DimOS {identity}")
    component = actor.static_mesh_component
    component.set_mobility(unreal.ComponentMobility.MOVABLE)
    component.set_static_mesh(mesh)
    component.set_simulate_physics(False)
    actor.set_actor_scale3d(unreal.Vector(*scale))
    tags = [f"dimos.id={identity}", f"dimos.role={'dynamic' if mass else 'fixed'}"]
    if mass is not None:
        tags.append(f"dimos.mass_kg={mass}")
    actor.set_editor_property("tags", [unreal.Name(tag) for tag in tags])
    return actor


def _source_hashes() -> dict[str, str] | None:
    source = os.environ.get("DIMOS_NATIVE_SOURCE_PROJECT")
    if not source:
        return None
    root = Path(source).expanduser().resolve()
    if not (root / "Content" / "Konbini").is_dir():
        raise ValueError("DIMOS_NATIVE_SOURCE_PROJECT must contain Content/Konbini")
    result = {}
    for path in sorted((root / "Content" / "Konbini").rglob("*")):
        if path.is_file():
            with path.open("rb") as stream:
                result[str(path.relative_to(root))] = hashlib.file_digest(
                    stream, "sha256"
                ).hexdigest()
    return result


def _export(actors: list[Any], spawn: Any, output: Path) -> dict[str, Any]:
    root = ET.Element("mujoco", model="konbini_native_proof")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 -9.81")
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    native: dict[str, Any] = {"version": 1, "geoms": {}, "hidden_geoms": []}
    entities: dict[str, Any] = {}
    audit = []
    for actor in actors:
        tags = _tags(actor)
        identity = tags["dimos.id"]
        movable = tags["dimos.role"] == "dynamic"
        component = actor.static_mesh_component
        transform = _transform(component.get_world_transform())
        description = json.loads(
            unreal.DimosAssetLibrary.describe_static_mesh(component.static_mesh)
        )
        if description.get("error") or description.get("unsupported_collision_count", 0):
            raise ValueError(f"Unsupported source collision for {identity}: {description}")
        if description.get("collision_trace") == "CTF_UseComplexAsSimple":
            raise ValueError(
                f"{identity} uses complex triangle collision as simple collision; "
                "this demo cannot substitute its simple hulls. Author suitable simple collision first."
            )
        colliders = description.get("colliders", [])
        if not colliders or (movable and len(colliders) != 1):
            raise ValueError("This demo requires collision and exactly one hull for a dynamic prop")
        body = ET.SubElement(
            world,
            "body",
            name=identity,
            pos=_text(_mj_position(transform["translation"])),
            quat=_text(_mj_quaternion(transform["rotation_xyzw"])),
        )
        if movable:
            ET.SubElement(body, "freejoint", name=f"{identity}_free")
        marker = f"unreal_{identity}_visual"
        ET.SubElement(
            body,
            "geom",
            name=marker,
            type="sphere",
            size="0.000001",
            mass="0",
            contype="0",
            conaffinity="0",
            rgba="1 1 1 1",
        )
        native["geoms"][marker] = {"asset": description["asset_path"], "scale": transform["scale"]}
        for index, collider in enumerate(colliders):
            name = f"{identity}_collision_{index}"
            vertices = _collision_vertices(collider, transform["scale"])
            ET.SubElement(
                asset, "mesh", name=name, vertex=_text(itertools.chain.from_iterable(vertices))
            )
            attributes = {
                "name": name,
                "type": "mesh",
                "mesh": name,
                "rgba": "0.5 0.5 0.5 1",
                "friction": "0.8 0.005 0.0001",
            }
            if movable:
                mass = float(tags["dimos.mass_kg"])
                if not math.isfinite(mass) or mass <= 0:
                    raise ValueError("Dynamic mass must be positive and finite")
                attributes["mass"] = str(mass)
            ET.SubElement(body, "geom", **attributes)
            native["hidden_geoms"].append(name)
            if identity == "floor":
                if (
                    abs(max(point[2] for point in vertices) + transform["translation"][2] / 100)
                    > 1e-6
                ):
                    raise ValueError(
                        "The exported floor must have its upper surface exactly at z=0"
                    )
        entities[identity] = {
            "body": identity,
            "label": actor.get_actor_label(),
            "kind": "prop" if movable else "fixture",
            "movable": movable,
        }
        audit.append(
            {
                "id": identity,
                "actor": actor.get_path_name(),
                "transform": transform,
                "source": description,
                "physics_tags": tags,
            }
        )
    custom = ET.SubElement(root, "custom")
    ET.SubElement(
        custom, "text", name="dimos_unreal_assets", data=json.dumps(native, separators=(",", ":"))
    )
    spawn_transform = _transform(spawn.get_actor_transform())
    quaternion = _mj_quaternion(spawn_transform["rotation_xyzw"])
    sidecar = {
        "format": "dimos.scene.v1",
        "id": "konbini-native-proof",
        "entities": entities,
        "spawns": {
            "default": {
                "position": _mj_position(spawn_transform["translation"]),
                "orientation": quaternion[1:] + quaternion[:1],
            }
        },
        "provenance": {
            "authoring_map": MAP_PATH,
            "exporter": "demo_native_scene.py",
            "limitations": [
                "Shelf retains its source solid outer collider; openings are not usable",
                "Only static meshes with box/convex collision; no articulated scene objects",
            ],
            "robot": "Existing G1 model attached separately at the authored spawn",
        },
    }
    output.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(output / "scene.xml", encoding="utf-8", xml_declaration=True)
    (output / "scene.json").write_text(json.dumps(sidecar, indent=2) + "\n")
    return {"map": MAP_PATH, "actors": audit, "native_assets": native, "spawn": spawn_transform}


def main() -> None:
    output = Path(os.environ["DIMOS_NATIVE_SCENE_OUTPUT"]).expanduser().resolve()
    before = _source_hashes()
    level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if level is None or actors is None:
        raise RuntimeError("Use -ExecutePythonScript=... editor mode, not -run=pythonscript")
    existing = unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH)
    if existing:
        if not level.load_level(MAP_PATH):
            raise RuntimeError(f"Could not load owned scratch map {MAP_PATH}")
        owned_ids = {"floor", "back_wall", "shelf", "product", "g1_spawn"}
        for actor in actors.get_all_level_actors():
            if (
                actor.get_path_name().startswith(MAP_PATH + ".")
                and _tags(actor).get("dimos.id") in owned_ids
            ):
                if not actors.destroy_actor(actor):
                    raise RuntimeError(
                        f"Could not replace owned demo actor {actor.get_path_name()}"
                    )
    elif not level.new_level(MAP_PATH):
        raise RuntimeError(f"Could not create scratch map {MAP_PATH}")
    placed = [
        _spawn_mesh(actors, "floor", CUBE_PATH, (0, 0, -5), (6, 6, 0.1)),
        _spawn_mesh(actors, "back_wall", CUBE_PATH, (-250, 0, 130), (0.1, 6, 2.6)),
        _spawn_mesh(actors, "shelf", SHELF_PATH, (-110, -130, 0), yaw=-45),
        _spawn_mesh(actors, "product", PRODUCT_PATH, (75, 35, 35), yaw=-45, mass=0.3),
    ]
    spawn = actors.spawn_actor_from_class(unreal.TargetPoint, unreal.Vector(0, 0, 0))
    spawn.set_actor_label("DimOS G1 spawn (existing robot model)")
    spawn.set_editor_property(
        "tags", [unreal.Name("dimos.id=g1_spawn"), unreal.Name("dimos.role=spawn")]
    )
    if not existing:
        actors.spawn_actor_from_class(
            unreal.DirectionalLight, unreal.Vector(0, 0, 300), unreal.Rotator(pitch=-50, yaw=-30)
        )
        actors.spawn_actor_from_class(unreal.SkyLight, unreal.Vector(0, 0, 250))
    if not level.save_current_level():
        raise RuntimeError(f"Could not save authored scratch map {MAP_PATH}")
    audit = _export(placed, spawn, output)
    after = _source_hashes()
    if before != after:
        raise RuntimeError("Original source asset hashes changed during export")
    audit["original_hashes_verified"] = before is not None
    audit["original_asset_count"] = len(before) if before is not None else None
    (output / "native-scene-audit.json").write_text(json.dumps(audit, indent=2) + "\n")
    unreal.log(f"DIMOS_NATIVE_SCENE_READY {output / 'scene.xml'}")


if __name__ == "__main__":
    main()
