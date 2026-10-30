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

"""Export audited Unreal simple collision for a static, natively rendered level.

Run outside Unreal with Python. Input is the JSON produced by the read-only
level audit using DimosAssetLibrary.describe_static_mesh. The authored level
supplies visuals; this file exports only collision, never duplicate actors.

Box and convex element transforms are baked exactly, including reflected and
nonuniform component scale. Curved primitives and complex triangle collision
are rejected. Friction is an explicit simulation setting, not imported from
Unreal physical materials. Movable components require --freeze-movable and must
also remain fixed in the renderer. Physics-simulated components are rejected.
The engine Plane's zero-thickness box requires explicit --surface-thickness;
that finite surface is extruded only below its original upper face.

The original Konbini building and front window each have one solid outer hull
which fills the interior or doorway. Those assets require corrected collision
in the scratch project before export; their visual meshes remain unchanged.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

_BLOCKING_SINGLE_HULLS = {
    "/Game/Konbini/Assets/Geo/SM_KombiniGeo.SM_KombiniGeo",
    "/Game/Konbini/Assets/Geo/SM_FrontWindows_01.SM_FrontWindows_01",
}


def _numbers(values: Any, count: int) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f"Expected {count} finite numbers: {values!r}")
    return result


def _transform(value: dict[str, Any]) -> tuple[tuple[float, ...], ...]:
    translation = _numbers(value["translation"], 3)
    rotation = _numbers(value["rotation_xyzw"], 4)
    scale = _numbers(value["scale"], 3)
    if abs(sum(v * v for v in rotation) - 1.0) > 1e-5:
        raise ValueError("Source rotation must be a unit quaternion")
    if any(v == 0 for v in scale):
        raise ValueError("Zero-scale collision cannot be exported")
    return translation, rotation, scale


def _rotate(point: tuple[float, ...], rotation: tuple[float, ...]) -> tuple[float, ...]:
    x, y, z, w = rotation
    px, py, pz = point
    tx, ty, tz = 2 * (y * pz - z * py), 2 * (z * px - x * pz), 2 * (x * py - y * px)
    return (
        px + w * tx + y * tz - z * ty,
        py + w * ty + z * tx - x * tz,
        pz + w * tz + x * ty - y * tx,
    )


def _position(values: tuple[float, ...]) -> tuple[float, ...]:
    return values[0] / 100, -values[1] / 100, values[2] / 100


def _quaternion(values: tuple[float, ...]) -> tuple[float, ...]:
    x, y, z, w = values
    return w, -x, y, -z


def _text(values: Any) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _identity(value: str) -> str:
    return hashlib.sha256(value.encode()).hexdigest()[:20]


def collision_vertices(
    collider: dict[str, Any],
    component_scale: tuple[float, ...],
    surface_thickness: float | None = None,
) -> list[tuple[float, ...]]:
    """Bake vertices in local metres; surface thickness is an explicit exception."""
    translation, rotation, scale = _transform(collider["transform"])
    surface = False
    if collider["type"] == "convex":
        points = [_numbers(point, 3) for point in collider["vertices"]]
    elif collider["type"] == "box":
        size = _numbers(collider["size"], 3)
        surface = size[2] == 0 and min(size[:2]) > 0
        if surface:
            if (
                surface_thickness is None
                or not math.isfinite(surface_thickness)
                or surface_thickness <= 0
            ):
                raise ValueError(
                    "Zero-thickness boxes require explicit positive --surface-thickness"
                )
        elif min(size) <= 0:
            raise ValueError("Source box dimensions must be positive")
        points = [
            tuple(sign[axis] * size[axis] / 2 for axis in range(3))
            for sign in itertools.product((-1, 1), repeat=3)
        ]
    else:
        raise ValueError(f"Unsupported collision primitive {collider['type']!r}; no approximation")
    if len(set(points)) < 4:
        raise ValueError("A collision hull requires at least four distinct vertices")
    if surface:
        # Inverse-transpose scaling preserves the authored surface normal under
        # nonuniform scale. Extrude only behind that surface, never above it.
        normal = _rotate((0.0, 0.0, 1.0), rotation)
        normal = tuple(normal[axis] / component_scale[axis] for axis in range(3))
        normal = (normal[0], -normal[1], normal[2])
        length = math.sqrt(sum(value * value for value in normal))
        normal = tuple(value / length for value in normal)
    result = []
    for point in points:
        rotated = _rotate(tuple(point[axis] * scale[axis] for axis in range(3)), rotation)
        result.append(
            _position(
                tuple(
                    (rotated[axis] + translation[axis]) * component_scale[axis] for axis in range(3)
                )
            )
        )
    if surface:
        assert surface_thickness is not None
        for index in range(0, len(result), 2):
            result[index] = tuple(
                result[index][axis] - surface_thickness * normal[axis] for axis in range(3)
            )
    return result


def export_level(
    audit: dict[str, Any],
    output: Path,
    *,
    spawn: tuple[float, ...],
    yaw_degrees: float = 0,
    freeze_movable: bool = False,
    friction: tuple[float, ...] = (0.8, 0.005, 0.0001),
    surface_thickness: float | None = None,
    source_sha256: str,
) -> dict[str, Any]:
    """Validate every component before writing the scene and collision manifest."""
    if audit.get("errors"):
        raise ValueError(f"Source audit failed: {audit['errors']}")
    spawn = _numbers(spawn, 3)
    friction = _numbers(friction, 3)
    if min(friction) < 0 or not math.isfinite(yaw_degrees):
        raise ValueError("Friction must be nonnegative and yaw must be finite")
    root = ET.Element("mujoco", model="unreal_static_level")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.005", integrator="implicitfast", gravity="0 0 -9.81")
    assets = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    mesh_names: dict[tuple[Any, ...], str] = {}
    source_provenance = {
        key: audit[key]
        for key in (
            "provenance",
            "project",
            "collision_repairs",
            "cinematics",
            "original_asset_count",
            "original_hashes_verified",
        )
        if key in audit
    }
    manifest: dict[str, Any] = {
        "map": audit["map"],
        "source_audit_sha256": source_sha256,
        "source_provenance": source_provenance,
        "components": [],
        "excluded": [],
        "frozen_movable_components": [],
        "collision_surface_exceptions": [],
    }
    hidden: list[str] = []
    for actor in audit["actors"]:
        if actor.get("errors"):
            raise ValueError(f"Incomplete actor audit {actor['path']}: {actor['errors']}")
        for component in actor["meshes"]:
            path = component["path"]
            if component.get("errors"):
                raise ValueError(f"Incomplete component audit {path}: {component['errors']}")
            enabled = component["collision_enabled"]
            if actor.get("is_editor_only_actor") or any(
                flag in enabled for flag in ("NO_COLLISION", "QUERY_ONLY")
            ):
                manifest["excluded"].append({"component": path, "collision_enabled": enabled})
                continue
            if not any(flag in enabled for flag in ("QUERY_AND_PHYSICS", "PHYSICS_ONLY")):
                raise ValueError(f"Unsupported collision mode {enabled!r}: {path}")
            if component["collision_profile"] not in ("BlockAll", "BlockAllDynamic"):
                raise ValueError(f"Collision response mapping is required for {path}")
            if component["simulating_physics"]:
                raise ValueError(
                    f"Dynamic component cannot be frozen by this static exporter: {path}"
                )
            if "MOVABLE" in component["mobility"]:
                if not freeze_movable:
                    raise ValueError(
                        f"Use --freeze-movable to freeze the authored component {path}"
                    )
                manifest["frozen_movable_components"].append(path)
            elif "STATIC" not in component["mobility"]:
                raise ValueError(f"Unsupported component mobility: {path}")
            asset_path = component["asset"]
            description = audit["assets"].get(asset_path)
            if (
                not description
                or description.get("error")
                or description.get("unsupported_collision_count")
            ):
                raise ValueError(f"Missing or unsupported collision asset: {asset_path}")
            if description["collision_trace"] not in (
                "CTF_UseSimpleAndComplex",
                "CTF_UseSimpleAsComplex",
            ):
                raise ValueError(
                    f"Complex triangle collision requires an explicit import: {asset_path}"
                )
            colliders = description["colliders"]
            if not colliders:
                raise ValueError(f"Collidable component has no exported collision shapes: {path}")
            if asset_path in _BLOCKING_SINGLE_HULLS and len(colliders) == 1:
                raise ValueError(
                    f"Authored outer hull blocks the store interior or entrance: {asset_path}"
                )
            instances = component.get(
                "instances", [{"index": None, "transform": component["transform"]}]
            )
            if len(instances) != component.get("instance_count", 1):
                raise ValueError(f"Incomplete instance audit: {path}")
            for instance in instances:
                if instance.get("errors"):
                    raise ValueError(f"Incomplete instance transform: {path}")
                translation, rotation, scale = _transform(instance["transform"])
                record = {
                    "actor": actor["path"],
                    "label": actor["label"],
                    "component": path,
                    "instance": instance["index"],
                    "asset": asset_path,
                    "transform": instance["transform"],
                    "geoms": [],
                }
                identity = _identity(f"{path}:{instance['index']}")
                for index, collider in enumerate(colliders):
                    key = (asset_path, index, scale)
                    if key not in mesh_names:
                        mesh_name = "mesh_" + _identity(json.dumps(key))
                        degenerate = collider["type"] == "box" and collider["size"][2] == 0
                        thickness = None
                        if degenerate:
                            if asset_path != "/Engine/BasicShapes/Plane.Plane":
                                raise ValueError(
                                    f"Unreviewed degenerate collision asset: {asset_path}"
                                )
                            thickness = surface_thickness
                            manifest["collision_surface_exceptions"].append(
                                {
                                    "asset": asset_path,
                                    "collider": index,
                                    "thickness_m": thickness,
                                    "reason": "Authored zero-thickness finite plane needs volume for rigid collision",
                                    "direction": "Behind the authored surface; upper surface and finite bounds preserved",
                                }
                            )
                        vertices = collision_vertices(collider, scale, thickness)
                        ET.SubElement(
                            assets,
                            "mesh",
                            name=mesh_name,
                            vertex=_text(itertools.chain.from_iterable(vertices)),
                        )
                        mesh_names[key] = mesh_name
                    name = f"ue_{identity}_{index}"
                    ET.SubElement(
                        world,
                        "geom",
                        name=name,
                        type="mesh",
                        mesh=mesh_names[key],
                        pos=_text(_position(translation)),
                        quat=_text(_quaternion(rotation)),
                        group="0",
                        rgba="0.5 0.5 0.5 1",
                        friction=_text(friction),
                    )
                    hidden.append(name)
                    record["geoms"].append(name)
                manifest["components"].append(record)
    if not hidden:
        raise ValueError("The audit contains no supported physical collision geometry")
    custom = ET.SubElement(root, "custom")
    ET.SubElement(
        custom,
        "text",
        name="dimos_unreal_assets",
        data=json.dumps({"version": 1, "geoms": {}, "hidden_geoms": hidden}, separators=(",", ":")),
    )
    manifest["summary"] = {
        "collision_instances": len(manifest["components"]),
        "collision_geoms": len(hidden),
        "reused_meshes": len(mesh_names),
        "excluded_components": len(manifest["excluded"]),
        "frozen_movable_components": len(manifest["frozen_movable_components"]),
        "physics_bodies": 1,
    }
    yaw = math.radians(yaw_degrees) / 2
    sidecar = {
        "format": "dimos.scene.v1",
        "id": "konbini-full-store",
        "spawns": {
            "default": {"position": spawn, "orientation": [0, 0, math.sin(yaw), math.cos(yaw)]}
        },
        "provenance": {
            "authoring_map": audit["map"],
            "exporter": Path(__file__).name,
            "source_audit_sha256": source_sha256,
            "collision_manifest": "collision-manifest.json",
            "visuals": "Original authored Unreal level; collision geoms are hidden",
            "friction": {
                "source": "Explicit simulation setting; not Unreal physical materials",
                "value": friction,
            },
            "frozen_movable_components": manifest["frozen_movable_components"],
            "collision_surface_exceptions": manifest["collision_surface_exceptions"],
            "source": source_provenance,
        },
    }
    output.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(output / "scene.xml", encoding="utf-8", xml_declaration=True)
    (output / "scene.json").write_text(json.dumps(sidecar, indent=2) + "\n")
    (output / "collision-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audit", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--spawn",
        type=float,
        nargs=3,
        required=True,
        metavar=("X", "Y", "Z"),
        help="Robot support pose in right-handed world metres",
    )
    parser.add_argument("--yaw-deg", type=float, default=0)
    parser.add_argument("--freeze-movable", action="store_true")
    parser.add_argument("--friction", type=float, nargs=3, default=(0.8, 0.005, 0.0001))
    parser.add_argument(
        "--surface-thickness",
        type=float,
        help="Explicit finite Plane collision thickness in metres, extending only below the authored surface",
    )
    args = parser.parse_args()
    source = args.audit.read_bytes()
    manifest = export_level(
        json.loads(source),
        args.output,
        spawn=tuple(args.spawn),
        yaw_degrees=args.yaw_deg,
        freeze_movable=args.freeze_movable,
        friction=tuple(args.friction),
        surface_thickness=args.surface_thickness,
        source_sha256=hashlib.sha256(source).hexdigest(),
    )
    print(json.dumps(manifest["summary"], indent=2))


if __name__ == "__main__":
    main()
