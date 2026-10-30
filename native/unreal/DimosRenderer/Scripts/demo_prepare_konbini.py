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

"""Prepare and audit the known local Konbini copy for a static robotics scene.

Run with UnrealEditor -ExecutePythonScript and -RenderOffscreen in the isolated
~/.cache/dimos/unreal-konbini/DimosRenderer project. Only the copied building and
front-window mesh collision sets are replaced, using Unreal convex decomposition.
The copied map's cinematic autoplay is disabled; all other authored content stays
intact. No source project asset is saved. Original content hashes are recorded and
checked before and after. If /tmp/dimos-store-original-hashes.json exists, they
must also match that manifest. DIMOS_KONBINI_SOURCE_HASHES may select a required
manifest explicitly.

DIMOS_KONBINI_AUDIT selects the JSON output (default /tmp/dimos-konbini-map-audit.json).
This is a preparation tool for this specific store, not a general map importer.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable
from functools import partial
import hashlib
import json
import os
from pathlib import Path
import traceback
from typing import Any

import unreal  # type: ignore[import-not-found]  # Provided by Unreal Editor.

MAP_PATH = "/Game/Konbini/Maps/Konbini"
PROJECT = Path.home() / ".cache/dimos/unreal-konbini/DimosRenderer"
SOURCE = Path.home() / "Downloads/Japanese_Convenience_Store_Konbini_UE5/Content/Konbini"
SOURCE_HASHES = Path("/tmp/dimos-store-original-hashes.json")
REPAIR_ASSETS = (
    "/Game/Konbini/Assets/Geo/SM_KombiniGeo",
    "/Game/Konbini/Assets/Geo/SM_FrontWindows_01",
)
DECOMPOSITION = {"hull_count": 64, "max_hull_verts": 32, "hull_precision": 1_000_000}


def _vector(value: Any) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def _transform(value: Any) -> dict[str, list[float]]:
    if isinstance(value, tuple):
        value = next(item for item in value if hasattr(item, "translation"))
    rotation = value.rotation
    return {
        "translation": _vector(value.translation),
        "rotation_xyzw": [
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        ],
        "scale": _vector(value.scale3d),
    }


def _instance_transform(component: Any, index: int) -> dict[str, list[float]]:
    return _transform(component.get_instance_transform(index, world_space=True))


def _serial(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if hasattr(value, "get_path_name"):
        return value.get_path_name()
    if isinstance(value, (list, tuple)) or type(value).__name__ == "Array":
        return [_serial(item) for item in value]
    return str(value)


def _attempt(record: dict[str, Any], key: str, call: Callable[[], Any]) -> None:
    try:
        record[key] = call()
    except Exception as error:
        record.setdefault("errors", []).append({"field": key, "error": str(error)})


def _prop(record: dict[str, Any], obj: Any, name: str) -> None:
    _attempt(record, name, lambda: _serial(obj.get_editor_property(name)))


def _hashes(root: Path) -> dict[str, str]:
    result = {}
    for path in sorted(root.rglob("*")):
        if path.is_file():
            with path.open("rb") as stream:
                result[path.relative_to(root).as_posix()] = hashlib.file_digest(
                    stream, "sha256"
                ).hexdigest()
    return result


def _guard_copied_file(asset_path: str, suffix: str) -> Path:
    path = PROJECT / "Content" / (asset_path.removeprefix("/Game/") + suffix)
    if not path.resolve().is_relative_to(PROJECT.resolve()) or not path.is_file():
        raise RuntimeError(f"Expected an existing asset inside the isolated project: {path}")
    original = SOURCE / (asset_path.removeprefix("/Game/Konbini/") + suffix)
    if original.exists() and path.samefile(original):
        raise RuntimeError(f"Refusing to edit a symlink or hard link to the original asset: {path}")
    return path


def _describe(mesh: Any) -> dict[str, Any]:
    result = json.loads(unreal.DimosAssetLibrary.describe_static_mesh(mesh))
    if not isinstance(result, dict) or result.get("error"):
        raise RuntimeError(f"Cannot inspect mesh: {result}")
    return result


def _repair_collision(audit: dict[str, Any]) -> None:
    meshes = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    for asset_path in REPAIR_ASSETS:
        _guard_copied_file(asset_path, ".uasset")
        mesh = unreal.load_asset(asset_path)
        if mesh is None:
            raise RuntimeError(f"Cannot load copied mesh {asset_path}")
        before = _describe(mesh)
        repair = {
            "asset": asset_path,
            "parameters": DECOMPOSITION,
            "before_colliders": before["colliders"],
            "before_hull_count": len(before["colliders"]),
        }
        audit["collision_repairs"].append(repair)
        if not meshes.set_convex_decomposition_collisions(mesh, **DECOMPOSITION):
            raise RuntimeError(f"Convex decomposition failed for {asset_path}")
        after = _describe(mesh)
        repair["after_colliders"] = after["colliders"]
        repair["after_hull_count"] = len(after["colliders"])
        if repair["after_hull_count"] <= 1:
            raise RuntimeError(f"Decomposition did not separate the enclosing hull: {asset_path}")
        if not unreal.EditorAssetLibrary.save_loaded_asset(mesh):
            raise RuntimeError(f"Cannot save copied collision asset {asset_path}")
        repair["saved"] = True


def _disable_cinematics(actors: list[Any], levels: Any, audit: dict[str, Any]) -> None:
    changed = False
    for actor in actors:
        if not isinstance(actor, unreal.LevelSequenceActor):
            continue
        settings = actor.get_editor_property("playback_settings")
        autoplay = bool(settings.get_editor_property("auto_play"))
        audit["cinematics"].append({"actor": actor.get_path_name(), "auto_play_before": autoplay})
        if autoplay:
            actor.modify()
            settings.set_editor_property("auto_play", False)
            actor.set_editor_property("playback_settings", settings)
            changed = True
        audit["cinematics"][-1]["auto_play_after"] = bool(
            actor.get_editor_property("playback_settings").get_editor_property("auto_play")
        )
    if changed and not levels.save_current_level():
        raise RuntimeError("Cannot save the copied map after disabling cinematic autoplay")
    audit["copied_map_saved"] = changed


def _mesh_component(component: Any, assets: dict[str, Any]) -> dict[str, Any]:
    result = {"path": component.get_path_name(), "class": component.get_class().get_name()}
    _attempt(result, "transform", lambda: _transform(component.get_world_transform()))
    _attempt(result, "collision_enabled", lambda: str(component.get_collision_enabled()))
    _attempt(result, "collision_profile", lambda: str(component.get_collision_profile_name()))
    _attempt(result, "simulating_physics", lambda: bool(component.is_simulating_physics()))
    for name in (
        "mobility",
        "visible",
        "hidden_in_game",
        "cast_shadow",
        "override_materials",
        "component_tags",
    ):
        _prop(result, component, name)
    mesh = component.get_editor_property("static_mesh")
    result["asset"] = mesh.get_path_name() if mesh is not None else None
    if mesh is not None and result["asset"] not in assets:
        try:
            assets[result["asset"]] = _describe(mesh)
        except Exception as error:
            assets[result["asset"]] = {"error": str(error)}
    _attempt(
        result,
        "effective_materials",
        lambda: [
            _serial(component.get_material(index)) for index in range(component.get_num_materials())
        ],
    )
    if isinstance(component, unreal.InstancedStaticMeshComponent):
        count = component.get_instance_count()
        result["instance_count"] = count
        result["instances"] = []
        for index in range(count):
            instance = {"index": index}
            _attempt(
                instance,
                "transform",
                partial(_instance_transform, component, index),
            )
            result["instances"].append(instance)
    return result


def _light_component(component: Any) -> dict[str, Any]:
    result = {"path": component.get_path_name(), "class": component.get_class().get_name()}
    _attempt(result, "transform", lambda: _transform(component.get_world_transform()))
    for name in (
        "mobility",
        "visible",
        "hidden_in_game",
        "intensity",
        "light_color",
        "cast_shadows",
    ):
        _prop(result, component, name)
    return result


def _audit_actor(actor: Any, assets: dict[str, Any]) -> dict[str, Any]:
    result = {
        "path": actor.get_path_name(),
        "label": actor.get_actor_label(),
        "class": actor.get_class().get_name(),
        "meshes": [],
        "lights": [],
    }
    _attempt(result, "transform", lambda: _transform(actor.get_actor_transform()))
    _attempt(result, "bounds", lambda: [_vector(value) for value in actor.get_actor_bounds(False)])
    for name in ("hidden", "tags", "is_editor_only_actor"):
        _prop(result, actor, name)
    for component in actor.get_components_by_class(unreal.StaticMeshComponent):
        try:
            result["meshes"].append(_mesh_component(component, assets))
        except Exception as error:
            result.setdefault("errors", []).append(
                {
                    "component": component.get_path_name(),
                    "error": str(error),
                    "traceback": traceback.format_exc(),
                }
            )
    for component in actor.get_components_by_class(unreal.LightComponentBase):
        result["lights"].append(_light_component(component))
    result["spawn_or_camera"] = any(
        word in result["class"].lower()
        for word in ("playerstart", "camera", "targetpoint", "spawn")
    )
    return result


def _prepare(audit: dict[str, Any]) -> None:
    actual = Path(unreal.Paths.project_dir()).resolve()
    if actual != PROJECT.resolve() or actual.is_relative_to(SOURCE.parents[1].resolve()):
        raise RuntimeError(
            f"Refusing to edit {actual}; this tool only edits copied project {PROJECT}"
        )
    _guard_copied_file(MAP_PATH, ".umap")
    for path in REPAIR_ASSETS:
        _guard_copied_file(path, ".uasset")
    before = _hashes(SOURCE)
    if not before:
        raise RuntimeError(f"Original Konbini content is missing: {SOURCE}")
    audit["original_hashes"] = before
    explicit_manifest = os.environ.get("DIMOS_KONBINI_SOURCE_HASHES")
    manifest = (
        Path(explicit_manifest).expanduser() if explicit_manifest is not None else SOURCE_HASHES
    )
    if explicit_manifest is not None or manifest.exists():
        expected = json.loads(manifest.read_text())
        if before != expected:
            raise RuntimeError("Original Konbini content does not match the recorded source hashes")
        audit["original_hash_manifest"] = str(manifest)
    audit["original_asset_count"] = len(before)
    audit["project"] = str(actual)
    try:
        _repair_collision(audit)
        levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
        actors_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
        if not levels.load_level(MAP_PATH):
            raise RuntimeError(f"Could not load copied map {MAP_PATH}")
        actors = list(actors_subsystem.get_all_level_actors())
        _disable_cinematics(actors, levels, audit)
        audit["actors"] = [_audit_actor(actor, audit["assets"]) for actor in actors]
        audit["summary"] = {
            "actors": len(actors),
            "actor_classes": dict(Counter(actor["class"] for actor in audit["actors"])),
            "mesh_components": sum(len(actor["meshes"]) for actor in audit["actors"]),
            "unique_mesh_assets": len(audit["assets"]),
            "mesh_instances": sum(
                component.get("instance_count", 1)
                for actor in audit["actors"]
                for component in actor["meshes"]
            ),
            "light_components": sum(len(actor["lights"]) for actor in audit["actors"]),
        }
    finally:
        audit["original_hashes_verified"] = _hashes(SOURCE) == before
        if not audit["original_hashes_verified"]:
            raise RuntimeError("Original Konbini content changed during copied-scene preparation")


def main() -> None:
    output = Path(os.environ.get("DIMOS_KONBINI_AUDIT", "/tmp/dimos-konbini-map-audit.json"))
    audit: dict[str, Any] = {
        "map": MAP_PATH,
        "actors": [],
        "assets": {},
        "errors": [],
        "collision_repairs": [],
        "cinematics": [],
    }
    try:
        _prepare(audit)
    except Exception as error:
        audit["errors"].append({"error": str(error), "traceback": traceback.format_exc()})
        unreal.log_error(f"Konbini preparation failed: {error}")
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(audit, indent=2) + "\n")
        unreal.log(f"DIMOS_KONBINI_MAP_AUDIT {output}: {audit.get('summary', {})}")
        unreal.SystemLibrary.quit_editor()


if __name__ == "__main__":
    main()
