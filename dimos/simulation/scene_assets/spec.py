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

"""Runtime scene package metadata contract.

Runtime modules consume the artifacts described here; cook-time policy lives
under ``dimos.experimental.scene_cooking``.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any

ARTIFACT_FRAMES = {
    "browser_visual": "source",
    "browser_collision": "source",
    "mujoco": "dimos_world",
    "mujoco_binary": "dimos_world",
}


@dataclass(frozen=True)
class SceneMeshAlignment:
    """Transform from a raw scene asset frame into DimOS world frame.

    Cookers use this to produce artifacts; runtime consumers keep it in
    package metadata so viewers and simulators know each artifact's frame.
    """

    scale: float = 1.0
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    y_up: bool = True


@dataclass(frozen=True)
class MujocoComposedBinary:
    """A precompiled robot+scene MuJoCo model.

    These binaries are cache artifacts. They are specific to the robot, spawn
    pose, entity policy, scene revision, and MuJoCo version used by the cooker.
    """

    key: str
    path: Path
    robot: str | None = None
    spawn: dict[str, Any] = field(default_factory=dict)
    entity_policy: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ScenePackage:
    """Cooked scene outputs for runtime modules."""

    package_dir: Path
    source_path: Path
    alignment: SceneMeshAlignment
    visual_path: Path | None = None
    browser_visuals: dict[str, Path] = field(default_factory=dict)
    browser_collision_path: Path | None = None
    objects_path: Path | None = None
    mujoco_scene_path: Path | None = None
    mujoco_binary_path: Path | None = None
    mujoco_composed_binaries: dict[str, MujocoComposedBinary] = field(default_factory=dict)
    metadata_path: Path | None = None
    entities: list[dict[str, Any]] = field(default_factory=list)
    stats: dict[str, Any] = field(default_factory=dict)

    def to_json_dict(self) -> dict[str, Any]:
        package_dir = self.package_dir.expanduser().resolve()

        return {
            "source_path": str(self.source_path),
            "package_dir": ".",
            "alignment": asdict(self.alignment),
            "artifact_frames": ARTIFACT_FRAMES,
            "artifacts": {
                "browser_visual": _serialize_package_path(self.visual_path, package_dir),
                "browser_visuals": _serialize_browser_visuals(
                    self.browser_visuals,
                    package_dir,
                ),
                "browser_collision": _serialize_package_path(
                    self.browser_collision_path,
                    package_dir,
                ),
                "objects": _serialize_package_path(self.objects_path, package_dir),
                "mujoco_scene": _serialize_package_path(self.mujoco_scene_path, package_dir),
                "mujoco_binary": _serialize_package_path(self.mujoco_binary_path, package_dir),
                "mujoco_composed_binaries": _serialize_mujoco_composed_binaries(
                    self.mujoco_composed_binaries,
                    package_dir,
                ),
            },
            "entities": _serialize_entity_paths(self.entities, package_dir),
            "stats": self.stats,
        }

    def browser_visual_path(self, target: str) -> Path | None:
        """Return a visual GLB for a specific browser/viewer backend.

        Scene packages can carry multiple browser-facing visuals because viewer
        support for glTF extensions differs. For example, Rerun currently needs
        conservative GLBs, while Babylon can use more web-oriented output.

        Older packages only have ``browser_visual``. Rerun may use that legacy
        artifact; other targets should cook their own named asset.
        """
        target_key = target.strip().lower()
        if target_key in self.browser_visuals:
            return self.browser_visuals[target_key]
        if target_key == "rerun":
            return self.browser_visuals.get("default") or self.visual_path
        return self.browser_visuals.get("default")

    def mujoco_composed_binary(
        self,
        key: str | None = None,
        *,
        robot: str | None = None,
        entity_policy: str | None = None,
    ) -> MujocoComposedBinary | None:
        """Return a declared robot+scene MuJoCo binary matching the request."""
        if key is not None:
            candidate = self.mujoco_composed_binaries.get(key)
            if candidate is None:
                return None
            return (
                candidate
                if _mujoco_composed_binary_matches(
                    candidate,
                    robot=robot,
                    entity_policy=entity_policy,
                )
                else None
            )

        matches = [
            binary
            for binary in self.mujoco_composed_binaries.values()
            if _mujoco_composed_binary_matches(
                binary,
                robot=robot,
                entity_policy=entity_policy,
            )
        ]
        if len(matches) > 1:
            keys = ", ".join(sorted(binary.key for binary in matches))
            raise ValueError(f"multiple composed MuJoCo binaries match; choose one key: {keys}")
        return matches[0] if matches else None

    def mujoco_composed_binary_path(
        self,
        key: str | None = None,
        *,
        robot: str | None = None,
        entity_policy: str | None = None,
    ) -> Path | None:
        """Return a robot+scene MuJoCo binary path matching the request."""
        binary = self.mujoco_composed_binary(
            key,
            robot=robot,
            entity_policy=entity_policy,
        )
        return binary.path if binary is not None else None

    def write_metadata(self, path: Path | None = None) -> Path:
        metadata_path = path or self.metadata_path or (self.package_dir / "scene.meta.json")
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(self.to_json_dict(), indent=2, sort_keys=True) + "\n")
        return metadata_path


def load_scene_package(path: str | Path) -> ScenePackage:
    """Load a previously written ``scene.meta.json``."""
    metadata_path = Path(path).expanduser().resolve()
    raw = json.loads(metadata_path.read_text())
    _validate_artifact_frames(raw, metadata_path)
    artifacts = raw.get("artifacts", {})
    align = SceneMeshAlignment(**raw["alignment"])
    package_dir = _resolve_package_dir(raw.get("package_dir"), metadata_path)
    browser_visuals = _resolve_browser_visuals(artifacts.get("browser_visuals"), package_dir)
    legacy_visual_path = _resolve_package_path(artifacts.get("browser_visual"), package_dir)
    return ScenePackage(
        package_dir=package_dir,
        source_path=Path(raw["source_path"]),
        alignment=align,
        visual_path=legacy_visual_path,
        browser_visuals=browser_visuals,
        browser_collision_path=(
            _resolve_package_path(artifacts.get("browser_collision"), package_dir)
        ),
        objects_path=_resolve_package_path(artifacts.get("objects"), package_dir),
        mujoco_scene_path=_resolve_package_path(artifacts.get("mujoco_scene"), package_dir),
        mujoco_binary_path=_resolve_package_path(artifacts.get("mujoco_binary"), package_dir),
        mujoco_composed_binaries=_resolve_mujoco_composed_binaries(
            artifacts.get("mujoco_composed_binaries"),
            package_dir,
        ),
        metadata_path=metadata_path,
        entities=_resolve_entity_paths(raw.get("entities", []), package_dir),
        stats=raw.get("stats", {}),
    )


def _validate_artifact_frames(raw: dict[str, Any], metadata_path: Path) -> None:
    frames = raw.get("artifact_frames")
    if frames is None:
        raise ValueError(
            f"scene package is missing artifact frame metadata: {metadata_path}. "
            "Recook it with dimos.experimental.scene_cooking.cook."
        )

    artifacts = raw.get("artifacts", {})
    required = {
        "browser_visual": "browser_visual",
        "browser_collision": "browser_collision",
        "mujoco_scene": "mujoco",
        "mujoco_binary": "mujoco_binary",
    }
    for artifact_name, frame_name in required.items():
        if artifacts.get(artifact_name) and frames.get(frame_name) != ARTIFACT_FRAMES[frame_name]:
            raise ValueError(
                f"scene package artifact frame mismatch in {metadata_path}: "
                f"{frame_name}={frames.get(frame_name)!r}, "
                f"expected {ARTIFACT_FRAMES[frame_name]!r}. Recook the scene package."
            )

    if (
        artifacts.get("mujoco_composed_binaries")
        and frames.get("mujoco_binary") != ARTIFACT_FRAMES["mujoco_binary"]
    ):
        raise ValueError(
            f"scene package artifact frame mismatch in {metadata_path}: "
            f"mujoco_binary={frames.get('mujoco_binary')!r}, "
            f"expected {ARTIFACT_FRAMES['mujoco_binary']!r}. Recook the scene package."
        )


def _serialize_package_path(path: Path | None, package_dir: Path) -> str | None:
    if path is None:
        return None
    resolved = path.expanduser().resolve()
    try:
        return resolved.relative_to(package_dir).as_posix()
    except ValueError:
        return str(resolved)


def _resolve_package_dir(raw: str | None, metadata_path: Path) -> Path:
    if raw is None:
        return metadata_path.parent
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path
    return (metadata_path.parent / path).resolve()


def _resolve_package_path(raw: str | None, package_dir: Path) -> Path | None:
    if not raw:
        return None
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path
    return (package_dir / path).resolve()


def _serialize_browser_visuals(
    visuals: dict[str, Path],
    package_dir: Path,
) -> dict[str, str]:
    return {
        str(target).strip().lower(): serialized
        for target, path in sorted(visuals.items())
        if (serialized := _serialize_package_path(path, package_dir)) is not None
    }


def _resolve_browser_visuals(raw: Any, package_dir: Path) -> dict[str, Path]:
    if not isinstance(raw, dict):
        return {}
    resolved: dict[str, Path] = {}
    for target, path in raw.items():
        if not isinstance(target, str) or not isinstance(path, str):
            continue
        visual_path = _resolve_package_path(path, package_dir)
        if visual_path is not None:
            resolved[target.strip().lower()] = visual_path
    return resolved


def _serialize_mujoco_composed_binaries(
    binaries: dict[str, MujocoComposedBinary],
    package_dir: Path,
) -> dict[str, dict[str, Any]]:
    serialized: dict[str, dict[str, Any]] = {}
    for key, binary in sorted(binaries.items()):
        path = _serialize_package_path(binary.path, package_dir)
        if path is None:
            continue
        payload: dict[str, Any] = {"path": path}
        if binary.robot is not None:
            payload["robot"] = binary.robot
        if binary.spawn:
            payload["spawn"] = binary.spawn
        if binary.entity_policy is not None:
            payload["entity_policy"] = binary.entity_policy
        if binary.metadata:
            payload["metadata"] = binary.metadata
        serialized[key] = payload
    return serialized


def _resolve_mujoco_composed_binaries(
    raw: Any,
    package_dir: Path,
) -> dict[str, MujocoComposedBinary]:
    if isinstance(raw, dict):
        items: list[tuple[Any, Any]] = list(raw.items())
    elif isinstance(raw, list):
        items = [(item.get("key"), item) for item in raw if isinstance(item, dict)]
    else:
        return {}

    resolved: dict[str, MujocoComposedBinary] = {}
    for raw_key, raw_value in items:
        if raw_key is None:
            continue
        key = str(raw_key).strip()
        if not key:
            continue

        if isinstance(raw_value, str):
            path = _resolve_package_path(raw_value, package_dir)
            robot = None
            spawn: dict[str, Any] = {}
            entity_policy = None
            metadata: dict[str, Any] = {}
        elif isinstance(raw_value, dict):
            key = str(raw_value.get("key") or key).strip()
            path = _resolve_package_path(raw_value.get("path"), package_dir)
            robot = _optional_str(raw_value.get("robot"))
            spawn = _dict_or_empty(raw_value.get("spawn"))
            entity_policy = _optional_str(raw_value.get("entity_policy"))
            metadata = _dict_or_empty(raw_value.get("metadata"))
        else:
            continue

        if not key or path is None:
            continue
        resolved[key] = MujocoComposedBinary(
            key=key,
            path=path,
            robot=robot,
            spawn=spawn,
            entity_policy=entity_policy,
            metadata=metadata,
        )
    return resolved


def _mujoco_composed_binary_matches(
    binary: MujocoComposedBinary,
    *,
    robot: str | None,
    entity_policy: str | None,
) -> bool:
    return (robot is None or binary.robot == robot) and (
        entity_policy is None or binary.entity_policy == entity_policy
    )


def _optional_str(raw: Any) -> str | None:
    if raw is None:
        return None
    return str(raw)


def _dict_or_empty(raw: Any) -> dict[str, Any]:
    return dict(raw) if isinstance(raw, dict) else {}


_ENTITY_PATH_KEYS = ("visual_path", "collision_path", "mesh_path")
_ENTITY_PATH_LIST_KEYS = ("collision_paths",)


def _serialize_entity_paths(
    entities: list[dict[str, Any]], package_dir: Path
) -> list[dict[str, Any]]:
    out = deepcopy(entities)
    for entity in out:
        _rewrite_entity_paths(entity, package_dir, serialize=True)
    return out


def _resolve_entity_paths(
    entities: list[dict[str, Any]], package_dir: Path
) -> list[dict[str, Any]]:
    out = deepcopy(entities)
    for entity in out:
        _rewrite_entity_paths(entity, package_dir, serialize=False)
    return out


def _rewrite_entity_paths(
    entity: dict[str, Any],
    package_dir: Path,
    *,
    serialize: bool,
) -> None:
    def rewrite(value: Any) -> Any:
        if not isinstance(value, str):
            return value
        path = Path(value).expanduser()
        if serialize:
            return _serialize_package_path(path, package_dir)
        if path.is_absolute():
            return str(path)
        return str((package_dir / path).resolve())

    for key in _ENTITY_PATH_KEYS:
        if key in entity:
            entity[key] = rewrite(entity[key])

    for key in _ENTITY_PATH_LIST_KEYS:
        value = entity.get(key)
        if isinstance(value, list):
            entity[key] = [rewrite(item) for item in value]

    artifacts = entity.get("artifacts")
    if isinstance(artifacts, dict):
        for key, value in list(artifacts.items()):
            artifacts[key] = rewrite(value)
