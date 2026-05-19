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

"""Scene package contracts for offline asset cooking.

Runtime modules consume the artifacts described here; they do not perform
the heavy bake themselves.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any

from dimos.simulation.scene_assets.mesh_scene import SceneMeshAlignment

ARTIFACT_FRAMES = {
    "browser_visual": "source",
    "browser_collision": "source",
    "mujoco": "dimos_world",
}


@dataclass(frozen=True)
class BrowserVisualSpec:
    """Browser-rendered asset policy."""

    enabled: bool = True
    output_name: str = "visual.glb"
    optimizer: str = "gltfpack"
    simplify_ratio: float = 0.3
    simplify_error: float = 0.02
    texture_format: str | None = None
    max_texture_size: int | None = None
    max_meshes: int = 200
    max_materials: int = 50
    max_textures: int = 750
    max_vertices: int = 750_000
    max_vertex_growth_ratio: float = 1.25


@dataclass(frozen=True)
class BrowserCollisionSpec:
    """Browser raycast/physics collision asset policy."""

    enabled: bool = True
    output_name: str = "collision.glb"
    target_faces: int = 100_000


@dataclass(frozen=True)
class MujocoSceneSpec:
    """MuJoCo collision asset policy."""

    enabled: bool = True
    include_visual_mesh: bool = False


@dataclass(frozen=True)
class SceneCookSpec:
    """Complete cook input for one source scene."""

    source_path: Path
    alignment: SceneMeshAlignment = field(default_factory=SceneMeshAlignment)
    browser_visual: BrowserVisualSpec = field(default_factory=BrowserVisualSpec)
    browser_collision: BrowserCollisionSpec = field(default_factory=BrowserCollisionSpec)
    mujoco: MujocoSceneSpec = field(default_factory=MujocoSceneSpec)


@dataclass(frozen=True)
class ScenePackage:
    """Cooked scene outputs for runtime modules."""

    package_dir: Path
    source_path: Path
    alignment: SceneMeshAlignment
    visual_path: Path | None = None
    browser_collision_path: Path | None = None
    mujoco_model_path: Path | None = None
    mujoco_wrapper_path: Path | None = None
    metadata_path: Path | None = None
    stats: dict[str, Any] = field(default_factory=dict)

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "source_path": str(self.source_path),
            "package_dir": str(self.package_dir),
            "alignment": asdict(self.alignment),
            "artifact_frames": ARTIFACT_FRAMES,
            "artifacts": {
                "browser_visual": str(self.visual_path) if self.visual_path else None,
                "browser_collision": (
                    str(self.browser_collision_path) if self.browser_collision_path else None
                ),
                "mujoco_model": str(self.mujoco_model_path) if self.mujoco_model_path else None,
                "mujoco_wrapper": (
                    str(self.mujoco_wrapper_path) if self.mujoco_wrapper_path else None
                ),
            },
            "stats": self.stats,
        }

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
    return ScenePackage(
        package_dir=Path(raw["package_dir"]),
        source_path=Path(raw["source_path"]),
        alignment=align,
        visual_path=Path(artifacts["browser_visual"]) if artifacts.get("browser_visual") else None,
        browser_collision_path=(
            Path(artifacts["browser_collision"]) if artifacts.get("browser_collision") else None
        ),
        mujoco_model_path=Path(artifacts["mujoco_model"])
        if artifacts.get("mujoco_model")
        else None,
        mujoco_wrapper_path=(
            Path(artifacts["mujoco_wrapper"]) if artifacts.get("mujoco_wrapper") else None
        ),
        metadata_path=metadata_path,
        stats=raw.get("stats", {}),
    )


def _validate_artifact_frames(raw: dict[str, Any], metadata_path: Path) -> None:
    frames = raw.get("artifact_frames")
    if frames is None:
        raise ValueError(
            f"scene package is missing artifact frame metadata: {metadata_path}. "
            "Recook it with dimos.simulation.scene_assets.cook."
        )

    artifacts = raw.get("artifacts", {})
    required = {
        "browser_visual": "browser_visual",
        "browser_collision": "browser_collision",
        "mujoco_model": "mujoco",
    }
    for artifact_name, frame_name in required.items():
        if artifacts.get(artifact_name) and frames.get(frame_name) != ARTIFACT_FRAMES[frame_name]:
            raise ValueError(
                f"scene package artifact frame mismatch in {metadata_path}: "
                f"{frame_name}={frames.get(frame_name)!r}, "
                f"expected {ARTIFACT_FRAMES[frame_name]!r}. Recook the scene package."
            )


__all__ = [
    "ARTIFACT_FRAMES",
    "BrowserCollisionSpec",
    "BrowserVisualSpec",
    "MujocoSceneSpec",
    "SceneCookSpec",
    "ScenePackage",
    "load_scene_package",
]
