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

"""Cook-time scene package configuration."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from dimos.simulation.scene_assets.spec import SceneMeshAlignment


@dataclass(frozen=True)
class BrowserVisualSpec:
    """Browser-rendered asset policy."""

    enabled: bool = True
    target: str = "rerun"
    output_name: str | None = None
    optimizer: str = "gltfpack"
    simplify_ratio: float = 0.3
    simplify_error: float = 0.02
    texture_format: str | None = None
    max_texture_size: int | None = None
    normalize_textures: bool = True
    quantize: bool = False
    use_gpu_instancing: bool = False
    demote_required_extensions: tuple[str, ...] = ("KHR_texture_transform",)
    max_meshes: int = 200
    max_materials: int = 50
    max_textures: int = 750
    max_vertices: int = 750_000
    max_vertex_growth_ratio: float = 1.25

    @property
    def target_key(self) -> str:
        return self.target.strip().lower()

    @property
    def artifact_name(self) -> str:
        return self.output_name or f"visual.{self.target_key}.glb"


#: Per-target overrides layered on top of ``BrowserVisualSpec``'s defaults.
#: "rerun" has no entry here -- its values *are* the dataclass defaults.
_BROWSER_VISUAL_PROFILES: dict[str, dict[str, Any]] = {
    "babylon": {
        "optimizer": "gltfpack",
        "simplify_ratio": 0.3,
        "simplify_error": 0.02,
        "texture_format": None,
        "max_texture_size": None,
        "normalize_textures": False,
        "quantize": True,
        "use_gpu_instancing": True,
        "demote_required_extensions": (),
        "max_meshes": 2_000,
        "max_materials": 400,
        "max_textures": 2_000,
        "max_vertices": 2_000_000,
        "max_vertex_growth_ratio": 2.0,
    },
    "generic": {
        "optimizer": "gltfpack",
        "simplify_ratio": 0.3,
        "simplify_error": 0.02,
        "texture_format": None,
        "max_texture_size": None,
        "normalize_textures": False,
        "quantize": False,
        "use_gpu_instancing": False,
        "demote_required_extensions": (),
        "max_meshes": 500,
        "max_materials": 100,
        "max_textures": 1_000,
        "max_vertices": 1_000_000,
        "max_vertex_growth_ratio": 1.5,
    },
}
BROWSER_VISUAL_TARGETS = tuple(sorted({"rerun", *_BROWSER_VISUAL_PROFILES}))


def browser_visual_spec_for_target(
    target: str,
    **overrides: Any,
) -> BrowserVisualSpec:
    """Build a visual cook spec for a named browser/viewer target."""
    target_key = target.strip().lower()
    if target_key not in BROWSER_VISUAL_TARGETS:
        known = ", ".join(BROWSER_VISUAL_TARGETS)
        raise ValueError(f"unknown browser visual target {target!r}; expected one of: {known}")
    values = dict(_BROWSER_VISUAL_PROFILES.get(target_key, {}))
    values.update(overrides)
    return BrowserVisualSpec(target=target_key, **values)


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
    compile_binary: bool = False


@dataclass(frozen=True)
class SceneCookSpec:
    """Complete cook input for one source scene."""

    source_path: Path
    alignment: SceneMeshAlignment = field(default_factory=SceneMeshAlignment)
    browser_visual: BrowserVisualSpec = field(default_factory=BrowserVisualSpec)
    browser_collision: BrowserCollisionSpec = field(default_factory=BrowserCollisionSpec)
    mujoco: MujocoSceneSpec = field(default_factory=MujocoSceneSpec)
