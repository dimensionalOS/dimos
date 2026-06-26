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

"""Small reference preflight for scene cooking inputs.

Run:

    python -m dimos.experimental.pimsim.scene.demo_scene_cook_requirements \
      data/dimos_office_mesh/dimos_office_mesh.glb \
      --cook-spec data/dimos_office_mesh/dimos_office_mesh.cook.json \
      --output-dir data/scene_packages/dimos_office \
      --scale 2.0 \
      --no-y-up

This module shows what a cooker needs before starting a potentially long bake:
one source asset, one optional sidecar, alignment, and an output directory. It
does not emit scene package artifacts.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
from typing import Any

import numpy as np

from dimos.experimental.pimsim.scene.inspect import inspect_scene_asset
from dimos.experimental.pimsim.scene.plan import build_scene_cook_plan
from dimos.experimental.pimsim.scene.sidecar import SceneCookSidecar
from dimos.experimental.pimsim.scene.source_asset import prepare_scene_source
from dimos.simulation.scene_assets.mesh_scene import load_scene_prims
from dimos.simulation.scene_assets.spec import SceneMeshAlignment


def inspect_cook_requirements(
    source: str | Path,
    *,
    output_dir: str | Path,
    cook_spec: str | Path | None = None,
    alignment: SceneMeshAlignment | None = None,
    sample_prims: int = 8,
    rebake: bool = False,
) -> dict[str, Any]:
    """Return a JSON-serializable summary of the inputs needed to cook a scene."""
    align = alignment or SceneMeshAlignment()
    source_path = Path(source).expanduser().resolve()
    prepared = prepare_scene_source(source_path, rebake=rebake)
    sidecar = (
        SceneCookSidecar.from_json(cook_spec)
        if cook_spec is not None
        else SceneCookSidecar.auto_discover(source_path)
    )
    stats = inspect_scene_asset(prepared.cook_path)
    prims = list(load_scene_prims(prepared.cook_path, alignment=align))
    plan = build_scene_cook_plan(
        prepared.cook_path,
        sidecar=sidecar,
        alignment=align,
        output_dir=output_dir,
    )

    return {
        "source": str(source_path),
        "prepared_source": prepared.to_json_dict(),
        "output_dir": str(Path(output_dir).expanduser().resolve()),
        "alignment": {
            "scale": align.scale,
            "translation": list(align.translation),
            "rotation_zyx_deg": list(align.rotation_zyx_deg),
            "y_up": align.y_up,
        },
        "source_stats": stats.to_json_dict(),
        "source_prim_count": len(prims),
        "sample_prims": [_prim_summary(prim) for prim in prims[:sample_prims]],
        "sidecar": {
            "path": str(sidecar.path) if sidecar.path else None,
            "collision_default": sidecar.collision.default,
            "collision_override_count": len(sidecar.collision.prim_overrides),
            "interactable_count": len(sidecar.interactables),
            "entity_group_count": len(sidecar.entity_groups),
        },
        "cook_plan": {
            "entities": len(plan.entities),
            "entity_prototypes": len(plan.prototypes),
            "stats": plan.stats,
        },
        "cook_command": _cook_command(
            source_path,
            output_dir=output_dir,
            cook_spec=cook_spec,
            alignment=align,
            rebake=rebake,
        ),
    }


def _prim_summary(prim: Any) -> dict[str, Any]:
    vertices = np.asarray(prim.vertices)
    lo = np.min(vertices, axis=0)
    hi = np.max(vertices, axis=0)
    return {
        "name": prim.visual_node_name or prim.prim_path or prim.name,
        "min": lo.round(4).tolist(),
        "max": hi.round(4).tolist(),
        "extent": (hi - lo).round(4).tolist(),
    }


def _cook_command(
    source: Path,
    *,
    output_dir: str | Path,
    cook_spec: str | Path | None,
    alignment: SceneMeshAlignment,
    rebake: bool,
) -> str:
    command = [
        "python",
        "-m",
        "dimos.experimental.pimsim.scene.cook",
        str(source),
        "--output-dir",
        str(output_dir),
        "--scale",
        str(alignment.scale),
    ]
    if cook_spec is not None:
        command.extend(["--cook-spec", str(cook_spec)])
    if alignment.translation != (0.0, 0.0, 0.0):
        command.extend(["--translation", *[str(value) for value in alignment.translation]])
    if alignment.rotation_zyx_deg != (0.0, 0.0, 0.0):
        command.extend(
            ["--rotation-zyx-deg", *[str(value) for value in alignment.rotation_zyx_deg]]
        )
    if not alignment.y_up:
        command.append("--no-y-up")
    if rebake:
        command.append("--rebake")
    return " ".join(shlex.quote(part) for part in command)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("--cook-spec")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--translation", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--rotation-zyx-deg", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--no-y-up", action="store_true")
    parser.add_argument("--sample-prims", type=int, default=8)
    parser.add_argument("--rebake", action="store_true")
    args = parser.parse_args()

    result = inspect_cook_requirements(
        args.source,
        output_dir=args.output_dir,
        cook_spec=args.cook_spec,
        alignment=SceneMeshAlignment(
            scale=args.scale,
            translation=tuple(args.translation),
            rotation_zyx_deg=tuple(args.rotation_zyx_deg),
            y_up=not args.no_y_up,
        ),
        sample_prims=args.sample_prims,
        rebake=args.rebake,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
