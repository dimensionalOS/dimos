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

"""RoboCasa VQA source boundary.

The normal dimOS process calls :func:`export_robocasa_snapshots`, which runs
this same file with a separately prepared RoboCasa Python interpreter. Keep
module-level imports limited to packages available in both environments.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from tempfile import TemporaryDirectory
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from dimos.evals.generate import VqaGenerationSpec
    from dimos.evals.vqa import SceneSnapshot

_RUN_EXPORTER = (
    "import runpy, sys; script = sys.argv.pop(1); runpy.run_path(script, run_name='__main__')"
)
ROBOCASA_EXPORT_SCHEMA_VERSION = 1


def export_robocasa_snapshots(
    spec: VqaGenerationSpec, *, source_python: str
) -> list[SceneSnapshot]:
    """Run RoboCasa out of process and load its private scene snapshots."""
    interpreter = _resolve_interpreter(source_python)
    timeout_s = max(300.0, spec.episodes * 120.0)
    with TemporaryDirectory(prefix="dimos-robocasa-") as temp:
        temp_dir = Path(temp)
        config_path = temp_dir / "export.json"
        export_dir = temp_dir / "snapshots"
        config_path.write_text(
            json.dumps(
                {
                    "task": spec.task,
                    "episodes": spec.episodes,
                    "seed": spec.seed,
                    "camera": spec.camera,
                    "image_size": list(spec.image_size),
                    "split": spec.split,
                    "robot": spec.robot,
                },
                indent=2,
            )
            + "\n"
        )
        completed = subprocess.run(
            [
                interpreter,
                "-c",
                _RUN_EXPORTER,
                str(Path(__file__).resolve()),
                "export",
                str(config_path),
                str(export_dir),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        if completed.returncode != 0 or not (export_dir / "snapshots.jsonl").is_file():
            detail = (completed.stderr or completed.stdout).strip()
            raise RuntimeError(
                f"RoboCasa exporter failed ({completed.returncode}): {detail[-2000:]}"
            )
        snapshots = _load_export(export_dir)
    if len(snapshots) != spec.episodes:
        raise RuntimeError(
            f"RoboCasa exported {len(snapshots)} snapshots, expected {spec.episodes}"
        )
    return snapshots


def _resolve_interpreter(value: str) -> str:
    candidate = shutil.which(value)
    if candidate is None:
        path = Path(value).expanduser()
        if path.is_file() and os.access(path, os.X_OK):
            candidate = str(path.resolve())
    if candidate is None:
        raise FileNotFoundError(f"RoboCasa Python interpreter not found or executable: {value}")
    return candidate


def _load_export(path: Path) -> list[SceneSnapshot]:
    from dimos.evals.vqa import SceneSnapshot

    manifest = path / "snapshots.jsonl"
    if not manifest.is_file():
        raise RuntimeError("RoboCasa exporter did not create snapshots.jsonl")
    root = path.resolve()
    snapshots: list[SceneSnapshot] = []
    for line_number, line in enumerate(manifest.read_text().splitlines(), start=1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise TypeError(f"RoboCasa snapshot line {line_number} must be an object")
        if value.get("schema_version") != ROBOCASA_EXPORT_SCHEMA_VERSION:
            raise ValueError(f"RoboCasa snapshot line {line_number} has unsupported schema_version")
        image_path = (root / str(value["image"])).resolve()
        if not image_path.is_relative_to(root) or not image_path.is_file():
            raise ValueError(f"RoboCasa snapshot line {line_number} has an invalid image path")
        image = np.load(image_path, allow_pickle=False)
        snapshots.append(SceneSnapshot.from_mapping(value, image=image.copy()))
    return snapshots


def _export(config_path: Path, output: Path) -> None:
    import mujoco
    import robocasa  # type: ignore[import-not-found]
    from robocasa.utils.env_utils import create_env  # type: ignore[import-not-found]

    config = json.loads(config_path.read_text())
    if output.exists():
        raise FileExistsError(f"RoboCasa export output already exists: {output}")
    output.mkdir(parents=True)
    width, height = (int(value) for value in config["image_size"])
    rows: list[dict[str, Any]] = []
    try:
        source_version = importlib.metadata.version("robocasa")
    except importlib.metadata.PackageNotFoundError:
        source_version = str(getattr(robocasa, "__version__", "unknown"))

    for episode in range(int(config["episodes"])):
        seed = int(config["seed"]) + episode
        env = create_env(
            env_name=str(config["task"]),
            robots=str(config["robot"]),
            camera_names=[str(config["camera"])],
            camera_widths=width,
            camera_heights=height,
            seed=seed,
            split=config.get("split"),
        )
        try:
            env.reset()
            rgb = env.sim.render(
                width=width,
                height=height,
                camera_name=str(config["camera"]),
            )[::-1].copy()
            segmentation = env.sim.render(
                width=width,
                height=height,
                camera_name=str(config["camera"]),
                segmentation=True,
            )[::-1].copy()
            if segmentation.ndim != 3 or segmentation.shape[2] != 2:
                raise RuntimeError(f"unexpected RoboCasa segmentation shape: {segmentation.shape}")
            entities = _extract_entities(env, segmentation, mujoco)
            image_name = f"frame-{episode:05d}.npy"
            np.save(output / image_name, rgb, allow_pickle=False)
            rows.append(
                {
                    "schema_version": ROBOCASA_EXPORT_SCHEMA_VERSION,
                    "image": image_name,
                    "entities": entities,
                    "provenance": {
                        "source": "robocasa",
                        "source_version": source_version,
                        "task": str(config["task"]),
                        "seed": seed,
                        "layout_id": _json_safe(env.layout_id),
                        "style_id": _json_safe(env.style_id),
                        "camera": str(config["camera"]),
                    },
                }
            )
        finally:
            close = getattr(env, "close", None)
            if callable(close):
                close()

    with (output / "snapshots.jsonl").open("w") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")


def _extract_entities(
    env: Any, segmentation: np.ndarray[Any, Any], mujoco: Any
) -> list[dict[str, Any]]:
    geom_type = int(mujoco.mjtObj.mjOBJ_GEOM)
    roots = {int(body_id): name for name, body_id in env.obj_body_id.items()}
    geom_owner: dict[int, str] = {}
    for geom_id in range(int(env.sim.model.ngeom)):
        body_id = int(env.sim.model.geom_bodyid[geom_id])
        while body_id != 0:
            owner = roots.get(body_id)
            if owner is not None:
                geom_owner[geom_id] = owner
                break
            body_id = int(env.sim.model.body_parentid[body_id])

    geom_pixels = segmentation[:, :, 0] == geom_type
    entities: list[dict[str, Any]] = []
    for name in sorted(env.objects):
        geom_ids = [geom_id for geom_id, owner in geom_owner.items() if owner == name]
        mask = geom_pixels & np.isin(segmentation[:, :, 1], geom_ids)
        ys, xs = np.nonzero(mask)
        pixel_area = int(xs.size)
        entity: dict[str, Any] = {
            "id": name,
            "category": _normalize_category(env.get_obj_lang(name)),
            "pixel_area": pixel_area,
            "centroid": None,
            "bbox": None,
        }
        if pixel_area:
            entity["centroid"] = [float(xs.mean()), float(ys.mean())]
            entity["bbox"] = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
        entities.append(entity)
    return entities


def _normalize_category(value: str) -> str:
    normalized = re.sub(r"[^a-z0-9]+", "_", value.strip().lower()).strip("_")
    if not normalized:
        raise ValueError("RoboCasa object category must not be empty")
    return normalized


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _main() -> None:
    parser = argparse.ArgumentParser(description="Export RoboCasa snapshots for dimOS VQA")
    subparsers = parser.add_subparsers(dest="command", required=True)
    export = subparsers.add_parser("export")
    export.add_argument("config", type=Path)
    export.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.command == "export":
        _export(args.config, args.output)


if __name__ == "__main__":
    _main()
