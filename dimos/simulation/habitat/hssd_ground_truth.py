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

"""Ground-truth object and wall boxes for HSSD Habitat scenes, computed offline.

HSSD composes a scene from a stage mesh plus placed object assets
(``scenes/<id>.scene_instance.json``). This module reads those files with
trimesh, measures a tight world-axis-aligned box per placed object and per
stage wall, and exports them through
:mod:`dimos.simulation.object_detections` in the DimOS Z-up ``world`` frame, the
frame the Habitat connection publishes odometry in. No simulator is needed.

Regenerate the checked-in files with::

    uv run python -m dimos.simulation.habitat.hssd_ground_truth

Labels come from ``semantics/objects.csv`` (category and product name). The
``semantic_id`` in object configs is not used: it does not match the assets.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import csv
from dataclasses import dataclass
from functools import cached_property
import json
import os
from pathlib import Path
import re
from typing import Any, Literal, cast

import numpy as np
from numpy.typing import NDArray
import trimesh

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.simulation.object_detections import (
    GroundTruthBox,
    Point3,
    boxes_to_detection3d_array,
    top_down,
    write_detection2d_json,
    write_detection3d_json,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Frame the Habitat connection publishes odometry and scans in (blueprints.WORLD_FRAME).
HABITAT_WORLD_FRAME = "world"
# The scene is static; a wall-clock stamp would change every regenerated file.
GROUND_TRUTH_TS = 0.0
# Same override the Habitat QA suites use for the dataset location.
HSSD_DATASET_CONFIG_ENV = "HSSD_DATASET_CONFIG"
DEFAULT_HSSD_DATASET_CONFIG = (
    DIMOS_PROJECT_ROOT / "target/habitat/data/hssd-hab/hssd-hab.scene_dataset_config.json"
)
DEFAULT_OUTPUT_DIR = DIMOS_PROJECT_ROOT / "misc/habitat/ground_truth/hssd"
# The scenes covered by the Habitat QA suites, in "HSSD scene 1..10" order.
HSSD_SCENE_IDS: tuple[str, ...] = (
    "102344193",
    "102344403",
    "103997424_171030444",
    "103997970_171031287",
    "104348463_171513588",
    "106366410_174226806",
    "106878858_174886965",
    "107734110_175999914",
    "108736851_177263586",
    "108736884_177263634",
)
# Stage meshes holding wall top faces: geometry_wallTop.124, whether under a wall_<n>
# group or merged into the stage's top-level leftovers. Not geometry_wall.* (bottom strips).
WALL_TOP_PATTERN = re.compile(r"(?:^|[-_. ])walltop(?:[-_. ]|$)", re.IGNORECASE)
# A top face is a flat quad at ceiling height; the same meshes also hold bottom strips.
WALL_FLAT_MAX_M = 0.02
WALL_TOP_MIN_HEIGHT_M = 1.0
# Touching collinear faces merge into one wall as long as the union stays wall-thin,
# which keeps perpendicular walls at L and T junctions apart.
WALL_MERGE_TOLERANCE_M = 0.03
WALL_MAX_THICKNESS_M = 0.4
# Pieces that nearly coincide (stacked bay-window and frame tops) merge even when thick.
WALL_COINCIDENT_AREA_RATIO = 1.25
WALL_MIN_LENGTH_M = 0.2
WALL_LABEL = "wall"
# Corners are rounded so regenerated files do not churn on float noise.
CORNER_DECIMALS = 6

LabelMode = Literal["category", "name", "category+name"]
LABEL_MODES: tuple[LabelMode, ...] = ("category", "name", "category+name")
DEFAULT_LABEL_MODE: LabelMode = "category+name"

Vertices = NDArray[np.float64]


def habitat_to_ros(x: float, y: float, z: float) -> Point3:
    """Habitat Y-up world to ROS Z-up world; the scalar form of ``frames.R_ROS_HAB``."""
    return (-z, -x, y)


@dataclass(frozen=True)
class ObjectSemantics:
    """One row of HSSD's ``semantics/objects.csv``, keyed by model hash."""

    name: str
    main_category: str
    super_category: str


@dataclass(frozen=True)
class SceneBoxes:
    """Boxes of one scene in the Habitat frame: placed objects, then stage walls."""

    objects: list[GroundTruthBox]
    walls: list[GroundTruthBox]

    @property
    def all(self) -> list[GroundTruthBox]:
        return self.objects + self.walls


class HssdDataset:
    """File access for an HSSD scene dataset, rooted at its dataset config."""

    def __init__(self, config_path: str | Path) -> None:
        self.config_path = Path(config_path)
        self.root = self.config_path.parent
        # "hssd-hab.scene_dataset_config.json" -> "hssd-hab"
        self.name = self.config_path.name.split(".", 1)[0]

    @classmethod
    def from_env(cls, override: str | None = None) -> HssdDataset:
        """Resolve the dataset config: explicit path, else the env override, else the default."""
        path = Path(
            override or os.environ.get(HSSD_DATASET_CONFIG_ENV) or DEFAULT_HSSD_DATASET_CONFIG
        )
        if not path.is_file():
            raise FileNotFoundError(
                f"HSSD dataset config not found: {path} "
                f"(set {HSSD_DATASET_CONFIG_ENV} or pass --dataset)"
            )
        return cls(path)

    def scene_instance(self, scene_id: str) -> dict[str, Any]:
        path = self.root / "scenes" / f"{scene_id}.scene_instance.json"
        instance: dict[str, Any] = json.loads(path.read_text())
        return instance

    def stage_glb(self, scene_instance: dict[str, Any]) -> Path:
        # template_name is a dataset-relative path such as "stages/102344193".
        config = (
            self.root / f"{scene_instance['stage_instance']['template_name']}.stage_config.json"
        )
        asset: str = json.loads(config.read_text())["render_asset"]
        return config.parent / asset

    def object_glb(self, template_name: str) -> Path:
        try:
            config = self._object_configs[template_name]
        except KeyError:
            raise ValueError(
                f"no object config for template {template_name!r} under {self.root / 'objects'}"
            ) from None
        asset: str = json.loads(config.read_text())["render_asset"]
        return config.parent / asset

    @cached_property
    def semantics(self) -> dict[str, ObjectSemantics]:
        with (self.root / "semantics" / "objects.csv").open(newline="") as f:
            return {
                row["id"]: ObjectSemantics(row["name"], row["main_category"], row["super_category"])
                for row in csv.DictReader(f)
            }

    @cached_property
    def _object_configs(self) -> dict[str, Path]:
        # One index over objects/<hex>/, objects/decomposed/<hash>/ and objects/openings/,
        # mirroring the dataset config's object search paths.
        suffix = ".object_config.json"
        index: dict[str, Path] = {}
        for path in sorted((self.root / "objects").rglob(f"*{suffix}")):
            stem = path.name[: -len(suffix)]
            if stem in index:
                raise ValueError(f"duplicate object config {stem!r}: {index[stem]} and {path}")
            index[stem] = path
        return index


def load_glb(path: Path) -> Any:
    """Load a GLB as a trimesh scene, keeping its node graph and geometry as authored."""
    return trimesh.load(str(path), force="scene", process=False)


def subtree_vertices(scene: Any, node: str) -> Vertices:
    """World-space vertices of every geometry under ``node``, itself included."""
    chunks: list[Vertices] = []
    stack = [node]
    while stack:
        current = stack.pop()
        transform, geometry = scene.graph[current]
        if geometry is not None:
            local = np.asarray(scene.geometry[geometry].vertices, dtype=np.float64)
            chunks.append(np.asarray(trimesh.transform_points(local, transform), dtype=np.float64))
        stack.extend(scene.graph.transforms.children.get(current, ()))
    return np.vstack(chunks) if chunks else np.zeros((0, 3), dtype=np.float64)


def instance_world_vertices(
    local: Vertices,
    translation: Sequence[float],
    rotation_wxyz: Sequence[float],
    scale: Sequence[float],
) -> Vertices:
    """Place asset-local vertices: scale, rotate (WXYZ quaternion), then translate.

    HSSD scene instances use ``translation_origin: asset_local``, so the
    translation is the asset's own origin and no center-of-mass shift applies.
    """
    scaled = local * np.asarray(scale, dtype=np.float64)
    return scaled @ rotation_matrix(rotation_wxyz).T + np.asarray(translation, dtype=np.float64)


def rotation_matrix(wxyz: Sequence[float]) -> NDArray[np.float64]:
    """3x3 rotation matrix of a WXYZ quaternion (normalized first)."""
    q = np.asarray(wxyz, dtype=np.float64)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def object_labels(
    template_name: str, semantics: dict[str, ObjectSemantics], mode: LabelMode
) -> tuple[str, ...]:
    """Detection labels for a template: category and/or product name, else the model hash.

    Decomposed parts (``<hash>_part_N``) share their base model's row.
    """
    base = template_name.split("_part_", 1)[0]
    row = semantics.get(base)
    category = row.main_category if row else ""
    name = row.name if row else ""
    ordered = (name, category) if mode == "name" else (category, name)
    labels = tuple(dict.fromkeys(label for label in ordered if label))
    if mode != "category+name":
        labels = labels[:1]
    return labels or (base,)


def object_boxes(
    dataset: HssdDataset,
    scene_instance: dict[str, Any],
    *,
    label_mode: LabelMode = DEFAULT_LABEL_MODE,
    cache: dict[str, Vertices] | None = None,
) -> list[GroundTruthBox]:
    """Tight world boxes of every placed object, ids ``<template>@<index>``.

    ``cache`` holds asset-local vertices per template so repeated templates load once.
    """
    cache = {} if cache is None else cache
    boxes = []
    for index, instance in enumerate(scene_instance["object_instances"]):
        template = instance["template_name"]
        local = cache.get(template)
        if local is None:
            asset = load_glb(dataset.object_glb(template))
            local = cache[template] = subtree_vertices(asset, asset.graph.base_frame)
        if local.shape[0] == 0:
            raise ValueError(f"object template {template!r} has no vertices")
        world = instance_world_vertices(
            local,
            instance["translation"],
            instance["rotation"],
            instance.get("non_uniform_scale", (1.0, 1.0, 1.0)),
        )
        boxes.append(
            GroundTruthBox(
                id=f"{template}@{index}",
                labels=object_labels(template, dataset.semantics, label_mode),
                min=_corner(world.min(axis=0)),
                max=_corner(world.max(axis=0)),
            )
        )
    return boxes


def wall_boxes(stage: Any) -> list[GroundTruthBox]:
    """One box per wall, from the stage's ceiling-height ``wallTop`` faces.

    HSSD stages carry the top face of every wall, exterior and interior, in
    meshes named ``geometry_wallTop*``: some under ``wall_<n>`` groups, the rest
    merged into one top-level mesh. Each face is a flat quad at ceiling height.
    Touching collinear faces merge into one segment while the union stays thinner
    than :data:`WALL_MAX_THICKNESS_M`, so perpendicular walls stay separate; each
    segment is extruded down to the stage floor. Walls are ordered by footprint
    position (ROS x, then y) and named ``wall_<k>`` with label ``wall``.
    """
    floor_y = float(subtree_vertices(stage, stage.graph.base_frame)[:, 1].min())
    faces: list[tuple[Vertices, Vertices]] = []
    for node in stage.graph.nodes_geometry:
        if not WALL_TOP_PATTERN.search(node):
            continue
        transform, geometry = stage.graph[node]
        for piece in stage.geometry[geometry].split(only_watertight=False):
            vertices = np.asarray(
                trimesh.transform_points(piece.vertices, transform), dtype=np.float64
            )
            lo, hi = vertices.min(axis=0), vertices.max(axis=0)
            if hi[1] - lo[1] > WALL_FLAT_MAX_M or lo[1] < floor_y + WALL_TOP_MIN_HEIGHT_M:
                continue
            faces.append((lo, hi))
    segments = [
        (lo, hi)
        for lo, hi in _merge_wall_faces(faces)
        if max(hi[0] - lo[0], hi[2] - lo[2]) >= WALL_MIN_LENGTH_M
    ]
    # Habitat (x, y, z) -> ROS (-z, -x, y): order by the ROS min corner.
    segments.sort(key=lambda seg: (round(-seg[1][2], 3), round(-seg[1][0], 3)))
    return [
        GroundTruthBox(
            id=f"wall_{index:03d}",
            labels=(WALL_LABEL,),
            min=_corner(np.array([lo[0], floor_y, lo[2]])),
            max=_corner(hi),
        )
        for index, (lo, hi) in enumerate(segments)
    ]


def _merge_wall_faces(faces: list[tuple[Vertices, Vertices]]) -> list[tuple[Vertices, Vertices]]:
    """Union touching faces into wall segments.

    Two footprints merge when they touch and the union is still wall-thin, or
    when they nearly coincide; a thick union of distinct pieces (an L or T
    junction) is never merged.
    """
    segments: list[tuple[Vertices, Vertices]] = []
    for lo, hi in faces:
        merged = True
        while merged:
            merged = False
            for index, (seg_lo, seg_hi) in enumerate(segments):
                if not _footprints_touch(lo, hi, seg_lo, seg_hi):
                    continue
                union_lo, union_hi = np.minimum(lo, seg_lo), np.maximum(hi, seg_hi)
                thin = (
                    min(union_hi[0] - union_lo[0], union_hi[2] - union_lo[2])
                    <= WALL_MAX_THICKNESS_M
                )
                # Against the smaller piece: near-duplicates merge, but a chain of partial
                # overlaps can never grow a segment step by step.
                coincident = _footprint_area(
                    union_lo, union_hi
                ) <= WALL_COINCIDENT_AREA_RATIO * min(
                    _footprint_area(lo, hi), _footprint_area(seg_lo, seg_hi)
                )
                if not (thin or coincident):
                    continue
                lo, hi = union_lo, union_hi
                del segments[index]
                merged = True
                break
        segments.append((lo, hi))
    return segments


def _footprint_area(lo: Vertices, hi: Vertices) -> float:
    return float((hi[0] - lo[0]) * (hi[2] - lo[2]))


def _footprints_touch(a_lo: Vertices, a_hi: Vertices, b_lo: Vertices, b_hi: Vertices) -> bool:
    tol = WALL_MERGE_TOLERANCE_M
    return bool(
        a_lo[0] - tol <= b_hi[0]
        and a_hi[0] + tol >= b_lo[0]
        and a_lo[2] - tol <= b_hi[2]
        and a_hi[2] + tol >= b_lo[2]
    )


def scene_boxes(
    dataset: HssdDataset, scene_id: str, *, label_mode: LabelMode = DEFAULT_LABEL_MODE
) -> SceneBoxes:
    instance = dataset.scene_instance(scene_id)
    return SceneBoxes(
        objects=object_boxes(dataset, instance, label_mode=label_mode),
        walls=wall_boxes(load_glb(dataset.stage_glb(instance))),
    )


def scene_detections(
    dataset: HssdDataset, scene_id: str, *, label_mode: LabelMode = DEFAULT_LABEL_MODE
) -> Detection3DArray:
    """Objects then walls of ``scene_id`` as a ``Detection3DArray`` in the ROS ``world`` frame."""
    return boxes_to_detection3d_array(
        scene_boxes(dataset, scene_id, label_mode=label_mode).all,
        to_ros=habitat_to_ros,
        frame_id=HABITAT_WORLD_FRAME,
        ts=GROUND_TRUTH_TS,
    )


def export_scene(
    dataset: HssdDataset,
    scene_id: str,
    out_dir: str | Path,
    *,
    label_mode: LabelMode = DEFAULT_LABEL_MODE,
) -> Path:
    """Write ``<out_dir>/<scene_id>.json`` and its top-down ``<scene_id>.top_down.json``.

    Both are JSON views with dataset provenance; the top-down file holds the
    ``Detection2DArray`` from :func:`dimos.simulation.object_detections.top_down`.
    """
    boxes = scene_boxes(dataset, scene_id, label_mode=label_mode)
    detections = boxes_to_detection3d_array(
        boxes.all, to_ros=habitat_to_ros, frame_id=HABITAT_WORLD_FRAME, ts=GROUND_TRUTH_TS
    )
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    provenance = {"dataset": dataset.name, "scene_id": scene_id}
    path = write_detection3d_json(detections, out / f"{scene_id}.json", provenance=provenance)
    flat = top_down(detections)
    write_detection2d_json(flat, out / f"{scene_id}.top_down.json", provenance=provenance)
    logger.info(
        "wrote HSSD ground truth",
        scene_id=scene_id,
        objects=len(boxes.objects),
        walls=len(boxes.walls),
        top_down=flat.detections_length,
        path=str(path),
    )
    return path


def export_scenes(
    dataset: HssdDataset,
    out_dir: str | Path,
    scene_ids: Sequence[str] = HSSD_SCENE_IDS,
    *,
    label_mode: LabelMode = DEFAULT_LABEL_MODE,
) -> list[Path]:
    return [
        export_scene(dataset, scene_id, out_dir, label_mode=label_mode) for scene_id in scene_ids
    ]


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Export HSSD scene objects and walls as Detection3DArray JSON ground truth."
    )
    parser.add_argument(
        "--dataset",
        help=f"HSSD scene dataset config (default: ${HSSD_DATASET_CONFIG_ENV}, "
        f"then {DEFAULT_HSSD_DATASET_CONFIG})",
    )
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT_DIR, help="output directory")
    parser.add_argument(
        "--scene",
        action="append",
        dest="scenes",
        metavar="ID",
        help="scene id, repeatable (default: the QA suite scenes)",
    )
    parser.add_argument("--label-mode", choices=LABEL_MODES, default=DEFAULT_LABEL_MODE)
    args = parser.parse_args(argv)
    dataset = HssdDataset.from_env(args.dataset)
    export_scenes(
        dataset,
        args.out,
        args.scenes or HSSD_SCENE_IDS,
        label_mode=cast("LabelMode", args.label_mode),
    )


def _corner(point: Vertices) -> Point3:
    x, y, z = np.round(point, CORNER_DECIMALS).tolist()
    # "+ 0.0" folds negative zero into 0.0 so the JSON stays clean.
    return (float(x) + 0.0, float(y) + 0.0, float(z) + 0.0)


if __name__ == "__main__":
    main()
