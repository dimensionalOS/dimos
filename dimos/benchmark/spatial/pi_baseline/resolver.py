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

# Copyright 2026 Dimensional Inc.
"""Exact public corpus resolution; oracle discovery is intentionally absent."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import TypeVar

from dimos.benchmark.spatial.corpus_loader import SpatialCorpusInstance
from dimos.benchmark.spatial.models import (
    Instance,
    Manifest,
    MapVariant,
    Question,
    Scene,
    Snapshot,
    SpatialModel,
    Trajectory,
)

_RecordT = TypeVar("_RecordT", bound=SpatialModel)


def resolve_public_instance(
    corpus_root: Path,
    *,
    scene_id: str,
    trajectory_id: str,
    question_id: str,
    variant: str,
    instance_id: str,
) -> SpatialCorpusInstance:
    """Resolve one exact public instance, rejecting missing or ambiguous records.

    This function never constructs an oracle root and never uses ``require_one``.
    Every supplied identity is mandatory so a caller cannot accidentally receive
    the loader's first match.
    """
    corpus_root = corpus_root.resolve()
    public_root = (corpus_root / "public").resolve()
    _public_file(corpus_root / "manifest.json", corpus_root)
    manifest = Manifest.model_validate_json((corpus_root / "manifest.json").read_bytes())
    scene_matches = [item for item in manifest.scenes if item.scene_id == scene_id]
    if len(scene_matches) != 1:
        raise ValueError(f"expected exactly one public scene, found {len(scene_matches)}")
    scene_entry = scene_matches[0]
    scene_path = _public_file(corpus_root / scene_entry.scene_path, public_root)
    scene = Scene.model_validate_json(scene_path.read_bytes())
    if scene.scene_id != scene_id:
        raise ValueError("scene record does not match requested identity")
    if len(set(scene.trajectory_ids)) != len(scene.trajectory_ids):
        raise ValueError("scene contains duplicate trajectory references")
    if trajectory_id not in scene.trajectory_ids:
        raise ValueError("trajectory_id is not referenced by scene")
    root = public_root / "scenes" / scene_id / "trajectories" / trajectory_id
    trajectory = Trajectory.model_validate_json(
        _public_file(root / "trajectory.json", public_root).read_bytes()
    )
    if trajectory.scene_id != scene_id or trajectory.trajectory_id != trajectory_id:
        raise ValueError("trajectory references do not match requested identity")
    questions = tuple(
        Question.model_validate_json(line)
        for line in _public_file(root / "questions.jsonl", public_root).read_text().splitlines()
        if line
    )
    for item in questions:
        if item.scene_id != scene_id or item.trajectory_id != trajectory_id:
            raise ValueError("question references do not match requested identity")
    question = _exact(questions, lambda item: item.question_id == question_id, "question_id")
    try:
        map_variant = MapVariant(variant)
    except ValueError as error:
        raise ValueError("variant is not a public map variant") from error
    variant_root = root / "variants" / map_variant.value
    snapshot = Snapshot.model_validate_json(
        _public_file(variant_root / "snapshot.json", public_root).read_bytes()
    )
    if snapshot.scene_id != scene_id or snapshot.trajectory_id != trajectory_id:
        raise ValueError("snapshot references do not match requested identity")
    if snapshot.variant is not map_variant:
        raise ValueError("snapshot variant does not match requested identity")
    map_path = (variant_root / snapshot.map_artifact_path).resolve()
    if map_path.is_symlink() or public_root not in map_path.parents or not map_path.is_file():
        raise ValueError("snapshot map artifact is not inside the public corpus")
    if map_path.relative_to(variant_root).as_posix() != snapshot.map_artifact_path:
        raise ValueError("snapshot map artifact path escapes its variant")
    instances = tuple(
        Instance.model_validate_json(line)
        for line in _public_file(variant_root / "instances.jsonl", public_root)
        .read_text()
        .splitlines()
        if line
    )
    for item in instances:
        if (item.scene_id, item.trajectory_id, item.variant) != (
            scene_id,
            trajectory_id,
            map_variant,
        ):
            raise ValueError("instance references do not match requested identity")
    instance = _exact(instances, lambda item: item.instance_id == instance_id, "instance_id")
    if instance.question_id != question.question_id or instance.snapshot_id != snapshot.snapshot_id:
        raise ValueError("instance references do not match requested question or snapshot")
    return SpatialCorpusInstance(
        corpus_root,
        corpus_root / "public",
        None,
        scene,
        trajectory,
        question,
        snapshot,
        instance,
        variant_root,
        None,
    )


def _public_file(path: Path, public_root: Path) -> Path:
    """Return a regular, non-link file contained by the public release."""
    if path.is_symlink() or any(
        parent.is_symlink() for parent in path.parents if parent != public_root
    ):
        raise ValueError("public corpus must not contain symlinks")
    resolved = path.resolve()
    if resolved != public_root and public_root not in resolved.parents:
        raise ValueError("public corpus path escapes public root")
    if not resolved.is_file() or resolved.is_symlink():
        raise ValueError(f"public corpus file does not exist: {path}")
    return resolved


def _exact(
    items: tuple[_RecordT, ...], predicate: Callable[[_RecordT], bool], name: str
) -> _RecordT:
    matches = [item for item in items if predicate(item)]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one public {name}, found {len(matches)}")
    return matches[0]
