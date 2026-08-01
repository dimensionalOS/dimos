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
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Read-only loader for static spatial benchmark inspection."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TypeVar

from pydantic import BaseModel

from dimos.benchmark.spatial.models import (
    Answer,
    Geometry,
    Instance,
    Manifest,
    MapVariant,
    OracleQuestionGeometry,
    Predicate,
    Question,
    Scene,
    Snapshot,
    SourceProvenance,
    Topology,
    Trajectory,
)

_ModelT = TypeVar("_ModelT", bound=BaseModel)


@dataclass(frozen=True)
class OracleRecords:
    """Private records joined only when an oracle root is supplied."""

    source: SourceProvenance
    geometry: Geometry
    topology: Topology
    answers: tuple[Answer, ...]
    question_geometries: tuple[OracleQuestionGeometry, ...]


@dataclass(frozen=True)
class SpatialCorpusInstance:
    """One fully resolved public instance and optional oracle join."""

    corpus_root: Path
    public_root: Path
    oracle_root: Path | None
    scene: Scene
    trajectory: Trajectory
    question: Question
    snapshot: Snapshot
    instance: Instance
    variant_root: Path
    oracle: OracleRecords | None = None

    @property
    def question_text(self) -> str:
        return self.question.text


@dataclass(frozen=True)
class SpatialCorpusSelection:
    scene_id: str | None = None
    trajectory_id: str | None = None
    question_id: str | None = None
    predicate: Predicate | None = None
    variant: MapVariant | None = None
    instance_id: str | None = None


class SpatialCorpusLoader:
    """Load immutable corpus records without writing to the public or oracle roots."""

    def __init__(self, corpus_root: Path, oracle_root: Path | None = None) -> None:
        self.corpus_root = corpus_root
        self.public_root = corpus_root / "public"
        self.oracle_root = (
            oracle_root if oracle_root is not None else _default_oracle_root(corpus_root)
        )
        self.manifest = _load_json_model(corpus_root / "manifest.json", Manifest)

    def instances(
        self, selection: SpatialCorpusSelection | None = None
    ) -> tuple[SpatialCorpusInstance, ...]:
        """Return all instances matching the requested scene/trajectory/question/variant filters."""

        selection = selection or SpatialCorpusSelection()
        loaded: list[SpatialCorpusInstance] = []
        for manifest_scene in self.manifest.scenes:
            if selection.scene_id is not None and manifest_scene.scene_id != selection.scene_id:
                continue
            scene = _load_json_model(self.corpus_root / manifest_scene.scene_path, Scene)
            for trajectory_id in scene.trajectory_ids:
                if selection.trajectory_id is not None and trajectory_id != selection.trajectory_id:
                    continue
                trajectory_root = (
                    self.public_root / "scenes" / scene.scene_id / "trajectories" / trajectory_id
                )
                trajectory = _load_json_model(trajectory_root / "trajectory.json", Trajectory)
                questions = _load_jsonl_models(trajectory_root / "questions.jsonl", Question)
                questions_by_id = {question.question_id: question for question in questions}
                oracle = self._load_oracle(scene.scene_id, trajectory_id)
                for variant in MapVariant:
                    if selection.variant is not None and variant is not selection.variant:
                        continue
                    variant_root = trajectory_root / "variants" / variant.value
                    snapshot = _load_json_model(variant_root / "snapshot.json", Snapshot)
                    for instance in _load_jsonl_models(variant_root / "instances.jsonl", Instance):
                        question = questions_by_id[instance.question_id]
                        if not _matches(selection, question, instance):
                            continue
                        loaded.append(
                            SpatialCorpusInstance(
                                corpus_root=self.corpus_root,
                                public_root=self.public_root,
                                oracle_root=self.oracle_root,
                                scene=scene,
                                trajectory=trajectory,
                                question=question,
                                snapshot=snapshot,
                                instance=instance,
                                variant_root=variant_root,
                                oracle=oracle,
                            )
                        )
        return tuple(loaded)

    def require_one(self, selection: SpatialCorpusSelection | None = None) -> SpatialCorpusInstance:
        matches = self.instances(selection)
        if not matches:
            raise ValueError("no corpus instance matches selection")
        return matches[0]

    def next_instance(self, current: SpatialCorpusInstance, step: int = 1) -> SpatialCorpusInstance:
        matches = self.instances(SpatialCorpusSelection(scene_id=current.scene.scene_id))
        return matches[(_index_of(matches, current.instance.instance_id) + step) % len(matches)]

    def variant(self, current: SpatialCorpusInstance, variant: MapVariant) -> SpatialCorpusInstance:
        return self.require_one(
            SpatialCorpusSelection(
                scene_id=current.scene.scene_id,
                trajectory_id=current.trajectory.trajectory_id,
                question_id=current.question.question_id,
                variant=variant,
            )
        )

    def _load_oracle(self, scene_id: str, trajectory_id: str) -> OracleRecords | None:
        if self.oracle_root is None or not self.oracle_root.exists():
            return None
        scene_root = self.oracle_root / "scenes" / scene_id
        trajectory_root = scene_root / "trajectories" / trajectory_id
        return OracleRecords(
            source=_load_json_model(scene_root / "source.json", SourceProvenance),
            geometry=_load_json_model(scene_root / "geometry.json", Geometry),
            topology=_load_json_model(scene_root / "topology.json", Topology),
            answers=_load_jsonl_models(trajectory_root / "answers.jsonl", Answer),
            question_geometries=_load_jsonl_models(
                trajectory_root / "question_geometry.jsonl", OracleQuestionGeometry
            ),
        )


def _default_oracle_root(corpus_root: Path) -> Path | None:
    path = corpus_root / "oracle"
    return path if path.exists() else None


def _matches(selection: SpatialCorpusSelection, question: Question, instance: Instance) -> bool:
    if selection.question_id is not None and question.question_id != selection.question_id:
        return False
    if selection.predicate is not None and question.predicate is not selection.predicate:
        return False
    if selection.instance_id is not None and instance.instance_id != selection.instance_id:
        return False
    return True


def _index_of(instances: tuple[SpatialCorpusInstance, ...], instance_id: str) -> int:
    for index, instance in enumerate(instances):
        if instance.instance.instance_id == instance_id:
            return index
    raise ValueError("current instance is not present in loader results")


def _load_json_model(path: Path, model: type[_ModelT]) -> _ModelT:
    return model.model_validate_json(path.read_bytes())


def _load_jsonl_models(path: Path, model: type[_ModelT]) -> tuple[_ModelT, ...]:
    return tuple(
        model.model_validate_json(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line
    )
