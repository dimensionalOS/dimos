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
"""Canonical public/oracle spatial corpus bundle writer."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from dimos.benchmark.spatial.map_generation import VariantAlignment
from dimos.benchmark.spatial.models import (
    EmptyGeometry,
    Geometry,
    Instance,
    Manifest,
    Marker,
    MarkerGeometry,
    OccupancyGeometry,
    OracleQuestionGeometry,
    RotationGeometry,
    Scene,
    Snapshot,
    SourceProvenance,
    SpatialModel,
    Topology,
    Trajectory,
    TranslationGeometry,
    write_record_schemas,
)
from dimos.benchmark.spatial.questions import (
    PhysicalQuestion,
    executable_definitions,
    public_question_index,
    question_identity_payload,
)
from dimos.benchmark.spatial.utilities import canonical_json, hash_file_sha256, stable_opaque_id


@dataclass(frozen=True)
class SnapshotVariant:
    """One already-written Snapshot and its directory containing map bytes."""

    snapshot: Snapshot
    directory: Path
    alignment: VariantAlignment


@dataclass(frozen=True)
class BundleInput:
    """Strict generation input; private records are never copied under public/."""

    manifest: Manifest
    scene: Scene
    trajectory: Trajectory
    source: SourceProvenance
    geometry: Geometry
    topology: Topology
    questions: tuple[PhysicalQuestion, ...]
    variants: tuple[SnapshotVariant, ...]


@dataclass(frozen=True)
class BundleResult:
    public_root: Path
    oracle_root: Path
    question_count: int
    instance_count: int


def write_bundle(root: Path, bundle: BundleInput) -> BundleResult:
    """Write design-v1 hierarchy with canonical JSON/JSONL and paired instances."""

    variants = tuple(sorted(bundle.variants, key=lambda item: item.snapshot.variant.value))
    expected_variants = {"clean", "noisy-01", "noisy-02"}
    if {item.snapshot.variant.value for item in variants} != expected_variants or len(
        variants
    ) != 3:
        raise ValueError("bundle requires exactly one each of clean/noisy-01/noisy-02")
    snapshots = tuple(item.snapshot for item in variants)
    scene_id, trajectory_id = snapshots[0].scene_id, snapshots[0].trajectory_id
    if {snapshot.scene_id for snapshot in snapshots} != {scene_id} or {
        snapshot.trajectory_id for snapshot in snapshots
    } != {trajectory_id}:
        raise ValueError("all snapshots must belong to one scene and trajectory")
    if len({snapshot.variant for snapshot in snapshots}) != len(snapshots):
        raise ValueError("snapshot variants must be unique")
    questions = tuple(sorted(bundle.questions, key=lambda item: item.question.question_id))
    _validate_refs(bundle, questions, scene_id, trajectory_id)
    public_trajectory = root / "public" / "scenes" / scene_id / "trajectories" / trajectory_id
    oracle_trajectory = root / "oracle" / "scenes" / scene_id / "trajectories" / trajectory_id
    _write_model(root / "manifest.json", bundle.manifest)
    write_record_schemas(root / "schemas")
    _write_model(public_trajectory.parent.parent / "scene.json", bundle.scene)
    _write_model(public_trajectory / "trajectory.json", bundle.trajectory)
    _write_jsonl(public_trajectory / "questions.jsonl", tuple(item.question for item in questions))
    _write_model(root / "oracle" / "scenes" / scene_id / "source.json", bundle.source)
    _write_model(root / "oracle" / "scenes" / scene_id / "geometry.json", bundle.geometry)
    _write_model(root / "oracle" / "scenes" / scene_id / "topology.json", bundle.topology)
    _write_jsonl(oracle_trajectory / "answers.jsonl", tuple(item.answer for item in questions))
    _write_jsonl(
        oracle_trajectory / "question_geometry.jsonl",
        tuple(
            OracleQuestionGeometry(
                question_id=item.question.question_id, pose=item.pose, markers=item.markers
            )
            for item in questions
        ),
    )
    _write_jsonl(oracle_trajectory / "review_overrides.jsonl", ())
    instances = 0
    for variant in variants:
        variant_directory = public_trajectory / "variants" / variant.snapshot.variant.value
        _write(
            variant_directory / "snapshot.json",
            canonical_json(variant.snapshot.model_dump(mode="json")) + b"\n",
        )
        source_map = variant.directory / variant.snapshot.map_artifact_path
        target_map = variant_directory / "global_map.pc2.lcm"
        target_map.parent.mkdir(parents=True, exist_ok=True)
        target_map.write_bytes(source_map.read_bytes())
        if hash_file_sha256(source_map) != variant.snapshot.map_artifact_sha256:
            raise ValueError("snapshot map hash does not match source artifact")
        if hash_file_sha256(target_map) != variant.snapshot.map_artifact_sha256:
            raise ValueError("copied map hash does not match snapshot")
        projected = tuple(
            _instance(item, variant.snapshot, variant.alignment) for item in questions
        )
        _write_jsonl(variant_directory / "instances.jsonl", projected)
        instances += len(projected)
    _validate_written_paths(root, bundle, variants)
    _scan_public_for_leakage(root / "public", bundle)
    return BundleResult(root / "public", root / "oracle", len(questions), instances)


def _instance(
    physical: PhysicalQuestion, snapshot: Snapshot, alignment: VariantAlignment
) -> Instance:
    geometry = _project_geometry(physical, alignment)
    return Instance(
        instance_id=stable_opaque_id(
            "instance",
            {"question": physical.question.question_id, "snapshot": snapshot.snapshot_id},
        ),
        question_id=physical.question.question_id,
        snapshot_id=snapshot.snapshot_id,
        scene_id=snapshot.scene_id,
        trajectory_id=snapshot.trajectory_id,
        variant=snapshot.variant,
        query_geometry=geometry,
    )


def _project_geometry(
    physical: PhysicalQuestion, alignment: VariantAlignment
) -> EmptyGeometry | OccupancyGeometry | TranslationGeometry | RotationGeometry | MarkerGeometry:
    if physical.markers:
        return MarkerGeometry(
            markers=tuple(
                Marker(
                    marker_id=marker.marker_id,
                    position=alignment.project_point(marker.position),
                )
                for marker in physical.markers
            )
        )
    if physical.pose is None:
        return EmptyGeometry()
    pose = alignment.project_pose(physical.pose)
    if physical.question.predicate.value == "pose-occupancy":
        return OccupancyGeometry(pose=pose)
    if physical.question.predicate.value == "straight-translation":
        return TranslationGeometry(start_pose=pose)
    return RotationGeometry(pose=pose)


def _write_model(path: Path, record: SpatialModel) -> None:
    _write(path, canonical_json(record.model_dump(mode="json")) + b"\n")


def _validate_refs(
    bundle: BundleInput, questions: tuple[PhysicalQuestion, ...], scene_id: str, trajectory_id: str
) -> None:
    if bundle.scene.scene_id != scene_id or bundle.trajectory.trajectory_id != trajectory_id:
        raise ValueError("scene/trajectory records must match snapshots")
    manifest_scene = next(
        (item for item in bundle.manifest.scenes if item.scene_id == scene_id), None
    )
    if manifest_scene is None or manifest_scene.split != bundle.scene.split:
        raise ValueError("manifest must reference scene with matching split")
    if manifest_scene.scene_path != f"public/scenes/{scene_id}/scene.json":
        raise ValueError("manifest scene_path must be root-relative public scene path")
    if (
        bundle.trajectory.scene_id != scene_id
        or bundle.source.scene_id != scene_id
        or bundle.geometry.scene_id != scene_id
        or bundle.topology.scene_id != scene_id
    ):
        raise ValueError("private records must reference the bundle scene")
    if trajectory_id not in bundle.scene.trajectory_ids:
        raise ValueError("scene must reference trajectory")
    question_ids = [item.question.question_id for item in questions]
    if len(set(question_ids)) != len(question_ids):
        raise ValueError("question IDs must be unique")
    if len({item.question.text for item in questions}) != len(questions):
        raise ValueError("question texts must be unique")
    answer_ids = [item.answer.question_id for item in questions]
    if sorted(answer_ids) != sorted(question_ids):
        raise ValueError("answers must pair one-to-one with questions")
    if any(
        item.question.scene_id != scene_id or item.question.trajectory_id != trajectory_id
        for item in questions
    ):
        raise ValueError("questions must belong to snapshot scene and trajectory")
    definitions = executable_definitions()
    for item in questions:
        definition = definitions[item.question.predicate]
        if item.answer.oracle_policy_version != definition:
            raise ValueError("answer oracle policy version does not match executable definition")
        if item.question.question_id != stable_opaque_id(
            "question",
            question_identity_payload(
                scene_id=scene_id,
                trajectory_id=trajectory_id,
                predicate=item.question.predicate,
                index=public_question_index(item.question),
                definition=definition,
            ),
        ):
            raise ValueError("question stable ID does not match executable identity")
        for marker in item.markers:
            if not marker.marker_id.startswith("marker_"):
                raise ValueError("marker IDs must use marker_ namespace")
    for record_id, prefix in (
        (scene_id, "scene_"),
        (trajectory_id, "trajectory_"),
        (bundle.manifest.release_id, "release_"),
    ):
        if not record_id.startswith(prefix):
            raise ValueError(f"record ID {record_id!r} must use {prefix} namespace")


def _validate_written_paths(
    root: Path, bundle: BundleInput, variants: tuple[SnapshotVariant, ...]
) -> None:
    required = [root / "manifest.json", root / "schemas"]
    required.extend(root / scene.scene_path for scene in bundle.manifest.scenes)
    for variant in variants:
        variant_root = (
            root
            / "public"
            / "scenes"
            / variant.snapshot.scene_id
            / "trajectories"
            / variant.snapshot.trajectory_id
            / "variants"
            / variant.snapshot.variant.value
        )
        required.append(variant_root / variant.snapshot.map_artifact_path)
    for path in required:
        if not path.exists():
            raise ValueError(f"referenced relative path does not exist: {path.relative_to(root)}")


def _scan_public_for_leakage(public_root: Path, bundle: BundleInput) -> None:
    forbidden_keys = {"room_id", "source_scene_key", "source_artifact_sha256"}
    forbidden_paths = {"oracle"}
    forbidden_exact_values = {
        bundle.source.source_scene_key,
        "source-provenance",
        "geometry",
        "topology",
    }
    for path in sorted(public_root.rglob("*")):
        if path.is_file() and path.suffix in {".json", ".jsonl"}:
            if any(part in forbidden_paths for part in path.relative_to(public_root).parts):
                raise ValueError(f"public bundle leaks private path segment in {path}")
            for record in _iter_json_records(path):
                _reject_private_fields(record, forbidden_keys, forbidden_exact_values, path)


def _iter_json_records(path: Path) -> tuple[object, ...]:
    if path.suffix == ".jsonl":
        return tuple(
            json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line
        )
    return (json.loads(path.read_text(encoding="utf-8")),)


def _reject_private_fields(
    value: object, forbidden_keys: set[str], forbidden_values: set[str], path: Path
) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key in forbidden_keys:
                raise ValueError(f"public bundle leaks private field {key!r} in {path}")
            _reject_private_fields(child, forbidden_keys, forbidden_values, path)
    elif isinstance(value, list):
        for child in value:
            _reject_private_fields(child, forbidden_keys, forbidden_values, path)
    elif isinstance(value, str) and value in forbidden_values:
        raise ValueError(f"public bundle leaks private value {value!r} in {path}")


def _write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)


def _write_jsonl(path: Path, records: tuple[SpatialModel, ...]) -> None:
    serialized = b"".join(
        canonical_json(record.model_dump(mode="json")) + b"\n" for record in records
    )
    _write(path, serialized)
