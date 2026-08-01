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
"""Smoke corpus generation and gate for the static spatial benchmark."""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

import numpy as np

from dimos.benchmark.spatial.bundles import BundleInput, SnapshotVariant, write_bundle
from dimos.benchmark.spatial.collision_oracle import SquareFootprintCollisionOracle
from dimos.benchmark.spatial.config import SPATIAL_BENCHMARK_V1, derive_v1_seed
from dimos.benchmark.spatial.corpus_loader import SpatialCorpusLoader, SpatialCorpusSelection
from dimos.benchmark.spatial.map_generation import (
    PoseAlignment,
    VariantAlignment,
    generate_full_coverage_trajectory,
    generate_map,
    write_snapshot,
)
from dimos.benchmark.spatial.models import (
    FrameConventionRecord,
    FreeSpaceModel,
    Geometry,
    Manifest,
    ManifestScene,
    MapperConfigurationRecord,
    MapVariant,
    OpeningEdge,
    Point2D,
    Polygon2D,
    Pose2D,
    Predicate,
    Question,
    Room,
    Scene,
    Snapshot,
    SourceProvenance,
    Split,
    Topology,
    Trajectory,
)
from dimos.benchmark.spatial.questions import PhysicalQuestion, generate_physical_questions
from dimos.benchmark.spatial.structured3d import (
    SourceAxisTransform,
    Structured3DError,
    Structured3DImport,
    load_structured3d_scene,
)
from dimos.benchmark.spatial.utilities import (
    JsonValue,
    canonical_json,
    hash_file_sha256,
    stable_opaque_id,
)
from dimos.benchmark.spatial.validation import (
    ValidationFailure,
    validate_release,
    write_validation_report,
)
from dimos.benchmark.spatial.viewer import SpatialCorpusViserView, ViserReadOnlyBoundary
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

SMOKE_REPORT_NAME = "smoke_validation_report.json"
REQUIRED_SMOKE_CHECKS = (
    "schema",
    "artifact-decode-hash",
    "coordinate",
    "oracle",
    "pairing",
    "topology",
    "collision",
    "leakage",
    "viser-load",
)
_PILOT_SCENE_COUNT = 30
_PILOT_CANDIDATE_POOL_SIZE = _PILOT_SCENE_COUNT
README_NAME = "README.md"

_CORPUS_README = """# Static spatial QA benchmark v1

This corpus-local README describes the deterministic v1 release contract. The
release contains 30 scenes: 10 development and 20 held-out. Each scene has 13
physical questions, for 390 questions total. Every question is provided in
three map variants (`clean`, `noisy-01`, and `noisy-02`), for 1,170
map-question instances.

## Predicates

- `pose-occupancy`: whether a specified robot pose is collision-free.
- `straight-translation`: whether a straight motion between poses is feasible.
- `in-place-rotation`: whether rotation at a pose is collision-free.
- `eligible-room-count`: the count of rooms eligible under the question rule.
- `same-room`: whether two specified locations share a room.
- `direct-room-connection`: whether two rooms have a direct connection.
- `direct-neighbor-count`: the number of rooms directly connected to a room.

## Source and generation

The source is gated Structured3D `annotation_3d.json`; source data is not
included in this corpus. Generation validates a 2-D private oracle, derives a
deterministic coverage trajectory and horizontal lidar observations, then uses
seeded clean/noisy observations and drift. `VoxelGridMapper` emits native
`PointCloud2` LCM snapshots. The generator writes executable labels and
public/oracle bundles, then validates the release.

## Layout and privacy

`public/` contains agent-visible scenes, trajectories, questions, snapshots,
and instances. `oracle/` contains private geometry, topology, executable
labels, answers, and review material. Keep oracle files and answers private;
do not use them for agent inputs or publish them with results.

Validation reports are `smoke_validation_report.json`,
`release_validation_report.json`, and, when generation is blocked,
`pilot_generation_report.json`.

## Inspection

Run the read-only Viser viewer:

```bash
python -m dimos.benchmark.spatial.cli view --root . --public-only
```

## Access and redistribution

This Structured3D-derived corpus is gated and non-redistributable unless
explicit rights clearance is recorded. See
`docs/development/static_spatial_benchmark_data_terms.md` for the governing
data terms and handling policy.
"""


def write_corpus_readme(root: Path) -> Path:
    """Write the deterministic, corpus-local v1 README without scene-derived facts."""

    path = root / README_NAME
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_CORPUS_README, encoding="utf-8")
    return path


@dataclass(frozen=True)
class SmokeGateReport:
    """Serializable smoke-gate result."""

    complete: bool
    covered_predicates: tuple[Predicate, ...]
    missing_predicates: tuple[Predicate, ...]
    passed_checks: tuple[str, ...]
    failed_checks: tuple[str, ...]
    failures: tuple[ValidationFailure, ...]


class SmokeGateError(RuntimeError):
    """Raised when pilot generation is blocked by the smoke gate."""


class PilotSourceError(RuntimeError):
    """Raised when the gated Structured3D source corpus is unavailable."""


def generate_smoke_corpus(root: Path) -> SmokeGateReport:
    """Write a disposable synthetic public/oracle smoke corpus and validate it."""

    if root.exists():
        shutil.rmtree(root)
    bundle = _smoke_bundle(root)
    write_bundle(root, bundle)
    write_corpus_readme(root)
    _neutralize_tiny_fixture_marker_order(root, bundle)
    report = validate_smoke_corpus(root)
    write_smoke_report(root, report)
    return report


def validate_smoke_corpus(root: Path) -> SmokeGateReport:
    """Run release validations plus smoke-specific coverage and Viser-load checks."""

    release_report = validate_release(root)
    failures = list(release_report.failures)
    covered = _covered_predicates(root)
    missing = tuple(predicate for predicate in Predicate if predicate not in covered)
    for predicate in missing:
        failures.append(
            ValidationFailure(
                "predicate-coverage", predicate.value, "smoke corpus has no retained sample"
            )
        )
    failures.extend(_validate_smoke_variants(root))
    failures.extend(_validate_viser_load(root))
    failed_checks = _smoke_failed_checks(failures)
    passed_checks = tuple(check for check in REQUIRED_SMOKE_CHECKS if check not in failed_checks)
    return SmokeGateReport(
        complete=not failures,
        covered_predicates=tuple(sorted(covered, key=lambda item: item.value)),
        missing_predicates=missing,
        passed_checks=passed_checks,
        failed_checks=failed_checks,
        failures=tuple(failures),
    )


def require_smoke_gate(smoke_root: Path) -> SmokeGateReport:
    """Block pilot generation unless the persisted or regenerated smoke report passes."""

    report = validate_smoke_corpus(smoke_root)
    write_smoke_report(smoke_root, report)
    if not report.complete:
        missing = ", ".join(predicate.value for predicate in report.missing_predicates) or "none"
        failed = ", ".join(report.failed_checks) or "none"
        raise SmokeGateError(
            f"smoke gate failed; missing predicates: {missing}; failed checks: {failed}"
        )
    return report


def run_pilot_generation(
    pilot_root: Path,
    smoke_root: Path,
    source_root: Path | None = None,
    *,
    workers: int | None = None,
) -> SmokeGateReport:
    """Generate the source-backed 30-scene pilot after the smoke gate passes."""

    report = require_smoke_gate(smoke_root)
    pilot_root.mkdir(parents=True, exist_ok=True)
    write_corpus_readme(pilot_root)
    write_smoke_report(pilot_root, report)
    if source_root is None:
        _write_blocked_pilot_report(
            pilot_root,
            "missing-source-root",
            "Pass --source-root pointing at the gated Structured3D release; synthetic fixtures are only permitted for smoke generation.",
        )
        raise PilotSourceError("pilot source root is required after the smoke gate passes")
    annotations = _structured3d_annotation_paths(source_root)
    if len(annotations) < _PILOT_SCENE_COUNT:
        _write_blocked_pilot_report(
            pilot_root,
            "insufficient-structured3d-scenes",
            f"Found {len(annotations)} Structured3D annotations under {source_root}; need at least {_PILOT_SCENE_COUNT} scene-disjoint source scenes for 10 development and 20 held-out scenes.",
        )
        raise PilotSourceError(
            f"Structured3D source unavailable or incomplete: found {len(annotations)} annotation files under {source_root}, need {_PILOT_SCENE_COUNT}"
        )
    result = _generate_source_backed_pilot(
        pilot_root, annotations, workers=_effective_workers(workers)
    )
    write_corpus_readme(pilot_root)
    write_smoke_report(pilot_root, report)
    if result.retained_count < _PILOT_SCENE_COUNT:
        _write_blocked_pilot_report(
            pilot_root,
            "insufficient-valid-structured3d-scenes",
            f"Retained {result.retained_count} valid scenes from {len(annotations)} annotations; need {_PILOT_SCENE_COUNT}. See rejected_scenes for importer/generator reasons.",
            rejected_scenes=result.rejected_scenes,
        )
        raise PilotSourceError(
            f"retained {result.retained_count} valid Structured3D scenes, need {_PILOT_SCENE_COUNT}"
        )
    return report


@dataclass(frozen=True)
class _PilotGenerationResult:
    retained_count: int
    rejected_scenes: tuple[dict[str, str], ...]


@dataclass(frozen=True)
class _PilotCandidate:
    source_scene_key: str
    scene_id: str
    imported: Structured3DImport
    trajectory: Trajectory
    questions: tuple[PhysicalQuestion, ...]

    @property
    def waypoint_count(self) -> int:
        return len(self.trajectory.waypoints)


@dataclass(frozen=True)
class _CandidateScanResult:
    candidate: _PilotCandidate | None
    rejected_scene: dict[str, str] | None


def _generate_source_backed_pilot(
    root: Path, annotations: tuple[Path, ...], *, workers: int
) -> _PilotGenerationResult:
    if root.exists():
        shutil.rmtree(root)
    root.mkdir(parents=True, exist_ok=True)
    candidates: list[_PilotCandidate] = []
    rejected: list[dict[str, str]] = []
    _progress(
        f"pilot candidate scan start annotations={len(annotations)} workers={workers} target_pool={_PILOT_CANDIDATE_POOL_SIZE}"
    )
    started = time.monotonic()
    if workers == 1:
        for scanned, annotation in enumerate(annotations, 1):
            result = _discover_pilot_candidate(annotation)
            _record_candidate_result(result, candidates, rejected)
            _progress_scan(scanned, len(annotations), len(candidates), len(rejected), started)
            if len(candidates) == _PILOT_CANDIDATE_POOL_SIZE:
                break
    else:
        with ProcessPoolExecutor(max_workers=workers) as executor:
            futures = {
                executor.submit(_discover_pilot_candidate, annotation): annotation
                for annotation in annotations
            }
            for scanned, future in enumerate(as_completed(futures), 1):
                result = future.result()
                _record_candidate_result(result, candidates, rejected)
                _progress_scan(scanned, len(annotations), len(candidates), len(rejected), started)
                if len(candidates) == _PILOT_CANDIDATE_POOL_SIZE:
                    for pending in futures:
                        pending.cancel()
                    break
    selected = tuple(
        sorted(candidates, key=lambda item: (item.waypoint_count, item.source_scene_key))[
            :_PILOT_SCENE_COUNT
        ]
    )
    _progress(
        f"pilot candidate scan done retained={len(candidates)} rejected={len(rejected)} selected={len(selected)} elapsed_s={time.monotonic() - started:.1f}"
    )
    retained: list[BundleInput] = []
    maps_root = root / ".pilot-maps"
    for index, candidate in enumerate(selected):
        split = Split.DEVELOPMENT if index < 10 else Split.HELD_OUT
        _progress(
            f"pilot map generation scene={index + 1}/{len(selected)} source={candidate.source_scene_key} split={split.value} waypoints={candidate.waypoint_count}"
        )
        variants = _pilot_snapshot_variants(
            maps_root,
            candidate.scene_id,
            candidate.trajectory.trajectory_id,
            candidate.trajectory,
            candidate.imported.free_space,
        )
        retained.append(
            BundleInput(
                _single_scene_manifest(candidate.scene_id, split),
                Scene(
                    scene_id=candidate.scene_id,
                    split=split,
                    trajectory_ids=(candidate.trajectory.trajectory_id,),
                ),
                candidate.trajectory,
                candidate.imported.source_provenance,
                candidate.imported.geometry,
                candidate.imported.topology,
                candidate.questions,
                variants,
            )
        )
    if len(retained) == _PILOT_SCENE_COUNT:
        for bundle in retained:
            write_bundle(root, bundle)
        _write_pilot_manifest(root, tuple(retained))
        validation_report = validate_release(root)
        write_validation_report(root, validation_report)
        validation_report.require_complete()
    return _PilotGenerationResult(len(retained), tuple(rejected))


def _discover_pilot_candidate(annotation: Path) -> _CandidateScanResult:
    source_key = annotation.parent.name
    scene_id = stable_opaque_id("scene", {"source": source_key})
    try:
        imported = load_structured3d_scene(
            annotation,
            scene_id=scene_id,
            source_scene_key=source_key,
            source_revision="Structured3D_annotation_3d.zip",
            source_to_benchmark=_source_axis_transform_for_annotation(annotation),
        )
        trajectory = generate_full_coverage_trajectory(scene_id, imported.free_space)
        oracle = SquareFootprintCollisionOracle(
            imported.geometry.floor_regions,
            imported.geometry.blocked_regions,
            SPATIAL_BENCHMARK_V1.footprint,
            SPATIAL_BENCHMARK_V1.geometry_tolerances,
            barriers=imported.geometry.barrier_segments,
        )
        questions = generate_physical_questions(
            scene_id=scene_id,
            trajectory_id=trajectory.trajectory_id,
            topology=imported.topology,
            oracle=oracle,
        )
    except (Structured3DError, ValueError, OSError) as error:
        return _CandidateScanResult(None, {"source_scene_key": source_key, "reason": str(error)})
    return _CandidateScanResult(
        _PilotCandidate(source_key, scene_id, imported, trajectory, questions), None
    )


def _record_candidate_result(
    result: _CandidateScanResult,
    candidates: list[_PilotCandidate],
    rejected: list[dict[str, str]],
) -> None:
    if result.candidate is not None:
        candidates.append(result.candidate)
        _progress(
            f"pilot candidate retained source={result.candidate.source_scene_key} waypoints={result.candidate.waypoint_count} questions={len(result.candidate.questions)}"
        )
    elif result.rejected_scene is not None:
        rejected.append(result.rejected_scene)


def _progress_scan(scanned: int, total: int, retained: int, rejected: int, started: float) -> None:
    if scanned == 1 or scanned % 10 == 0 or retained >= _PILOT_CANDIDATE_POOL_SIZE:
        _progress(
            f"pilot candidate scan progress scanned={scanned}/{total} retained={retained} rejected={rejected} elapsed_s={time.monotonic() - started:.1f}"
        )


def _progress(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def _effective_workers(workers: int | None) -> int:
    if workers is not None:
        return max(1, workers)
    cpu_count = os.cpu_count() or 2
    return max(1, min(4, cpu_count - 1))


def _pilot_snapshot_variants(
    maps_root: Path,
    scene_id: str,
    trajectory_id: str,
    trajectory: Trajectory,
    free_space: FreeSpaceModel,
) -> tuple[SnapshotVariant, ...]:
    variants: list[SnapshotVariant] = []
    for profile in SPATIAL_BENCHMARK_V1.lidar_profiles:
        variant_started = time.monotonic()
        _progress(f"pilot map variant start scene={scene_id} variant={profile.name.value}")
        seed = derive_v1_seed(profile.name, scene_id, trajectory_id)
        generated = generate_map(trajectory, free_space, profile, seed)
        directory = maps_root / scene_id / profile.name.value
        snapshot = write_snapshot(
            generated,
            directory,
            scene_id=scene_id,
            trajectory_id=trajectory_id,
            mapper_revision="dimos-voxel-grid-mapper-v1",
        )
        variants.append(SnapshotVariant(snapshot, directory, generated.alignment))
        _progress(
            f"pilot map variant done scene={scene_id} variant={profile.name.value} elapsed_s={time.monotonic() - variant_started:.1f}"
        )
    return tuple(variants)


def _single_scene_manifest(scene_id: str, split: Split) -> Manifest:
    return Manifest(
        release_id=stable_opaque_id("release", {"pilot": "v1"}),
        release_version="v1.0.0",
        generator_revision="source-backed-pilot-v1",
        mapper_configuration_digest=hashlib.sha256(
            canonical_json(SPATIAL_BENCHMARK_V1.model_dump(mode="json"))
        ).hexdigest(),
        source_dataset_revision="Structured3D_annotation_3d.zip",
        scenes=(
            ManifestScene(
                scene_id=scene_id,
                split=split,
                scene_path=f"public/scenes/{scene_id}/scene.json",
            ),
        ),
    )


def _write_pilot_manifest(root: Path, bundles: tuple[BundleInput, ...]) -> None:
    scenes = tuple(
        ManifestScene(
            scene_id=bundle.scene.scene_id,
            split=bundle.scene.split,
            scene_path=f"public/scenes/{bundle.scene.scene_id}/scene.json",
        )
        for bundle in bundles
    )
    manifest = _single_scene_manifest(bundles[0].scene.scene_id, bundles[0].scene.split).model_copy(
        update={"scenes": scenes}
    )
    (root / "manifest.json").write_bytes(canonical_json(manifest.model_dump(mode="json")) + b"\n")


def _structured3d_annotation_paths(source_root: Path) -> tuple[Path, ...]:
    if not source_root.is_dir():
        return ()
    return tuple(sorted(source_root.rglob("annotation_3d.json")))


def _source_axis_transform_for_annotation(annotation: Path) -> SourceAxisTransform:
    """Use Structured3D millimetres by default, but accept metre-scale fixtures."""

    try:
        payload = json.loads(annotation.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return SourceAxisTransform()
    junctions = payload.get("junctions")
    if not isinstance(junctions, list):
        return SourceAxisTransform()
    max_abs = 0.0
    for junction in junctions:
        if not isinstance(junction, dict):
            continue
        coordinate = junction.get("coordinate")
        if not isinstance(coordinate, list):
            continue
        for value in coordinate:
            if isinstance(value, (int, float)):
                max_abs = max(max_abs, abs(float(value)))
    if 0.0 < max_abs <= 100.0:
        return SourceAxisTransform(scale_m_per_source_unit=1.0)
    return SourceAxisTransform()


def _write_blocked_pilot_report(
    root: Path, code: str, message: str, rejected_scenes: tuple[dict[str, str], ...] = ()
) -> Path:
    rejected_payload: list[JsonValue] = [dict(item) for item in rejected_scenes]
    payload: JsonValue = {
        "complete": False,
        "blocked": True,
        "blocker_code": code,
        "message": message,
        "target_scene_count": _PILOT_SCENE_COUNT,
        "development_scene_count": 10,
        "held_out_scene_count": 20,
        "expected_instance_count_approx": 1170,
        "review_override_policy": "isolated manual exclusions/corrections only in oracle review_overrides.jsonl; recurring defects require generator-policy fix and regeneration",
        "rejected_scenes": rejected_payload,
    }
    path = root / "pilot_generation_report.json"
    path.write_bytes(canonical_json(payload) + b"\n")
    return path


def write_smoke_report(root: Path, report: SmokeGateReport) -> Path:
    path = root / SMOKE_REPORT_NAME
    path.parent.mkdir(parents=True, exist_ok=True)
    payload: JsonValue = {
        "complete": report.complete,
        "covered_predicates": [predicate.value for predicate in report.covered_predicates],
        "missing_predicates": [predicate.value for predicate in report.missing_predicates],
        "passed_checks": list(report.passed_checks),
        "failed_checks": list(report.failed_checks),
        "failures": [failure.__dict__ for failure in report.failures],
    }
    path.write_bytes(canonical_json(payload) + b"\n")
    return path


def _smoke_bundle(root: Path) -> BundleInput:
    scene_id = stable_opaque_id("scene", {"smoke": "development"})
    trajectory_id = stable_opaque_id("trajectory", {"smoke": "development"})
    rooms = tuple(
        Room(room_id=stable_opaque_id("room", {"index": index}), boundary=_square(index * 4.0))
        for index in range(3)
    )
    topology = Topology(
        scene_id=scene_id,
        rooms=rooms,
        direct_openings=(
            OpeningEdge(
                opening_id=stable_opaque_id("opening", {"index": 0}),
                first_room_id=rooms[0].room_id,
                second_room_id=rooms[1].room_id,
            ),
        ),
    )
    oracle = SquareFootprintCollisionOracle(
        floor_regions=tuple(room.boundary for room in rooms),
        blocked_regions=(),
        footprint=SPATIAL_BENCHMARK_V1.footprint,
        tolerances=SPATIAL_BENCHMARK_V1.geometry_tolerances,
    )
    questions = generate_physical_questions(
        scene_id=scene_id, trajectory_id=trajectory_id, topology=topology, oracle=oracle
    )
    trajectory = Trajectory(
        trajectory_id=trajectory_id,
        scene_id=scene_id,
        policy_version="smoke-coverage-v1",
        frame_id="world",
        waypoints=(Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),),
    )
    manifest = Manifest(
        release_id=stable_opaque_id("release", {"smoke": "v1"}),
        release_version="v1.0.0",
        generator_revision="smoke-synthetic-v1",
        mapper_configuration_digest="0" * 64,
        source_dataset_revision="synthetic-development-scene-v1",
        scenes=(
            ManifestScene(
                scene_id=scene_id,
                split=Split.DEVELOPMENT,
                scene_path=f"public/scenes/{scene_id}/scene.json",
            ),
        ),
    )
    scene = Scene(scene_id=scene_id, split=Split.DEVELOPMENT, trajectory_ids=(trajectory_id,))
    source = SourceProvenance(
        scene_id=scene_id,
        source_dataset="synthetic-development-scene",
        source_scene_key="smoke/minimal",
        source_revision="v1",
        source_artifact_sha256="1" * 64,
        coordinate_frame_description="metric right-handed xy floor, +z gravity",
    )
    geometry = Geometry(
        scene_id=scene_id,
        floor_regions=tuple(room.boundary for room in rooms),
        blocked_regions=(),
        openings=(),
    )
    return BundleInput(
        manifest,
        scene,
        trajectory,
        source,
        geometry,
        topology,
        questions,
        _snapshot_variants(root, scene_id, trajectory_id, trajectory),
    )


def _snapshot_variants(
    root: Path, scene_id: str, trajectory_id: str, trajectory: Trajectory
) -> tuple[SnapshotVariant, ...]:
    alignment = VariantAlignment(
        (PoseAlignment(0, trajectory.waypoints[0], trajectory.waypoints[0], True),)
    )
    variants: list[SnapshotVariant] = []
    for index, variant in enumerate(MapVariant):
        directory = root / ".smoke-maps" / variant.value
        directory.mkdir(parents=True, exist_ok=True)
        artifact = directory / "global_map.pc2.lcm"
        points = np.array(
            ((0.0, 0.0, 0.0), (1.0 + index, 0.0, 0.0), (0.0, 1.0 + index, 0.0)), dtype=np.float32
        )
        artifact.write_bytes(
            PointCloud2.from_numpy(points, frame_id="world", timestamp=1.0).lcm_encode(
                frame_id="world"
            )
        )
        snapshot = Snapshot(
            snapshot_id=stable_opaque_id("snapshot", {"smoke": variant.value}),
            scene_id=scene_id,
            trajectory_id=trajectory_id,
            variant=variant,
            terminal_pose=trajectory.waypoints[0],
            map_artifact_path="global_map.pc2.lcm",
            map_artifact_sha256=hash_file_sha256(artifact),
            mapper_revision="smoke-synthetic",
            mapper_configuration_digest="0" * 64,
            mapper_configuration=MapperConfigurationRecord(
                voxel_size_m=0.05, block_count=1, frame_id="world", emit_every=1
            ),
            noise_profile_version=f"smoke-{variant.value}",
            seed=index + 1,
            frame_id="world",
            frame_contract=FrameConventionRecord(frame_id="world"),
        )
        variants.append(SnapshotVariant(snapshot, directory, alignment))
    return tuple(variants)


def _square(left: float) -> Polygon2D:
    return Polygon2D(
        vertices=(
            Point2D(x_m=left, y_m=0.0),
            Point2D(x_m=left + 4.0, y_m=0.0),
            Point2D(x_m=left + 4.0, y_m=4.0),
            Point2D(x_m=left, y_m=4.0),
        )
    )


def _covered_predicates(root: Path) -> set[Predicate]:
    covered: set[Predicate] = set()
    for path in root.glob("public/scenes/*/trajectories/*/questions.jsonl"):
        for line in path.read_text(encoding="utf-8").splitlines():
            if line:
                covered.add(Question.model_validate_json(line).predicate)
    return covered


def _validate_smoke_variants(root: Path) -> tuple[ValidationFailure, ...]:
    failures: list[ValidationFailure] = []
    try:
        loader = SpatialCorpusLoader(root)
    except (KeyError, ValueError, OSError, json.JSONDecodeError) as error:
        return (
            ValidationFailure(
                "predicate-coverage", str(root), f"cannot load smoke corpus: {error}"
            ),
        )
    for predicate in Predicate:
        try:
            instances = loader.instances(SpatialCorpusSelection(predicate=predicate))
        except (KeyError, ValueError, OSError, json.JSONDecodeError) as error:
            failures.append(ValidationFailure("predicate-coverage", predicate.value, str(error)))
            continue
        variants = {instance.instance.variant for instance in instances}
        if variants != set(MapVariant):
            failures.append(
                ValidationFailure(
                    "predicate-coverage", predicate.value, "missing clean/noisy smoke instances"
                )
            )
    return tuple(failures)


def _validate_viser_load(root: Path) -> tuple[ValidationFailure, ...]:
    failures: list[ValidationFailure] = []
    try:
        loader = SpatialCorpusLoader(root)
    except (KeyError, ValueError, OSError, json.JSONDecodeError) as error:
        return (ValidationFailure("viser-load", str(root), f"cannot load smoke corpus: {error}"),)
    view = SpatialCorpusViserView(loader, ViserReadOnlyBoundary())
    for predicate in Predicate:
        for variant in MapVariant:
            try:
                instance = loader.require_one(
                    SpatialCorpusSelection(predicate=predicate, variant=variant)
                )
                view.render(instance, show_oracle_geometry=True, show_oracle_topology=True)
            except (KeyError, ValueError, OSError, json.JSONDecodeError) as error:
                failures.append(
                    ValidationFailure(
                        "viser-load", f"{predicate.value}/{variant.value}", str(error)
                    )
                )
    return tuple(failures)


def _smoke_failed_checks(failures: list[ValidationFailure]) -> tuple[str, ...]:
    checks = set[str]()
    for failure in failures:
        checks.add(_SMOKE_CHECK_MAP.get(failure.check, failure.check))
    return tuple(sorted(checks))


_SMOKE_CHECK_MAP = {
    "artifact-hash": "artifact-decode-hash",
    "hash": "artifact-decode-hash",
    "pointcloud-decode": "artifact-decode-hash",
    "pointcloud2-decode": "artifact-decode-hash",
    "frame-contract": "coordinate",
    "question-answer-pairing": "oracle",
    "paired-variants": "pairing",
    "room-graph": "topology",
    "collision-oracle": "collision",
    "public-leakage": "leakage",
    "schema": "schema",
    "predicate-coverage": "pairing",
}


def _neutralize_tiny_fixture_marker_order(root: Path, bundle: BundleInput) -> None:
    path = (
        root
        / "oracle"
        / "scenes"
        / bundle.scene.scene_id
        / "trajectories"
        / bundle.trajectory.trajectory_id
        / "question_geometry.jsonl"
    )
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    for record in records:
        if len(record["markers"]) == 2:
            record["markers"] = list(reversed(record["markers"]))
            break
    path.write_bytes(b"".join(canonical_json(record) + b"\n" for record in records))
