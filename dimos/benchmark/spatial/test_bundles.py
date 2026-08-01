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

"""Bundle writer regressions for the spatial corpus hierarchy and refs."""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import pytest

from dimos.benchmark.spatial.bundles import BundleInput, SnapshotVariant, write_bundle
from dimos.benchmark.spatial.collision_oracle import SquareFootprintCollisionOracle
from dimos.benchmark.spatial.config import SPATIAL_BENCHMARK_V1
from dimos.benchmark.spatial.map_generation import PoseAlignment, VariantAlignment
from dimos.benchmark.spatial.models import (
    FrameConventionRecord,
    Geometry,
    Manifest,
    ManifestScene,
    MapperConfigurationRecord,
    MapVariant,
    OpeningEdge,
    Point2D,
    Polygon2D,
    Pose2D,
    Room,
    Scene,
    Snapshot,
    SourceProvenance,
    Split,
    Topology,
    Trajectory,
)
from dimos.benchmark.spatial.questions import generate_physical_questions
from dimos.benchmark.spatial.utilities import canonical_json, hash_file_sha256, stable_opaque_id


def _id(namespace: str) -> str:
    return stable_opaque_id(namespace, {"test": namespace})


def _square(left: float) -> Polygon2D:
    return Polygon2D(
        vertices=(
            Point2D(x_m=left, y_m=0.0),
            Point2D(x_m=left + 4.0, y_m=0.0),
            Point2D(x_m=left + 4.0, y_m=4.0),
            Point2D(x_m=left, y_m=4.0),
        )
    )


def _bundle(tmp_path: Path) -> BundleInput:
    scene_id, trajectory_id = _id("scene"), _id("trajectory")
    rooms = tuple(
        Room(room_id=_id(f"room-{index}"), boundary=_square(index * 4.0)) for index in range(3)
    )
    topology = Topology(
        scene_id=scene_id,
        rooms=rooms,
        direct_openings=(
            OpeningEdge(
                opening_id=_id("opening"),
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
        policy_version="test-coverage-v1",
        frame_id="world",
        waypoints=(Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),),
    )
    manifest = Manifest(
        release_id=_id("release"),
        release_version="v1.0.0",
        generator_revision="test",
        mapper_configuration_digest="0" * 64,
        source_dataset_revision="test",
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
        source_dataset="structured3d",
        source_scene_key="private/source/key",
        source_revision="test",
        source_artifact_sha256="1" * 64,
        coordinate_frame_description="test",
    )
    geometry = Geometry(
        scene_id=scene_id,
        floor_regions=tuple(room.boundary for room in rooms),
        blocked_regions=(),
        openings=(),
    )
    alignment = VariantAlignment(
        (PoseAlignment(0, trajectory.waypoints[0], trajectory.waypoints[0], True),)
    )
    variants = []
    for variant in MapVariant:
        directory = tmp_path / "maps" / variant.value
        directory.mkdir(parents=True)
        (directory / "global_map.pc2.lcm").write_bytes(f"map-{variant.value}".encode())
        snapshot = Snapshot(
            snapshot_id=stable_opaque_id("snapshot", {"variant": variant.value}),
            scene_id=scene_id,
            trajectory_id=trajectory_id,
            variant=variant,
            terminal_pose=trajectory.waypoints[0],
            map_artifact_path="global_map.pc2.lcm",
            map_artifact_sha256=hash_file_sha256(directory / "global_map.pc2.lcm"),
            mapper_revision="test",
            mapper_configuration_digest="0" * 64,
            mapper_configuration=MapperConfigurationRecord(
                voxel_size_m=0.05, block_count=1, frame_id="world", emit_every=1
            ),
            noise_profile_version="test",
            seed=1,
            frame_id="world",
            frame_contract=FrameConventionRecord(frame_id="world"),
        )
        variants.append(SnapshotVariant(snapshot, directory, alignment))
    return BundleInput(
        manifest, scene, trajectory, source, geometry, topology, questions, tuple(variants)
    )


def test_bundle_write_hierarchy_refs_hashes_and_leakage_scan(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    result = write_bundle(tmp_path / "corpus", bundle)

    assert result.question_count == 13
    assert result.instance_count == 39
    assert (tmp_path / "corpus" / "manifest.json").exists()
    assert (tmp_path / "corpus" / "schemas").is_dir()
    assert not (result.public_root / "manifest.json").exists()
    assert not (result.public_root / "schemas").exists()

    questions_path = (
        result.public_root
        / "scenes"
        / bundle.scene.scene_id
        / "trajectories"
        / bundle.trajectory.trajectory_id
        / "questions.jsonl"
    )
    question_records = [json.loads(line) for line in questions_path.read_text().splitlines()]
    assert len({record["text"] for record in question_records}) == len(question_records)
    assert canonical_json(question_records[0]) == canonical_json(
        json.loads(canonical_json(question_records[0]))
    )

    for variant in bundle.variants:
        variant_root = questions_path.parent / "variants" / variant.snapshot.variant.value
        assert hash_file_sha256(variant.directory / "global_map.pc2.lcm") == hash_file_sha256(
            variant_root / "global_map.pc2.lcm"
        )
        instances = [
            json.loads(line) for line in (variant_root / "instances.jsonl").read_text().splitlines()
        ]
        assert {item["question_id"] for item in instances} == {
            question.question.question_id for question in bundle.questions
        }
        assert all(item["snapshot_id"] == variant.snapshot.snapshot_id for item in instances)


def test_question_stable_id_validation_does_not_use_answer_value(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    target = next(item for item in bundle.questions if item.question.predicate.value == "same-room")
    flipped = target.answer.model_copy(
        update={
            "value": target.answer.value.model_copy(update={"value": not target.answer.value.value})
        }
    )
    questions = tuple(
        replace(item, answer=flipped) if item is target else item for item in bundle.questions
    )

    result = write_bundle(
        tmp_path / "answer-independent",
        BundleInput(
            bundle.manifest,
            bundle.scene,
            bundle.trajectory,
            bundle.source,
            bundle.geometry,
            bundle.topology,
            questions,
            bundle.variants,
        ),
    )

    assert result.question_count == 13


def test_bundle_leakage_rejects_private_keys_without_query_geometry_false_positive(
    tmp_path: Path,
) -> None:
    bundle = _bundle(tmp_path)
    write_bundle(tmp_path / "clean", bundle)
    bad_source = bundle.source.model_copy(update={"source_scene_key": "query_geometry"})
    query_geometry_source_bundle = BundleInput(
        bundle.manifest,
        bundle.scene,
        bundle.trajectory,
        bad_source,
        bundle.geometry,
        bundle.topology,
        bundle.questions,
        bundle.variants,
    )
    write_bundle(tmp_path / "false-positive", query_geometry_source_bundle)

    bad_manifest = bundle.manifest.model_copy(
        update={
            "scenes": (
                ManifestScene(
                    scene_id=bundle.scene.scene_id,
                    split=Split.DEVELOPMENT,
                    scene_path=f"oracle/scenes/{bundle.scene.scene_id}/scene.json",
                ),
            )
        }
    )
    with pytest.raises(ValueError, match="manifest scene_path"):
        write_bundle(
            tmp_path / "bad",
            BundleInput(
                bad_manifest,
                bundle.scene,
                bundle.trajectory,
                bundle.source,
                bundle.geometry,
                bundle.topology,
                bundle.questions,
                bundle.variants,
            ),
        )
