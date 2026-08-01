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

"""Read-only spatial corpus loader and Viser boundary tests."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np

from dimos.benchmark.spatial.bundles import BundleInput, SnapshotVariant, write_bundle
import dimos.benchmark.spatial.cli as spatial_cli
from dimos.benchmark.spatial.cli import main
from dimos.benchmark.spatial.corpus_loader import SpatialCorpusLoader, SpatialCorpusSelection
from dimos.benchmark.spatial.models import (
    BarrierSegment,
    MapVariant,
    Point2D,
    Polygon2D,
    Predicate,
)
from dimos.benchmark.spatial.test_bundles import _bundle
from dimos.benchmark.spatial.utilities import hash_file_sha256
from dimos.benchmark.spatial.viewer import (
    DERIVED_QUERY_CLEARANCE_M,
    DERIVED_THRESHOLD_DEPTH_M,
    DERIVED_THRESHOLD_HEIGHT_M,
    DERIVED_TOPOLOGY_CLEARANCE_M,
    DERIVED_WALL_HEIGHT_M,
    DERIVED_WALL_OPACITY,
    DERIVED_WALL_WIDTH_M,
    OBSERVED_MAP_COLOR,
    ORACLE_OPENING_COLOR,
    ORACLE_TOPOLOGY_COLOR,
    ORACLE_WALL_COLOR,
    QUERY_COLOR,
    DrawCommand,
    SpatialCorpusViserView,
    SpatialQASelector,
    ViserReadOnlyBoundary,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _valid_bundle(tmp_path: Path) -> BundleInput:
    bundle = _bundle(tmp_path)
    variants: list[SnapshotVariant] = []
    for index, variant in enumerate(bundle.variants):
        artifact = variant.directory / "global_map.pc2.lcm"
        points = np.array(
            ((0.0, 0.0, 0.0), (1.0 + index, 0.0, 0.0), (0.0, 1.0 + index, 0.0)),
            dtype=np.float32,
        )
        cloud = PointCloud2.from_numpy(points, frame_id=variant.snapshot.frame_id, timestamp=1.0)
        artifact.write_bytes(cloud.lcm_encode(frame_id=variant.snapshot.frame_id))
        snapshot = variant.snapshot.model_copy(
            update={"map_artifact_sha256": hash_file_sha256(artifact)}
        )
        variants.append(SnapshotVariant(snapshot, variant.directory, variant.alignment))
    return BundleInput(
        bundle.manifest,
        bundle.scene,
        bundle.trajectory,
        bundle.source,
        bundle.geometry,
        bundle.topology,
        bundle.questions,
        tuple(variants),
    )


def test_loader_selects_scene_question_variant_and_joins_oracle(tmp_path: Path) -> None:
    bundle = _valid_bundle(tmp_path)
    write_bundle(tmp_path / "corpus", bundle)
    loader = SpatialCorpusLoader(tmp_path / "corpus")

    loaded = loader.require_one(
        SpatialCorpusSelection(
            scene_id=bundle.scene.scene_id,
            trajectory_id=bundle.trajectory.trajectory_id,
            predicate=Predicate.SAME_ROOM,
            variant=MapVariant.CLEAN,
        )
    )

    assert loaded.scene.scene_id == bundle.scene.scene_id
    assert loaded.question.predicate is Predicate.SAME_ROOM
    assert loaded.snapshot.variant is MapVariant.CLEAN
    assert loaded.oracle is not None
    assert loaded.oracle.geometry.scene_id == bundle.scene.scene_id


def test_viewer_opens_every_predicate_and_variant_without_writes(tmp_path: Path) -> None:
    bundle = _valid_bundle(tmp_path)
    write_bundle(tmp_path / "corpus", bundle)
    corpus_files = sorted(path for path in (tmp_path / "corpus").rglob("*") if path.is_file())
    before = {path: path.stat().st_mtime_ns for path in corpus_files}
    loader = SpatialCorpusLoader(tmp_path / "corpus")
    view = SpatialCorpusViserView(loader, ViserReadOnlyBoundary())

    for predicate in Predicate:
        for variant in MapVariant:
            instance = loader.require_one(
                SpatialCorpusSelection(predicate=predicate, variant=variant)
            )
            view.render(instance)
            command_kinds = {command.kind for command in view.boundary.commands}
            command_names = {command.name for command in view.boundary.commands}
            assert "point-cloud" in command_kinds
            assert "inspector-section" in command_kinds
            question_section = next(
                command
                for command in view.boundary.commands
                if command.name == "/inspector/question"
            )
            assert _inspector_rows(question_section)["Question"] == instance.question.text
            assert "/agent-visible/observed-map" in command_names
            _assert_predicate_overlay(predicate, command_names)
            assert not any(name.startswith("/private-oracle/relief/") for name in command_names)
            assert not any(
                name.startswith("/private-oracle/walls/blocked/") for name in command_names
            )
            if predicate in {
                Predicate.POSE_OCCUPANCY,
                Predicate.STRAIGHT_TRANSLATION,
                Predicate.IN_PLACE_ROTATION,
                Predicate.ELIGIBLE_ROOM_COUNT,
            }:
                assert not any(
                    name.startswith("/private-oracle/topology/") for name in command_names
                )

    after = {path: path.stat().st_mtime_ns for path in corpus_files}
    assert after == before


def test_viewer_navigation_exposes_previous_next_and_variant(tmp_path: Path) -> None:
    bundle = _valid_bundle(tmp_path)
    write_bundle(tmp_path / "corpus", bundle)
    loader = SpatialCorpusLoader(tmp_path / "corpus")
    view = SpatialCorpusViserView(loader, ViserReadOnlyBoundary())
    current = loader.require_one(SpatialCorpusSelection(variant=MapVariant.CLEAN))

    assert view.next_instance(current).instance.instance_id != current.instance.instance_id
    assert (
        view.previous_instance(view.next_instance(current)).instance.instance_id
        == current.instance.instance_id
    )
    assert view.variant(current, "noisy-01").snapshot.variant is MapVariant.NOISY_01


def test_viewer_uses_semantic_evidence_groups_colors_and_briefing(tmp_path: Path) -> None:
    bundle = _valid_bundle(tmp_path)
    private_polygon = Polygon2D(
        vertices=(
            Point2D(x_m=0.0, y_m=0.0),
            Point2D(x_m=0.5, y_m=0.0),
            Point2D(x_m=0.5, y_m=0.5),
        )
    )
    bundle = replace(
        bundle,
        geometry=bundle.geometry.model_copy(
            update={
                "blocked_regions": (private_polygon,),
                "barrier_segments": (
                    BarrierSegment(
                        start=Point2D(x_m=1.0, y_m=0.0),
                        end=Point2D(x_m=1.0, y_m=0.5),
                    ),
                ),
                "openings": (private_polygon,),
            }
        ),
    )
    write_bundle(tmp_path / "corpus", bundle)
    loader = SpatialCorpusLoader(tmp_path / "corpus")
    view = SpatialCorpusViserView(loader, ViserReadOnlyBoundary())

    instance = loader.require_one(
        SpatialCorpusSelection(predicate=Predicate.SAME_ROOM, variant=MapVariant.CLEAN)
    )
    view.render(instance, show_oracle_geometry=True)

    commands = {command.name: command for command in view.boundary.commands}
    question_section = commands["/inspector/question"]
    answer_section = commands["/inspector/private-answer"]
    evidence_section = commands["/inspector/evidence-key"]
    relief_section = commands["/inspector/private-relief"]
    assert instance.oracle is not None
    answer = next(
        candidate
        for candidate in instance.oracle.answers
        if candidate.question_id == instance.question.question_id
    )
    answer_value = answer.value.value
    expected_answer = (
        "Yes" if answer_value is True else "No" if answer_value is False else str(answer_value)
    )

    assert question_section.kind == "inspector-section"
    assert question_section.text == "Question"
    assert question_section.group == "agent-visible"
    assert _inspector_rows(question_section) == {
        "Question": instance.question.text,
        "Predicate": instance.question.predicate.value,
        "Map variant": instance.instance.variant.value,
    }
    assert answer_section.text == "Private answer"
    assert answer_section.group == "private-oracle"
    assert _inspector_rows(answer_section)["Oracle truth"] == expected_answer
    assert evidence_section.text == "Evidence key"
    assert (
        _inspector_rows(evidence_section)["Public"] == "Gray: observed scan · Magenta: active query"
    )
    assert "Muted red: walls" in _inspector_rows(evidence_section)["Private"]
    assert relief_section.text == "Private relief"
    assert _inspector_rows(relief_section)["Source"] == "Derived from private 2-D oracle geometry."
    assert _inspector_rows(relief_section)["Walls and openings"] == "Shown in this review."
    assert expected_answer not in " ".join(_inspector_rows(question_section).values())
    assert expected_answer not in " ".join(_inspector_rows(evidence_section).values())

    observed_map = commands["/agent-visible/observed-map"]
    assert observed_map.color == OBSERVED_MAP_COLOR
    assert observed_map.group == "agent-visible"
    presentation_base_z_m = min(point[2] for point in observed_map.points)
    query_markers = commands["/agent-visible/query/markers"]
    assert query_markers.color == QUERY_COLOR
    assert all(
        point[2] == presentation_base_z_m + DERIVED_WALL_HEIGHT_M + DERIVED_QUERY_CLEARANCE_M
        for point in query_markers.points
    )
    wall_relief = commands["/private-oracle/relief/walls/barrier/0"]
    assert wall_relief.kind == "box"
    assert wall_relief.color == ORACLE_WALL_COLOR
    assert wall_relief.dimensions_m == (0.5, DERIVED_WALL_WIDTH_M, DERIVED_WALL_HEIGHT_M)
    assert wall_relief.opacity == DERIVED_WALL_OPACITY
    assert wall_relief.base_z_m == presentation_base_z_m
    assert wall_relief.points[0][2] - DERIVED_WALL_HEIGHT_M / 2.0 == presentation_base_z_m
    assert wall_relief.material == "standard"
    assert wall_relief.derived
    assert wall_relief.source == "private-barrier-segment"
    assert wall_relief.presentation == "derived-private-architectural-relief"
    threshold_relief = commands["/private-oracle/relief/openings/threshold/0"]
    assert threshold_relief.kind == "box"
    assert threshold_relief.color == ORACLE_OPENING_COLOR
    assert threshold_relief.dimensions_m is not None
    assert threshold_relief.dimensions_m[1:] == (
        DERIVED_THRESHOLD_DEPTH_M,
        DERIVED_THRESHOLD_HEIGHT_M,
    )
    assert threshold_relief.base_z_m == presentation_base_z_m
    assert threshold_relief.points[0][2] - DERIVED_THRESHOLD_HEIGHT_M / 2.0 == presentation_base_z_m
    assert DERIVED_THRESHOLD_DEPTH_M < DERIVED_WALL_WIDTH_M
    assert DERIVED_THRESHOLD_HEIGHT_M < DERIVED_WALL_HEIGHT_M
    assert threshold_relief.source == "private-opening-polygon"
    assert threshold_relief.presentation == "derived-private-architectural-relief"
    assert not any(
        command.kind == "polyline" and command.color == ORACLE_WALL_COLOR
        for command in commands.values()
    )
    topology_commands = [
        command
        for command in commands.values()
        if command.name.startswith("/private-oracle/topology/")
    ]
    assert all(command.color == ORACLE_TOPOLOGY_COLOR for command in topology_commands)
    assert all(
        point[2] == presentation_base_z_m + DERIVED_WALL_HEIGHT_M + DERIVED_TOPOLOGY_CLEARANCE_M
        for command in topology_commands
        for point in command.points
    )
    assert len(topology_commands) < len(bundle.topology.rooms) + len(
        bundle.topology.direct_openings
    )
    assert commands["/agent-visible/query/markers/A"].text == "A"
    assert commands["/agent-visible/query/markers/B"].text == "B"
    assert not any(command.kind == "camera" for command in commands.values())
    assert not any("/private-oracle/floor/" in name for name in commands)
    assert not any(name.startswith("/private-oracle/walls/") for name in commands)
    assert not any("/agent-visible/robot/" in name for name in commands)
    assert not any(command.kind == "dashboard" for command in commands.values())


def test_briefing_displays_matching_integer_private_answer(tmp_path: Path) -> None:
    bundle = _valid_bundle(tmp_path)
    root = tmp_path / "corpus"
    write_bundle(root, bundle)
    loader = SpatialCorpusLoader(root)
    view = SpatialCorpusViserView(loader, ViserReadOnlyBoundary())
    instance = loader.require_one(
        SpatialCorpusSelection(predicate=Predicate.ELIGIBLE_ROOM_COUNT, variant=MapVariant.CLEAN)
    )

    view.render(instance)

    assert instance.oracle is not None
    answer = next(
        candidate
        for candidate in instance.oracle.answers
        if candidate.question_id == instance.question.question_id
    )
    assert answer.value.kind == "integer"
    question_section = next(
        command for command in view.boundary.commands if command.name == "/inspector/question"
    )
    answer_section = next(
        command for command in view.boundary.commands if command.name == "/inspector/private-answer"
    )
    assert _inspector_rows(question_section)["Question"] == instance.question.text
    assert _inspector_rows(answer_section)["Oracle truth"] == str(answer.value.value)


def test_qa_selector_cascades_predicate_sample_and_variant_renders_evidence(tmp_path: Path) -> None:
    bundle = _valid_bundle(tmp_path)
    root = tmp_path / "corpus"
    write_bundle(root, bundle)
    loader = SpatialCorpusLoader(root)
    boundary = ViserReadOnlyBoundary()
    view = SpatialCorpusViserView(loader, boundary)

    initial = view.start_qa_review()

    assert initial.snapshot.variant is MapVariant.CLEAN
    assert initial.scene.split.value == "development"
    assert boundary.qa_selector is not None
    assert boundary.qa_selector.sample_labels[0].startswith("Development 01 · ")
    assert boundary.on_qa_selection is not None
    assert not any(
        command.name.startswith("/private-oracle/relief/") for command in boundary.commands
    )

    changed = boundary.qa_selector.select_predicate_label("Same Room")
    boundary.on_qa_selection(changed)

    assert changed.question.predicate is Predicate.SAME_ROOM
    assert changed.instance.instance_id != initial.instance.instance_id
    assert "/agent-visible/observed-map" in {command.name for command in boundary.commands}
    assert "/agent-visible/query/markers" in {command.name for command in boundary.commands}
    assert not any(
        command.name.startswith("/private-oracle/relief/") for command in boundary.commands
    )

    prior_map = next(
        command for command in boundary.commands if command.name == "/agent-visible/observed-map"
    )
    paired = boundary.qa_selector.select_variant(MapVariant.NOISY_01)
    boundary.on_qa_selection(paired)
    noisy_map = next(
        command for command in boundary.commands if command.name == "/agent-visible/observed-map"
    )

    assert paired.question.question_id == changed.question.question_id
    assert paired.snapshot.variant is MapVariant.NOISY_01
    assert paired.instance.instance_id != changed.instance.instance_id
    assert noisy_map.points != prior_map.points


def test_viewer_without_oracle_is_strictly_public_only(tmp_path: Path) -> None:
    bundle = _valid_bundle(tmp_path)
    root = tmp_path / "corpus"
    write_bundle(root, bundle)
    loader = SpatialCorpusLoader(root, oracle_root=root / "__oracle_disabled__")
    view = SpatialCorpusViserView(loader, ViserReadOnlyBoundary())

    view.render(loader.require_one())

    assert not any(command.group == "private-oracle" for command in view.boundary.commands)
    assert not any(command.derived for command in view.boundary.commands)
    answer_section = next(
        command for command in view.boundary.commands if command.name == "/inspector/private-answer"
    )
    question_section = next(
        command for command in view.boundary.commands if command.name == "/inspector/question"
    )
    evidence_section = next(
        command for command in view.boundary.commands if command.name == "/inspector/evidence-key"
    )
    assert answer_section.group == "reviewer-status"
    assert (
        _inspector_rows(answer_section)["Oracle truth"] == "Unavailable — no private oracle loaded."
    )
    assert "Unavailable" not in " ".join(_inspector_rows(question_section).values())
    assert "Unavailable" not in " ".join(_inspector_rows(evidence_section).values())


def test_selector_defaults_to_clean_but_honors_cli_variant_initialization(tmp_path: Path) -> None:
    bundle = _valid_bundle(tmp_path)
    root = tmp_path / "corpus"
    write_bundle(root, bundle)
    loader = SpatialCorpusLoader(root)

    selector = SpatialQASelector(loader, SpatialCorpusSelection(variant=MapVariant.NOISY_02))

    assert selector.current_instance().snapshot.variant is MapVariant.NOISY_02
    assert selector.current_instance().scene.split.value == "development"


def test_view_cli_once_initializes_selector_and_renders_selection(
    tmp_path: Path, monkeypatch, capsys
) -> None:
    bundle = _valid_bundle(tmp_path)
    root = tmp_path / "corpus"
    write_bundle(root, bundle)
    boundaries: list[ViserReadOnlyBoundary] = []

    class CapturingBoundary(ViserReadOnlyBoundary):
        def __init__(self) -> None:
            super().__init__()
            boundaries.append(self)

    monkeypatch.setattr(spatial_cli, "ViserReadOnlyBoundary", CapturingBoundary)

    code = main(
        [
            "view",
            "--root",
            str(root),
            "--predicate",
            Predicate.SAME_ROOM.value,
            "--variant",
            MapVariant.CLEAN.value,
            "--once",
        ]
    )

    assert code == 0
    output = capsys.readouterr().out
    assert "viewer url=" in output
    assert f"scene_id={bundle.scene.scene_id}" in output
    assert "variant=clean" in output
    assert "point-cloud" in output
    assert len(boundaries) == 1
    assert boundaries[0].qa_selector is not None
    assert boundaries[0].qa_selector.selected_predicate_label == "Same Room"
    assert boundaries[0].qa_selector.selected_variant is MapVariant.CLEAN


def test_view_cli_reports_missing_viser_actionably(tmp_path: Path, monkeypatch, capsys) -> None:
    bundle = _valid_bundle(tmp_path)
    root = tmp_path / "corpus"
    write_bundle(root, bundle)

    class MissingViserBoundary:
        def __init__(self, *, host: str, port: int) -> None:
            raise ImportError("install visualization extra")

    monkeypatch.setattr(spatial_cli, "RealViserReadOnlyBoundary", MissingViserBoundary)

    code = main(["view", "--root", str(root)])

    assert code == 4
    assert "install visualization extra" in capsys.readouterr().out


def _assert_predicate_overlay(predicate: Predicate, command_names: set[str]) -> None:
    if predicate is Predicate.POSE_OCCUPANCY:
        assert "/agent-visible/query/pose-occupancy/footprint" in command_names
        assert "/agent-visible/query/pose-occupancy/heading" in command_names
    elif predicate is Predicate.STRAIGHT_TRANSLATION:
        assert "/agent-visible/query/translation-start/footprint" in command_names
        assert "/agent-visible/query/translation-sweep" in command_names
    elif predicate is Predicate.IN_PLACE_ROTATION:
        assert "/agent-visible/query/rotation-pose/footprint" in command_names
        assert "/agent-visible/query/rotation-arc" in command_names
    elif predicate in {
        Predicate.SAME_ROOM,
        Predicate.DIRECT_ROOM_CONNECTION,
        Predicate.DIRECT_NEIGHBOR_COUNT,
    }:
        assert "/agent-visible/query/markers" in command_names
    else:
        assert predicate is Predicate.ELIGIBLE_ROOM_COUNT


def _inspector_rows(command: DrawCommand) -> dict[str, str]:
    return {row.label: row.value for row in command.rows}
