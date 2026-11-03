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

"""Contracts for --with-localize generation (ego + cross-view)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.evals.vqa.author import FixedQuestionAuthor
from dimos.evals.vqa.contracts import InsufficientEvidenceError, QuestionProposal
from dimos.evals.vqa.families import (
    AVAILABLE_FAMILIES,
    EGO_ANY_FORE_AFT_FAMILY,
    EGO_FORE_AFT_FAMILY,
    EGO_TURN_FAMILY,
    HEIGHT_ORDER_FAMILY,
    answer_question,
)
from dimos.evals.vqa.generate import (
    GenerationFrame,
    GenerationRequest,
    GenerationResult,
    PublicCase,
    generate_dataset,
    generate_frames_dataset,
)
from dimos.evals.vqa.pointcloud_frame import PointCloudFrameUnavailableError
from dimos.evals.vqa.primitives.localize_scene import (
    LocalizeInstanceEvidence,
    LocalizeSceneEvidence,
)
from dimos.evals.vqa.suite import load_suite
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class _TestVlModel(VlModel):
    def __init__(self) -> None:
        pass

    def query(self, image: Image, query: str, **kwargs: object) -> str:
        raise NotImplementedError

    def stop(self) -> None:
        pass


class _Detector(_TestVlModel):
    def __init__(self, present: bool) -> None:
        self._present = present

    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
        detections = []
        if self._present:
            detections.append(
                Detection2DBBox(
                    bbox=(0.0, 0.0, 2.0, 2.0),
                    track_id=0,
                    class_id=-1,
                    confidence=1.0,
                    name=query,
                    ts=image.ts,
                    image=image,
                )
            )
        return ImageDetections2D(image, detections)


def test_materialize_cross_view_from_shared_localize_views(tmp_path: Path) -> None:
    """Cross-view uses localize window sightings + color_frames (no second localize)."""
    from dimos.evals.vqa.generate import _materialize_cross_view_clips
    from dimos.evals.vqa.primitives.cross_view_identity import SightingView
    from dimos.msgs.sensor_msgs.Image import ImageFormat

    def _image(ts: float) -> Image:
        rgb = np.full((200, 200, 3), 40, dtype=np.uint8)
        return Image.from_numpy(rgb, format=ImageFormat.RGB, frame_id="camera_optical", ts=ts)

    def _view(instance_id: str, ts: float) -> SightingView:
        return SightingView(
            query="storage box",
            instance_id=instance_id,
            image=_image(ts),
            bbox=(70.0, 70.0, 130.0, 130.0),
            confidence=0.7,
            timestamp=ts,
            n_views=5,
            semantic_score=0.7,
        )

    color_frames = tuple((float(t), _image(float(t))) for t in range(10, 50))
    views_by_query = {
        "storage box": {
            # min_views gates on sighting count (not Localization.n_views metadata).
            "aaaa1111": (
                _view("aaaa1111", 20.0),
                _view("aaaa1111", 30.0),
                _view("aaaa1111", 40.0),
            ),
            "bbbb2222": (
                _view("bbbb2222", 25.0),
                _view("bbbb2222", 26.0),
                _view("bbbb2222", 27.0),
            ),
        }
    }
    staging = tmp_path / "staging"
    staging.mkdir()
    (staging / "audit").mkdir()
    cases, labels, stats, audit_by_id = _materialize_cross_view_clips(
        staging,
        [],
        [],
        color_frames=color_frames,
        views_by_query=views_by_query,
        min_views=3,
    )
    assert stats["question_count"] >= 1
    assert len(cases) == len(labels) == stats["question_count"]
    assert all(case.clips is not None and case.image is None for case in cases)
    assert any((staging / "assets" / "clips").rglob("manifest.json"))
    assert "first_clip" in cases[0].question and "second_clip" in cases[0].question
    assert set(audit_by_id) == {case.id for case in cases}


def test_materialize_cross_view_empty_views_is_noop(tmp_path: Path) -> None:
    from dimos.evals.vqa.generate import _materialize_cross_view_clips

    staging = tmp_path / "staging"
    staging.mkdir()
    (staging / "audit").mkdir()
    cases, labels, stats, audit = _materialize_cross_view_clips(
        staging,
        [],
        [],
        color_frames=(),
        views_by_query={"chair": {}},
        min_views=3,
    )
    assert cases == [] and labels == []
    assert stats["question_count"] == 0
    assert stats["n_pairs_proposed"] == 0
    assert audit == {}


def test_generate_frames_publishes_cross_view_when_frame_cases_empty(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Empty per-frame cases must not abort before cross-view materialization."""
    from dimos.evals.vqa.generate import GenerationFrame, generate_frames_dataset
    from dimos.evals.vqa.primitives.cross_view_identity import SightingView
    from dimos.msgs.sensor_msgs.Image import ImageFormat

    def _image(ts: float) -> Image:
        rgb = np.full((200, 200, 3), 40, dtype=np.uint8)
        return Image.from_numpy(rgb, format=ImageFormat.RGB, frame_id="camera_optical", ts=ts)

    centered = (70.0, 70.0, 130.0, 130.0)

    def _view(ts: float) -> SightingView:
        return SightingView(
            query="box",
            instance_id="aaaa1111",
            image=_image(ts),
            bbox=centered,
            confidence=0.7,
            timestamp=ts,
            n_views=5,
            semantic_score=0.7,
            camera_frame="camera_optical",
        )

    class _SilentAuthor:
        def propose(self, image: Image, families: object) -> tuple[object, ...]:
            del image, families
            return ()

    color_frames = tuple((float(t), _image(float(t))) for t in range(10, 50))
    views_by_query = {
        "box": {
            "aaaa1111": (_view(20.0), _view(30.0), _view(40.0)),
        }
    }
    output = tmp_path / "out"
    result = generate_frames_dataset(
        GenerationRequest(
            dataset="recording.db",
            image_index=0,
            output=output,
            with_localize=True,
            min_views=3,
        ),
        (GenerationFrame(0, _image(25.0)),),
        _SilentAuthor(),  # type: ignore[arg-type]
        _Detector(present=False),
        color_frames=color_frames,
        color_frames_by_camera={"camera_optical": color_frames},
        views_by_query=views_by_query,
    )
    assert result.cases
    assert all(case.id.startswith("xview-") for case in result.cases)


def test_materialize_cross_view_audits_rejected_pairs(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from dimos.evals.vqa.generate import _materialize_cross_view_clips
    from dimos.evals.vqa.primitives.cross_view_identity import IdentityPair, SightingView
    from dimos.msgs.sensor_msgs.Image import ImageFormat

    def _image(ts: float) -> Image:
        rgb = np.full((200, 200, 3), 40, dtype=np.uint8)
        return Image.from_numpy(rgb, format=ImageFormat.RGB, frame_id="camera_optical", ts=ts)

    good = SightingView(
        query="box",
        instance_id="aaaa1111",
        image=_image(1.0),
        bbox=(70.0, 70.0, 130.0, 130.0),
        confidence=0.7,
        timestamp=1.0,
        n_views=5,
        semantic_score=0.7,
    )
    bad = SightingView(
        query="box",
        instance_id="aaaa1111",
        image=_image(5.0),
        bbox=(0.0, 0.0, 20.0, 20.0),
        confidence=0.7,
        timestamp=5.0,
        n_views=5,
        semantic_score=0.7,
    )
    ok_right = SightingView(
        query="box",
        instance_id="aaaa1111",
        image=_image(12.0),
        bbox=(70.0, 70.0, 130.0, 130.0),
        confidence=0.7,
        timestamp=12.0,
        n_views=5,
        semantic_score=0.7,
    )
    color_frames = tuple((float(t), _image(float(t))) for t in range(20))
    monkeypatch.setattr(
        "dimos.evals.vqa.generate.propose_identity_pairs",
        lambda *_a, **_k: (
            IdentityPair(query="box", left=good, right=bad, same_object=True),
            IdentityPair(query="box", left=good, right=ok_right, same_object=True),
        ),
    )
    staging = tmp_path / "staging"
    staging.mkdir()
    (staging / "audit").mkdir()
    cases, _labels, stats, _audit_by_id = _materialize_cross_view_clips(
        staging,
        [],
        [],
        color_frames=color_frames,
        views_by_query={"box": {"aaaa1111": (good,)}},
        min_views=3,
    )
    assert len(cases) == 1
    assert stats["rejected_question_count"] == 1
    audit = json.loads((staging / "audit" / "pairs.json").read_text())
    assert any(row["status"] == "rejected" for row in audit)
    assert any(row["status"] == "answered" for row in audit)


def test_answer_question_rejects_cross_view_on_frame_path() -> None:
    image = Image.from_numpy(np.zeros((8, 8, 3), dtype=np.uint8))
    with pytest.raises(InsufficientEvidenceError, match="with --with-localize"):
        answer_question(
            QuestionProposal(family="cross_view_same_object", object_names=("box",)),
            image,
            _TestVlModel(),
        )


def test_generate_dataset_attaches_localize_scene_when_enabled(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from contextlib import contextmanager
    from types import SimpleNamespace

    from dimos.evals.vqa.primitives.localize_scene import (
        LocalizeInstanceEvidence,
        LocalizeSceneEvidence,
    )

    captured_scenes: list[LocalizeSceneEvidence | None] = []
    localize_windows: list[tuple[object, object, float, float]] = []

    class _Obs:
        def __init__(self, ts: float) -> None:
            self.ts = ts

    class _Images:
        def offset(self, index: int) -> _Images:
            self._index = index
            return self

        def limit(self, _n: int) -> _Images:
            return self

        def first(self) -> _Obs:
            return _Obs(100.0 + self._index)

    class _FakeImage:
        def __init__(self, ts: float) -> None:
            self.ts = ts

    class _FakeLoader:
        def __init__(self, dataset: object, tolerance_s: float = 0.1) -> None:
            self.image_count = 10
            self.images = _Images()

        def __enter__(self) -> _FakeLoader:
            return self

        def __exit__(self, *_args: object) -> None:
            return None

        def load_image(self, index: int) -> _FakeImage:
            return _FakeImage(100.0 + index)

        def load(self, index: int) -> object:
            raise PointCloudFrameUnavailableError(f"no point cloud for {index}")

    class _FakeModel:
        def __init__(self) -> None:
            self.config = type("config", (), {"model_name": "fake"})()

        def stop(self) -> None:
            return None

    class _FakeCatalog:
        def __init__(self) -> None:
            self.config = type("config", (), {"model_name": "fake-owlv2"})()

        def stop(self) -> None:
            return None

        def names_for(self, image: object) -> tuple[str, ...]:
            del image
            return ("chair",)

        def query_detections(self, image: object, query: str, **_kwargs: object) -> object:
            del image, query
            return object()

    @contextmanager
    def fake_localize_window(
        dataset: object, queries: object, after: float, before: float, **_kwargs: object
    ):
        localize_windows.append((dataset, queries, after, before))
        fake_hit = SimpleNamespace(
            instance_id="chair-1",
            last_seen_timestamp=105.0,
            position_world_xyz=(1.0, 0.0, 0.5),
            n_views=2,
            semantic_score=0.7,
            frame_id="camera_optical",
            ambiguity_margin=0.1,
        )
        yield SimpleNamespace(
            named_hits=(("chair", fake_hit),),
            rig=object(),
            queries=("chair", "table"),
            first_seen_by_instance={"chair-1": 100.0},
            color_frames=(),
            color_frames_by_camera={},
            views_by_query={},
        )

    def fake_scene_at_timestamp(
        named_hits: object, rig: object, timestamp: float, **_kwargs: object
    ):
        del named_hits, rig, timestamp
        return LocalizeSceneEvidence(
            robot_position_xyz=(0.0, 0.0, 0.0),
            robot_yaw_rad=0.0,
            instances=(
                LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(1.0, 0.0, 0.5)),
            ),
        )

    def fake_generate_frames_dataset(
        request: GenerationRequest,
        frames: object,
        *_args: object,
        **_kwargs: object,
    ) -> GenerationResult:
        for frame in frames:  # type: ignore[attr-defined]
            captured_scenes.append(frame.localize_scene)
        return GenerationResult(
            output=request.output_directory(),
            cases=(
                PublicCase(
                    id="q",
                    image="assets/frame.png",
                    question="Is the chair ahead?",
                    choices=("ahead", "behind"),
                ),
            ),
        )

    monkeypatch.setattr("dimos.evals.vqa.generate.PointCloudFrameLoader", _FakeLoader)
    monkeypatch.setattr("dimos.evals.vqa.localize_recording.localize_window", fake_localize_window)
    monkeypatch.setattr(
        "dimos.evals.vqa.localize_recording.scene_at_timestamp", fake_scene_at_timestamp
    )
    monkeypatch.setattr("dimos.evals.vqa.generate.OpenVocabDetectionCatalog", _FakeCatalog)
    monkeypatch.setattr("dimos.models.vl.openai.OpenAIVlModel", _FakeModel)
    monkeypatch.setattr("dimos.models.vl.moondream.MoondreamVlModel", _FakeModel)
    monkeypatch.setattr(
        "dimos.evals.vqa.generate.generate_frames_dataset", fake_generate_frames_dataset
    )
    monkeypatch.setattr(
        "dimos.evals.vqa.generate.EdgeTAMObjectMaskPipeline", lambda detector: object()
    )
    monkeypatch.setattr("dimos.evals.vqa.generate.LidarRangeEstimator", lambda masks: object())
    monkeypatch.setattr("dimos.evals.vqa.generate.OpenAIQuestionAuthor", lambda model: object())

    generate_dataset(
        GenerationRequest(
            dataset="recording.db",
            image_index=4,
            output=tmp_path / "out",
            with_localize=True,
            localize_queries=("chair", "table"),
            localize_pad_s=2.5,
        )
    )

    assert localize_windows == [("recording.db", ("chair", "table"), 101.5, 106.5)]
    assert len(captured_scenes) == 1
    assert captured_scenes[0] is not None
    assert captured_scenes[0].instances[0].object_name == "chair"


def test_generation_request_defaults_localize_off() -> None:
    request = GenerationRequest(dataset="go2_short", image_index=4)
    assert request.with_localize is False
    assert request.localize_queries == ()


def test_generation_request_allows_localize_without_preset_queries() -> None:
    request = GenerationRequest(dataset="go2_short", image_index=4, with_localize=True)
    assert request.with_localize is True
    assert request.localize_queries == ()


def test_generation_request_rejects_queries_without_localize_flag() -> None:
    with pytest.raises(ValidationError, match="with_localize"):
        GenerationRequest(
            dataset="go2_short",
            image_index=4,
            localize_queries=("chair",),
        )


def _localize_scene(*, with_first_seen: bool = True) -> LocalizeSceneEvidence:
    # Distinct first_seen times so height_order clips can keep both object anchors.
    seen = {"chair": 10.0, "shelf": 12.0, "table": 14.0, "door": 11.0} if with_first_seen else {}
    return LocalizeSceneEvidence(
        robot_position_xyz=(0.0, 0.0, 0.0),
        robot_yaw_rad=0.0,
        instances=(
            LocalizeInstanceEvidence(
                object_name="chair",
                position_world_xyz=(2.0, 0.5, 0.5),
                first_seen_timestamp=seen.get("chair"),
            ),
            LocalizeInstanceEvidence(
                object_name="shelf",
                position_world_xyz=(1.5, -0.5, 1.6),
                first_seen_timestamp=seen.get("shelf"),
            ),
            LocalizeInstanceEvidence(
                object_name="table",
                position_world_xyz=(1.5, 0.8, 0.7),
                first_seen_timestamp=seen.get("table"),
            ),
            LocalizeInstanceEvidence(
                object_name="door",
                position_world_xyz=(1.0, 1.5, 1.0),
                first_seen_timestamp=seen.get("door"),
            ),
        ),
    )


def test_localize_families_are_registered() -> None:
    assert EGO_FORE_AFT_FAMILY in AVAILABLE_FAMILIES
    assert HEIGHT_ORDER_FAMILY in AVAILABLE_FAMILIES
    assert EGO_TURN_FAMILY in AVAILABLE_FAMILIES
    assert EGO_ANY_FORE_AFT_FAMILY in AVAILABLE_FAMILIES


def test_ego_fore_aft_answers_from_localize_scene() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="ego_fore_aft", object_names=("chair",))

    answer = answer_question(
        proposal, image, _Detector(present=True), localize_scene=_localize_scene()
    )

    assert answer.answer == "ahead"
    assert answer.choices == ("ahead", "behind")
    assert answer.evidence["primitive"] == "localize_scene"


def test_height_order_answers_from_localize_scene() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="height_order", object_names=("shelf", "table"))

    answer = answer_question(
        proposal, image, _Detector(present=True), localize_scene=_localize_scene()
    )

    assert answer.answer == "shelf"
    assert answer.choices == ("shelf", "table")
    objects = answer.evidence["objects"]
    assert isinstance(objects, list) and objects
    assert objects[0]["first_seen_timestamp"] == 12.0
    assert objects[1]["first_seen_timestamp"] == 14.0
    assert answer.evidence["first_seen_timestamp"] == 12.0


def test_height_order_rejects_missing_first_seen() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="height_order", object_names=("shelf", "table"))
    with pytest.raises(InsufficientEvidenceError, match="first_seen_timestamp"):
        answer_question(
            proposal,
            image,
            _Detector(present=True),
            localize_scene=_localize_scene(with_first_seen=False),
        )


def test_ego_turn_answers_from_localize_scene() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="ego_turn", object_names=("door",))

    answer = answer_question(
        proposal, image, _Detector(present=True), localize_scene=_localize_scene()
    )

    assert answer.answer == "left"
    assert answer.choices == ("left", "right", "turn around", "ahead")


def test_ego_any_fore_aft_answers_from_localize_scene() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="ego_any_fore_aft", object_names=("chair",))

    answer = answer_question(
        proposal, image, _Detector(present=True), localize_scene=_localize_scene()
    )

    assert answer.answer == "yes"
    assert answer.choices == ("yes", "no")
    assert "ahead of the robot" in answer.question


def test_ego_turn_rejects_left_right_referents() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    scene = LocalizeSceneEvidence(
        robot_position_xyz=(0.0, 0.0, 0.0),
        robot_yaw_rad=0.0,
        instances=(
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 1.2, 0.5)),
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, -1.2, 0.5)),
        ),
    )
    with pytest.raises(InsufficientEvidenceError, match="not supported"):
        answer_question(
            QuestionProposal(family="ego_turn", object_names=("left chair",)),
            image,
            _Detector(present=True),
            localize_scene=scene,
        )


def test_ego_fore_aft_rejects_left_right_referents() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    scene = LocalizeSceneEvidence(
        robot_position_xyz=(0.0, 0.0, 0.0),
        robot_yaw_rad=0.0,
        instances=(
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 1.2, 0.5)),
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, -1.2, 0.5)),
        ),
    )
    with pytest.raises(InsufficientEvidenceError, match="not supported"):
        answer_question(
            QuestionProposal(family="ego_fore_aft", object_names=("left chair",)),
            image,
            _Detector(present=True),
            localize_scene=scene,
        )


def test_ego_fore_aft_nearest_referent() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    scene = LocalizeSceneEvidence(
        robot_position_xyz=(0.0, 0.0, 0.0),
        robot_yaw_rad=0.0,
        instances=(
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(3.0, 0.0, 0.5)),
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(1.0, 0.0, 0.5)),
        ),
    )
    answer = answer_question(
        QuestionProposal(family="ego_fore_aft", object_names=("nearest chair",)),
        image,
        _Detector(present=True),
        localize_scene=scene,
    )
    assert answer.answer == "ahead"
    assert answer.evidence["selector"] == "nearest"
    assert answer.evidence["position_world_xyz"] == [1.0, 0.0, 0.5]


def test_localize_families_require_scene_evidence() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    with pytest.raises(InsufficientEvidenceError, match="localize scene"):
        answer_question(
            QuestionProposal(family="ego_fore_aft", object_names=("chair",)),
            image,
            _Detector(present=True),
        )


def test_generate_localize_families_with_fixed_author(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((8, 8, 3), dtype=np.uint8), ts=3.0)
    author = FixedQuestionAuthor(
        (
            QuestionProposal(family="ego_fore_aft", object_names=("chair",)),
            QuestionProposal(family="height_order", object_names=("table", "shelf")),
            QuestionProposal(family="ego_turn", object_names=("door",)),
        )
    )

    result = generate_frames_dataset(
        GenerationRequest(dataset="recording.db", image_index=7, output=output),
        (GenerationFrame(7, image, localize_scene=_localize_scene()),),
        author,
        _Detector(present=True),
    )

    case_ids = {case.id for case in result.cases}
    assert "frame-000007-chair-ego_fore_aft" in case_ids
    assert "frame-000007-shelf-vs-table-height_order" in case_ids
    assert "frame-000007-door-ego_turn" in case_ids
    # Verified scene also seeds deterministic localize proposals (ego_turn/height pairs).
    assert "frame-000007-chair-ego_turn" in case_ids
    labels = {
        json.loads(line)["id"]: json.loads(line)["answer"]
        for line in (output / "labels.jsonl").read_text().splitlines()
    }
    assert labels["frame-000007-chair-ego_fore_aft"] == "ahead"
    assert labels["frame-000007-shelf-vs-table-height_order"] == "shelf"
    assert labels["frame-000007-door-ego_turn"] == "left"
    suite = load_suite(output)
    assert len(suite) == len(result.cases)


def test_frame_without_localize_does_not_expose_localize_families(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    author = FixedQuestionAuthor(
        (QuestionProposal(family="ego_fore_aft", object_names=("chair",)),)
    )

    with pytest.raises(ValueError, match="no selected frame produced an answerable question"):
        generate_frames_dataset(
            GenerationRequest(dataset="recording.db", image_index=4, output=output),
            (GenerationFrame(4, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))),),
            author,
            _Detector(present=True),
        )


def test_empty_localize_scene_does_not_expose_localize_families(tmp_path: Path) -> None:
    """Pose-only empty scenes must not offer requires_localize families to the author."""
    output = tmp_path / "vqa"
    empty_scene = LocalizeSceneEvidence(
        robot_position_xyz=(0.0, 0.0, 0.0),
        robot_yaw_rad=0.0,
        instances=(),
    )
    author = FixedQuestionAuthor(
        (QuestionProposal(family="ego_fore_aft", object_names=("chair",)),)
    )

    with pytest.raises(ValueError, match="no selected frame produced an answerable question"):
        generate_frames_dataset(
            GenerationRequest(dataset="recording.db", image_index=4, output=output),
            (
                GenerationFrame(
                    4,
                    Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8)),
                    localize_scene=empty_scene,
                ),
            ),
            author,
            _Detector(present=True),
        )
