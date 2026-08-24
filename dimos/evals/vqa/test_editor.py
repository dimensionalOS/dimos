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

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.evals.vqa import editor
from dimos.evals.vqa.editor import EditableQuestion, FrameDraft, VqaEditorSession
from dimos.evals.vqa.families import OBJECT_DISTANCE_FAMILY
from dimos.evals.vqa.generate import (
    GeneratedFrame,
    GenerationFrame,
    PrivateLabel,
    PublicCase,
)
from dimos.evals.vqa.pointcloud_frame import PointCloudFrameUnavailableError
from dimos.msgs.sensor_msgs.Image import Image


class FakePreprocessor:
    def __init__(self) -> None:
        self.started = False
        self.rectified: list[int] = []
        self.count_reads = 0

    @property
    def image_count(self) -> int:
        assert self.started
        self.count_reads += 1
        return 5

    def start(self) -> FakePreprocessor:
        self.started = True
        return self

    def stop(self) -> None:
        self.started = False

    def load_raw_image(self, frame_index: int) -> Image:
        return _image(frame_index)

    def load_image(self, frame_index: int) -> Image:
        self.rectified.append(frame_index)
        return _image(frame_index + 10)

    def load(self, frame_index: int) -> None:
        raise PointCloudFrameUnavailableError("no synchronized geometry")


def _image(value: int) -> Image:
    return Image.from_numpy(np.full((3, 4, 3), value, dtype=np.uint8), ts=float(value))


def _write_dataset(root: Path) -> None:
    (root / "assets").mkdir(parents=True)
    cases = [
        {
            "id": "frame-000001-old",
            "image": "assets/frame-000001.png",
            "question": "Old frame one?",
            "choices": ["yes", "no"],
        },
        {
            "id": "frame-000003-keep",
            "image": "assets/frame-000003.png",
            "question": "Keep frame three?",
            "choices": ["yes", "no"],
        },
        {
            "id": "custom-keep",
            "image": "assets/imported.png",
            "question": "Keep custom case?",
            "choices": ["left", "right"],
        },
    ]
    labels = [
        {"id": "frame-000001-old", "answer": "yes"},
        {"id": "frame-000003-keep", "answer": "no"},
        {"id": "custom-keep", "answer": "left"},
    ]
    (root / "cases.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in cases), encoding="utf-8"
    )
    (root / "labels.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in labels), encoding="utf-8"
    )


def _read_rows(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_editable_question_requires_valid_unique_choices() -> None:
    with pytest.raises(ValidationError, match="choices must be unique"):
        EditableQuestion(id="q", question="Question?", choices=("yes", "yes"), answer="yes")
    with pytest.raises(ValidationError, match="answer must be one of the choices"):
        EditableQuestion(id="q", question="Question?", choices=("yes", "no"), answer="maybe")


def test_session_preloads_and_reuses_generation_models(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from dimos.models.segmentation import edge_tam
    from dimos.models.vl import moondream, openai

    calls = {"author_stop": 0, "detector_start": 0, "detector_stop": 0, "segmenter": 0}

    class FakeAuthorModel:
        def stop(self) -> None:
            calls["author_stop"] += 1

    class FakeDetectorModel:
        def start(self) -> None:
            calls["detector_start"] += 1

        def stop(self) -> None:
            calls["detector_stop"] += 1

    class FakeSegmenter:
        def __init__(self) -> None:
            calls["segmenter"] += 1

    monkeypatch.setattr(openai, "OpenAIVlModel", FakeAuthorModel)
    monkeypatch.setattr(moondream, "MoondreamVlModel", FakeDetectorModel)
    monkeypatch.setattr(edge_tam, "EdgeTAMImageSegmenter", FakeSegmenter)

    session = VqaEditorSession(
        "recording.db",
        tmp_path,
        preprocessor=FakePreprocessor(),  # type: ignore[arg-type]
    ).start()
    session.preload_generation_models()
    session.preload_generation_models()

    assert calls == {
        "author_stop": 0,
        "detector_start": 1,
        "detector_stop": 0,
        "segmenter": 1,
    }

    session.stop()

    assert calls["author_stop"] == 1
    assert calls["detector_stop"] == 1


@pytest.mark.parametrize("precreate", [False, True])
def test_session_creates_or_accepts_blank_dataset_directory(
    tmp_path: Path, precreate: bool
) -> None:
    output = tmp_path / "new-dataset"
    if precreate:
        output.mkdir()
    session = VqaEditorSession(
        "recording.db",
        output,
        preprocessor=FakePreprocessor(),  # type: ignore[arg-type]
    ).start()

    assert output.is_dir()
    assert session.state().existing_frames == ()
    assert session.state().total_questions == 0
    session.replace_draft(
        FrameDraft(
            index=2,
            questions=(
                EditableQuestion(
                    id="frame-000002-manual",
                    question="Manual question?",
                    choices=("yes", "no"),
                    answer="yes",
                ),
            ),
        )
    )
    session.submit()

    assert _read_rows(output / "cases.jsonl")[0]["id"] == "frame-000002-manual"
    assert _read_rows(output / "labels.jsonl") == [{"id": "frame-000002-manual", "answer": "yes"}]
    session.stop()


def test_session_rejects_partially_initialized_dataset_directory(tmp_path: Path) -> None:
    output = tmp_path / "partial"
    output.mkdir()
    (output / "cases.jsonl").write_text("", encoding="utf-8")

    with pytest.raises(ValueError, match="both cases.jsonl and labels.jsonl"):
        VqaEditorSession(
            "recording.db",
            output,
            preprocessor=FakePreprocessor(),  # type: ignore[arg-type]
        ).start()


def test_session_keeps_edits_in_memory_until_submit_and_preserves_other_cases(
    tmp_path: Path,
) -> None:
    _write_dataset(tmp_path)
    preprocessor = FakePreprocessor()
    session = VqaEditorSession(
        "recording.db",
        tmp_path,
        preprocessor=preprocessor,  # type: ignore[arg-type]
    ).start()
    original_cases = (tmp_path / "cases.jsonl").read_text(encoding="utf-8")

    assert session.draft(1).questions[0].id == "frame-000001-old"
    assert session.state().total_questions == 3
    session.replace_draft(
        FrameDraft(
            index=1,
            questions=(
                EditableQuestion(
                    id="frame-000001-old",
                    question="Edited frame one?",
                    choices=("yes", "no"),
                    answer="no",
                ),
            ),
        )
    )
    session.replace_draft(
        FrameDraft(
            index=2,
            questions=(
                EditableQuestion(
                    id="frame-000002-manual",
                    question="Manual frame two?",
                    choices=("left", "right"),
                    answer="right",
                ),
            ),
        )
    )

    assert (tmp_path / "cases.jsonl").read_text(encoding="utf-8") == original_cases
    assert session.state().total_questions == 4
    result = session.submit()
    cases = _read_rows(tmp_path / "cases.jsonl")
    labels = {row["id"]: row["answer"] for row in _read_rows(tmp_path / "labels.jsonl")}

    assert result.frame_count == 2
    assert result.question_count == 2
    assert {row["id"] for row in cases} == {
        "frame-000001-old",
        "frame-000002-manual",
        "frame-000003-keep",
        "custom-keep",
    }
    assert next(row for row in cases if row["id"] == "frame-000001-old")["question"] == (
        "Edited frame one?"
    )
    assert next(row for row in cases if row["id"] == "custom-keep")["image"] == (
        "assets/imported.png"
    )
    assert labels["frame-000001-old"] == "no"
    assert labels["frame-000002-manual"] == "right"
    assert preprocessor.rectified == [1, 2]
    assert (tmp_path / "assets" / "frame-000001.png").is_file()
    assert (tmp_path / "assets" / "frame-000002.png").is_file()
    audited_cases = json.loads(
        (tmp_path / "audit" / "frame-000001" / "cases.json").read_text(encoding="utf-8")
    )
    audited_labels = json.loads(
        (tmp_path / "audit" / "frame-000002" / "labels.json").read_text(encoding="utf-8")
    )
    assert audited_cases[0]["question"] == "Edited frame one?"
    assert audited_labels[0]["answer"] == "right"
    run = json.loads((tmp_path / "audit" / "run.json").read_text(encoding="utf-8"))
    assert run["question_count"] == 4
    assert run["submitted_frames"] == [1, 2]
    assert session.state().dirty_frames == ()
    assert preprocessor.count_reads == 1
    session.stop()


def test_generate_uses_rectified_image_and_retains_generated_draft(tmp_path: Path) -> None:
    _write_dataset(tmp_path)
    preprocessor = FakePreprocessor()
    seen: list[GenerationFrame] = []

    def generate(source: GenerationFrame) -> GeneratedFrame:
        seen.append(source)
        case = PublicCase(
            id="frame-000002-generated",
            image="assets/frame-000002.png",
            question="Generated?",
            choices=("yes", "no"),
        )
        return GeneratedFrame(
            index=source.index,
            image=source.image,
            cases=(case,),
            labels=(PrivateLabel(id=case.id, answer="yes"),),
            audit_rows=(
                {
                    "proposal": {
                        "family": "object_distance",
                        "object_names": ["chair"],
                    },
                    "status": "rejected",
                    "reason": "not enough projected points",
                },
            ),
            families=(OBJECT_DISTANCE_FAMILY,),
        )

    session = VqaEditorSession(
        "recording.db",
        tmp_path,
        preprocessor=preprocessor,  # type: ignore[arg-type]
        frame_generator=generate,
    ).start()
    generated = session.generate(2)

    assert generated == session.draft(2)
    assert generated.questions[0].answer == "yes"
    assert generated.depth_attempt_count == 1
    assert generated.depth_answered_count == 0
    assert generated.depth_rejections == ("not enough projected points",)
    assert seen[0].image.ts == 12.0
    assert session.state().dirty_frames == (2,)
    session.submit()
    ground_truth = json.loads(
        (tmp_path / "audit" / "frame-000002" / "ground_truth.json").read_text(encoding="utf-8")
    )
    frame_audit = json.loads(
        (tmp_path / "audit" / "frame-000002" / "frame.json").read_text(encoding="utf-8")
    )
    run_audit = json.loads((tmp_path / "audit" / "run.json").read_text(encoding="utf-8"))
    assert ground_truth[0]["reason"] == "not enough projected points"
    assert frame_audit["available_families"] == ["object_distance"]
    assert frame_audit["attempted_families"] == ["object_distance"]
    assert frame_audit["answered_families"] == []
    assert run_audit["attempted_families"] == ["object_distance"]
    assert run_audit["rejected_question_count"] == 1
    session.stop()


def test_generate_preserves_questions_and_keeps_prior_success_on_failure(tmp_path: Path) -> None:
    _write_dataset(tmp_path)

    def generate(source: GenerationFrame) -> GeneratedFrame:
        if source.index == 2:
            raise RuntimeError("generation failed")
        return GeneratedFrame(
            index=source.index,
            image=source.image,
            cases=(),
            labels=(),
            audit_rows=(),
            families=(),
        )

    session = VqaEditorSession(
        "recording.db",
        tmp_path,
        preprocessor=FakePreprocessor(),  # type: ignore[arg-type]
        frame_generator=generate,
    ).start()

    generated = session.generate(1)
    assert generated.questions[0].id == "frame-000001-old"
    with pytest.raises(RuntimeError, match="generation failed"):
        session.generate(2)
    assert session.state().dirty_frames == (1,)
    session.stop()


def test_session_exclusively_locks_dataset_output(tmp_path: Path) -> None:
    _write_dataset(tmp_path)
    first = VqaEditorSession(
        "recording.db",
        tmp_path,
        preprocessor=FakePreprocessor(),  # type: ignore[arg-type]
    ).start()
    second = VqaEditorSession(
        "recording.db",
        tmp_path,
        preprocessor=FakePreprocessor(),  # type: ignore[arg-type]
    )

    with pytest.raises(RuntimeError, match="already open"):
        second.start()

    first.stop()
    second.start()
    second.stop()


def test_submit_rejects_question_id_from_an_untouched_frame(tmp_path: Path) -> None:
    _write_dataset(tmp_path)
    session = VqaEditorSession(
        "recording.db",
        tmp_path,
        preprocessor=FakePreprocessor(),  # type: ignore[arg-type]
    ).start()
    session.replace_draft(
        FrameDraft(
            index=1,
            questions=(
                EditableQuestion(
                    id="frame-000003-keep",
                    question="Collision?",
                    choices=("yes", "no"),
                    answer="yes",
                ),
            ),
        )
    )

    with pytest.raises(ValueError, match="another frame"):
        session.submit()
    session.stop()


def test_submit_restores_all_outputs_after_write_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_dataset(tmp_path)
    original_cases = (tmp_path / "cases.jsonl").read_bytes()
    original_labels = (tmp_path / "labels.jsonl").read_bytes()
    session = VqaEditorSession(
        "recording.db",
        tmp_path,
        preprocessor=FakePreprocessor(),  # type: ignore[arg-type]
    ).start()
    session.replace_draft(
        FrameDraft(
            index=2,
            questions=(
                EditableQuestion(
                    id="frame-000002-manual",
                    question="Manual?",
                    choices=("yes", "no"),
                    answer="yes",
                ),
            ),
        )
    )
    original_write = editor._write_jsonl_atomic

    def fail_cases(path: Path, rows: Any) -> None:
        if path.name == "cases.jsonl":
            raise OSError("disk full")
        original_write(path, rows)

    monkeypatch.setattr(editor, "_write_jsonl_atomic", fail_cases)

    with pytest.raises(OSError, match="disk full"):
        session.submit()

    assert (tmp_path / "cases.jsonl").read_bytes() == original_cases
    assert (tmp_path / "labels.jsonl").read_bytes() == original_labels
    assert not (tmp_path / "assets" / "frame-000002.png").exists()
    assert not (tmp_path / "audit" / "frame-000002").exists()
    assert session.state().dirty_frames == (2,)
    session.stop()
