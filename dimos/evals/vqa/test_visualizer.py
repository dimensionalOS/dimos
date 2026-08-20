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

from pathlib import Path
from typing import Any

from fastapi.testclient import TestClient
import numpy as np

from dimos.evals.vqa.editor import EditorState, FrameDraft, SubmitResult
from dimos.evals.vqa.visualizer import create_editor_app
from dimos.msgs.sensor_msgs.Image import Image


class FakeSession:
    def __init__(self, output: Path) -> None:
        self.output = output
        self.started = False
        self.updated: FrameDraft | None = None

    def start(self) -> FakeSession:
        self.started = True
        return self

    def stop(self) -> None:
        self.started = False

    def state(self) -> EditorState:
        return EditorState(
            recording="recording.db",
            output=self.output,
            image_count=3,
            existing_frames=(1,),
            dirty_frames=(),
        )

    def draft(self, frame_index: int) -> FrameDraft:
        if frame_index >= 3:
            raise IndexError("out of range")
        return FrameDraft(index=frame_index)

    def raw_image(self, frame_index: int) -> Image:
        if frame_index >= 3:
            raise IndexError("out of range")
        return Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8))

    def replace_draft(self, draft: FrameDraft) -> FrameDraft:
        self.updated = draft
        return draft

    def generate(self, indices: Any) -> tuple[FrameDraft, ...]:
        return tuple(FrameDraft(index=index) for index in indices)

    def submit(self) -> SubmitResult:
        return SubmitResult(output=self.output, frame_count=1, question_count=2)


def test_editor_app_serves_page_and_session_routes(tmp_path: Path) -> None:
    session = FakeSession(tmp_path)
    app = create_editor_app(session)  # type: ignore[arg-type]

    with TestClient(app) as client:
        assert session.started
        page = client.get("/")
        assert page.status_code == 200
        assert "VQA Frame Editor" in page.text
        assert client.get("/api/session").json()["image_count"] == 3
        assert client.get("/api/frames/1").json() == {"index": 1, "questions": []}
        assert client.get("/api/frames/9").status_code == 404
        assert client.get("/api/frames/3/image").status_code == 404
        generated = client.post("/api/generate", json={"start": 0, "stop": 3, "stride": 2}).json()
        assert [draft["index"] for draft in generated] == [0, 2]
        assert client.post("/api/submit").json()["question_count"] == 2

    assert not session.started


def test_editor_app_rejects_mismatched_draft_index(tmp_path: Path) -> None:
    session = FakeSession(tmp_path)
    app = create_editor_app(session)  # type: ignore[arg-type]

    with TestClient(app) as client:
        response = client.put("/api/frames/1", json={"index": 2, "questions": []})

    assert response.status_code == 400
    assert session.updated is None
