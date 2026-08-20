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

"""Local HTTP interface for generating and editing a VQA dataset."""

from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Response
from fastapi.responses import FileResponse
from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.evals.vqa.editor import FrameDraft, SubmitResult, VqaEditorSession

DEFAULT_EDITOR_PORT = 8765
_INDEX = Path(__file__).with_name("editor.html")


class GenerationSelection(BaseModel):
    """Single-frame or range generation request from the browser."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    image_index: int | None = Field(default=None, ge=0)
    start: int | None = Field(default=None, ge=0)
    stop: int | None = Field(default=None, gt=0)
    stride: int = Field(default=1, ge=1)

    @model_validator(mode="after")
    def valid_selection(self) -> GenerationSelection:
        if self.image_index is not None:
            if self.start is not None or self.stop is not None:
                raise ValueError("image_index cannot be combined with start or stop")
            return self
        if self.start is None or self.stop is None:
            raise ValueError("provide image_index or both start and stop")
        if self.stop <= self.start:
            raise ValueError("stop must be greater than start")
        return self

    def indices(self) -> range:
        if self.image_index is not None:
            return range(self.image_index, self.image_index + 1)
        assert self.start is not None and self.stop is not None
        return range(self.start, self.stop, self.stride)


def create_editor_app(session: VqaEditorSession) -> FastAPI:
    """Create a localhost-oriented editor application around one session."""

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        session.start()
        try:
            yield
        finally:
            session.stop()

    app = FastAPI(title="DimOS VQA Editor", lifespan=lifespan)

    @app.get("/", include_in_schema=False)
    def index() -> FileResponse:
        return FileResponse(_INDEX, media_type="text/html")

    @app.get("/api/session")
    def state() -> object:
        return session.state()

    @app.get("/api/frames/{frame_index}")
    def frame(frame_index: int) -> FrameDraft:
        try:
            return session.draft(frame_index)
        except (IndexError, LookupError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.get("/api/frames/{frame_index}/image")
    def image(frame_index: int) -> Response:
        try:
            data = session.raw_image(frame_index).to_jpeg_bytes(quality=85)
        except (IndexError, LookupError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.put("/api/frames/{frame_index}")
    def update_frame(frame_index: int, draft: FrameDraft) -> FrameDraft:
        if draft.index != frame_index:
            raise HTTPException(status_code=400, detail="draft index does not match URL")
        try:
            return session.replace_draft(draft)
        except (IndexError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/generate")
    def generate(selection: GenerationSelection) -> tuple[FrameDraft, ...]:
        try:
            return session.generate(selection.indices())
        except (IndexError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/submit")
    def submit() -> SubmitResult:
        try:
            return session.submit()
        except (OSError, RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    return app


def run_editor(recording: str, output: Path, port: int = DEFAULT_EDITOR_PORT) -> None:
    """Run the VQA editor on the local loopback interface."""
    import uvicorn

    session = VqaEditorSession(recording, output)
    uvicorn.run(create_editor_app(session), host="127.0.0.1", port=port)
