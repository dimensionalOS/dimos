# Copyright 2025-2026 Dimensional Inc.
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

"""Public-only CandidateAnswerer adapter for TemporalMemory evaluation."""

from collections.abc import Callable
from pathlib import Path

from dimos.benchmark.spatiotemporal.models import Question
from dimos.benchmark.spatiotemporal.ports import CandidateReadiness


class TemporalMemoryAnswerer:
    """Adapt TemporalMemory ingestion, query, and cleanup callables for evaluation."""

    def __init__(
        self,
        *,
        ingest: Callable[[Path], int],
        query: Callable[[str], str | bool | None],
        cleanup: Callable[[], None],
    ) -> None:
        self._ingest = ingest
        self._query = query
        self._cleanup = cleanup
        self._readiness: CandidateReadiness | None = None
        self._closed = False

    def ingest_video(self, video_path: Path) -> CandidateReadiness:
        """Ingest only the public source video and report candidate readiness."""
        self._require_open()
        readiness = CandidateReadiness(
            ready=(frame_count := self._ingest(video_path)) > 0,
            ingested_frame_count=frame_count,
        )
        self._readiness = readiness
        return readiness

    def answer(self, question: Question) -> str | bool | None:
        """Answer a public question only after successful video ingestion."""
        self._require_open()
        if self._readiness is None or not self._readiness.ready:
            raise RuntimeError("video must be ingested before answering questions")
        return self._query(question.text)

    def close(self) -> None:
        """Release TemporalMemory resources exactly once."""
        if self._closed:
            return
        self._closed = True
        self._cleanup()

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("TemporalMemory answerer is closed")
