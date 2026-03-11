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

"""VLM interaction layer for temporal memory.

Isolated from state and I/O — accepts frames + state dict, returns
parsed results and raw VLM text for logging.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

from . import temporal_utils as tu

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel
    from dimos.msgs.sensor_msgs import Image

    from .frame_window_accumulator import Frame

logger = setup_logger()


@dataclass
class AnalysisResult:
    """Result of a single window analysis VLM call."""

    parsed: dict[str, Any]
    raw_vlm_response: str
    w_start: float
    w_end: float


@dataclass
class SummaryResult:
    """Result of a rolling-summary VLM call."""

    summary_text: str
    raw_vlm_response: str


@dataclass
class QueryResult:
    """Result of a query VLM call."""

    answer: str
    raw_vlm_response: str


class WindowAnalyzer:
    """Handles all VLM interactions for temporal memory.

    Stateless — caller provides frames, state snapshots, and config.
    """

    def __init__(self, vlm: VlModel, *, max_tokens: int = 900, temperature: float = 0.2) -> None:
        self._vlm = vlm
        self.max_tokens = max_tokens
        self.temperature = temperature

    @property
    def vlm(self) -> VlModel:
        return self._vlm

    # ------------------------------------------------------------------
    # VLM Call #1: Window analysis
    # ------------------------------------------------------------------

    def analyze_window(
        self,
        frames: list[Frame],
        state_dict: dict[str, Any],
        w_start: float,
        w_end: float,
    ) -> AnalysisResult | None:
        """Run VLM window analysis. Returns None on failure."""
        query = tu.build_window_prompt(
            w_start=w_start,
            w_end=w_end,
            frame_count=len(frames),
            state=state_dict,
        )
        try:
            fmt = tu.get_structured_output_format()
            if len(frames) > 1:
                responses = self._vlm.query_batch(
                    [f.image for f in frames], query, response_format=fmt
                )
                raw = responses[0] if responses else ""
            else:
                raw = self._vlm.query(frames[0].image, query, response_format=fmt)
        except Exception as e:
            logger.error(f"vlm query failed [{w_start:.1f}-{w_end:.1f}s]: {e}", exc_info=True)
            return None

        if raw is None:
            return None

        parsed = tu.parse_window_response(raw, w_start, w_end, len(frames))
        return AnalysisResult(parsed=parsed, raw_vlm_response=raw, w_start=w_start, w_end=w_end)

    # ------------------------------------------------------------------
    # VLM Call #2: Distance estimation (delegated to EntityGraphDB)
    # ------------------------------------------------------------------
    # Distance estimation is handled by EntityGraphDB.estimate_and_save_distances.
    # It's called from the orchestrator, not here.

    # ------------------------------------------------------------------
    # VLM Call #3: Rolling summary
    # ------------------------------------------------------------------

    def update_summary(
        self,
        latest_frame: Image,
        rolling_summary: str,
        chunk_buffer: list[dict[str, Any]],
    ) -> SummaryResult | None:
        """Generate updated rolling summary. Returns None on failure."""
        if not chunk_buffer or not latest_frame:
            return None

        prompt = tu.build_summary_prompt(
            rolling_summary=rolling_summary,
            chunk_windows=chunk_buffer,
        )
        try:
            raw = self._vlm.query(latest_frame, prompt)
            if raw and raw.strip():
                return SummaryResult(summary_text=raw.strip(), raw_vlm_response=raw)
        except Exception as e:
            logger.error(f"summary update failed: {e}", exc_info=True)
        return None

    # ------------------------------------------------------------------
    # VLM Call #5: Query answer
    # ------------------------------------------------------------------

    def answer_query(
        self,
        question: str,
        context: dict[str, Any],
        latest_frame: Image,
    ) -> QueryResult | None:
        """Answer a user query. Returns None on failure."""
        prompt = tu.build_query_prompt(question=question, context=context)
        try:
            raw = self._vlm.query(latest_frame, prompt)
            return QueryResult(answer=raw.strip(), raw_vlm_response=raw)
        except Exception as e:
            logger.error(f"query failed: {e}", exc_info=True)
            return None
