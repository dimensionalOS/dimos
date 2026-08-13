"""Validated input configuration for resumable VQA dataset generation."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

QUESTION_MODEL = "gpt-4o-mini"
ORACLE_MODEL = "gpt-4o-mini"


class GroundingConfig(BaseModel):
    """Private grounding quality thresholds for generated VQA labels."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)

    min_mask_area_px: int = Field(default=128, ge=1)
    min_foreground_points: int = Field(default=3, ge=1)


class GenerationConfig(BaseModel):
    """One reproducible multi-frame VQA generation request."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)

    recording: str = Field(min_length=1)
    start_index: int = Field(default=0, ge=0)
    stop_index: int = Field(gt=0)
    stride: int = Field(default=1, ge=1)
    question_mode: Literal["constrained", "agentic"] = "constrained"
    grounding: GroundingConfig = GroundingConfig()
    output: str | None = None
