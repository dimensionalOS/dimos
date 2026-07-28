"""Fail-closed visual verification for offline semantic target candidates."""

from __future__ import annotations

import base64
import binascii
from collections.abc import Sequence
import os
from pathlib import Path
from typing import Any, Literal, Protocol

import cv2
import numpy as np
from openai import (
    APIConnectionError,
    APITimeoutError,
    InternalServerError,
    OpenAI,
    OpenAIError,
    RateLimitError,
)
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator, model_validator

from dimos.perception.visual_memory import VisualMemory

Verdict = Literal["yes", "no", "uncertain"]


class CameraPose(BaseModel):
    """Optional map-frame camera pose captured with an evidence image."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    x: float
    y: float
    yaw: float


class CandidateView(BaseModel):
    """One explicitly selected JPEG image supplied to a verifier."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    frame_id: str = Field(min_length=1, max_length=200)
    jpeg_base64: str = Field(min_length=1, repr=False)
    width: int = Field(gt=0, le=4096)
    height: int = Field(gt=0, le=4096)
    mime_type: Literal["image/jpeg"] = "image/jpeg"
    captured_at: float | None = None
    camera_pose: CameraPose | None = None

    @field_validator("jpeg_base64")
    @classmethod
    def validate_jpeg_base64(cls, value: str) -> str:
        try:
            decoded = base64.b64decode(value, validate=True)
        except (ValueError, binascii.Error) as exc:
            raise ValueError("jpeg_base64 must be valid base64") from exc
        if not decoded:
            raise ValueError("jpeg_base64 must not decode to empty bytes")
        if len(decoded) > 2_000_000:
            raise ValueError("candidate JPEG exceeds the 2 MB upload limit")
        return value


class CandidateEvidenceBundle(BaseModel):
    """One candidate plus the only images authorized for verification."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    candidate_id: str = Field(min_length=1, max_length=200)
    target_description: str = Field(min_length=1, max_length=1000)
    views: list[CandidateView] = Field(min_length=1, max_length=3)

    @model_validator(mode="after")
    def validate_unique_frames(self) -> CandidateEvidenceBundle:
        frame_ids = [view.frame_id for view in self.views]
        if len(set(frame_ids)) != len(frame_ids):
            raise ValueError("candidate frame IDs must be unique")
        return self


class VerificationView(BaseModel):
    """Structured model judgment for one candidate view."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    frame_id: str = Field(min_length=1, max_length=200)
    verdict: Verdict
    bbox: tuple[float, float, float, float] | None = None

    @field_validator("bbox")
    @classmethod
    def validate_bbox(
        cls,
        value: tuple[float, float, float, float] | None,
    ) -> tuple[float, float, float, float] | None:
        if value is None:
            return None
        x1, y1, x2, y2 = value
        if any(coordinate < 0.0 or coordinate > 1.0 for coordinate in value):
            raise ValueError("bbox coordinates must be normalized from zero to one")
        if x1 >= x2 or y1 >= y2:
            raise ValueError("bbox coordinates must be ordered as x1 < x2 and y1 < y2")
        return value


class TargetVerification(BaseModel):
    """Typed semantic result; it is evidence and never movement permission."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    verdict: Verdict
    target_type: str = Field(min_length=1, max_length=200)
    confidence: float = Field(ge=0.0, le=1.0)
    passable: bool
    need_more_views: bool
    reason: str = Field(min_length=1, max_length=2000)
    views: list[VerificationView] = Field(max_length=3)

    @classmethod
    def uncertain(cls, reason: str) -> TargetVerification:
        return cls(
            verdict="uncertain",
            target_type="unknown",
            confidence=0.0,
            passable=False,
            need_more_views=True,
            reason=reason,
            views=[],
        )


def apply_verification_policy(
    bundle: CandidateEvidenceBundle,
    result: TargetVerification,
) -> TargetVerification:
    """Require two known, distinct `yes` boxes before accepting a cloud `yes`."""

    if result.verdict != "yes":
        return result

    known_frame_ids = {view.frame_id for view in bundle.views}
    returned_frame_ids = {view.frame_id for view in result.views}
    unknown_frame_ids = sorted(returned_frame_ids - known_frame_ids)
    if unknown_frame_ids:
        return TargetVerification.uncertain(
            "provider returned unknown frame IDs: " + ", ".join(unknown_frame_ids)
        )

    confirmed_frame_ids = {
        view.frame_id
        for view in result.views
        if view.verdict == "yes" and view.bbox is not None
    }
    if result.need_more_views or len(confirmed_frame_ids) < 2:
        return TargetVerification.uncertain(
            "fewer than two known views contain a positive normalized target box"
        )
    return result


class PersistedCandidateLoader:
    """Load and resize only allowlisted frames from one persisted VisualMemory."""

    def __init__(
        self,
        memory_path: str | Path,
        *,
        maximum_edge_px: int = 1024,
        jpeg_quality: int = 82,
    ) -> None:
        self.memory_path = Path(memory_path)
        if maximum_edge_px <= 0 or maximum_edge_px > 4096:
            raise ValueError("maximum_edge_px must be between 1 and 4096")
        if jpeg_quality < 1 or jpeg_quality > 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        self.maximum_edge_px = maximum_edge_px
        self.jpeg_quality = jpeg_quality

    def load(
        self,
        *,
        candidate_id: str,
        target_description: str,
        frame_ids: Sequence[str],
    ) -> CandidateEvidenceBundle:
        normalized_frame_ids = [frame_id.strip() for frame_id in frame_ids]
        if not 1 <= len(normalized_frame_ids) <= 3:
            raise ValueError("one to three frame IDs are required")
        if any(not frame_id for frame_id in normalized_frame_ids):
            raise ValueError("frame IDs must not be empty")
        if len(set(normalized_frame_ids)) != len(normalized_frame_ids):
            raise ValueError("candidate frame IDs must be unique")
        if not self.memory_path.is_file():
            raise ValueError(f"visual memory file not found: {self.memory_path}")

        memory = VisualMemory.load(str(self.memory_path))
        views: list[CandidateView] = []
        for frame_id in normalized_frame_ids:
            image = memory.get(frame_id)
            if image is None:
                raise ValueError(f"candidate frame is missing from visual memory: {frame_id}")
            resized = self._resize(image)
            success, encoded = cv2.imencode(
                ".jpg",
                resized,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if not success:
                raise ValueError(f"failed to encode candidate frame: {frame_id}")
            height, width = resized.shape[:2]
            views.append(
                CandidateView(
                    frame_id=frame_id,
                    jpeg_base64=base64.b64encode(encoded.tobytes()).decode("ascii"),
                    width=width,
                    height=height,
                )
            )
        return CandidateEvidenceBundle(
            candidate_id=candidate_id,
            target_description=target_description,
            views=views,
        )

    def _resize(self, image: np.ndarray[Any, Any]) -> np.ndarray[Any, Any]:
        height, width = image.shape[:2]
        longest_edge = max(width, height)
        if longest_edge <= self.maximum_edge_px:
            return image
        scale = self.maximum_edge_px / longest_edge
        return cv2.resize(
            image,
            (max(1, round(width * scale)), max(1, round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )


class _Responses(Protocol):
    def parse(self, **kwargs: Any) -> Any: ...


class _OpenAIClient(Protocol):
    responses: _Responses


class VisionVerifier(Protocol):
    """Provider-neutral semantic verification interface."""

    def verify(self, bundle: CandidateEvidenceBundle) -> TargetVerification: ...


class OpenAIResponsesVisionVerifier:
    """Verify selected candidate frames with typed OpenAI Responses output."""

    def __init__(
        self,
        *,
        client: _OpenAIClient | None = None,
        model: str = "gpt-5.6-terra",
        timeout_s: float = 8.0,
        max_retries: int = 1,
    ) -> None:
        if not model.strip():
            raise ValueError("model must not be empty")
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if max_retries < 0 or max_retries > 3:
            raise ValueError("max_retries must be between zero and three")
        self._provided_client = client
        self.model = model
        self.timeout_s = timeout_s
        self.max_retries = max_retries

    def verify(self, bundle: CandidateEvidenceBundle) -> TargetVerification:
        try:
            client = self._provided_client or self._create_client()
        except ValueError as exc:
            return TargetVerification.uncertain(
                f"provider configuration error: {exc}"
            )

        request_input = self._request_input(bundle)
        retryable_errors = (
            TimeoutError,
            APIConnectionError,
            APITimeoutError,
            RateLimitError,
            InternalServerError,
        )
        for attempt in range(self.max_retries + 1):
            try:
                response = client.responses.parse(
                    model=self.model,
                    instructions=(
                        "Verify a visual target from only the supplied images. "
                        "Do not infer metric distance or authorize robot movement. "
                        "Use yes only when the requested target is visible in at least "
                        "two distinct views; otherwise return no or uncertain."
                    ),
                    input=request_input,
                    text_format=TargetVerification,
                    max_output_tokens=1000,
                    store=False,
                    timeout=self.timeout_s,
                )
                parsed = getattr(response, "output_parsed", None)
                if parsed is None:
                    return TargetVerification.uncertain(
                        "provider returned no structured result"
                    )
                try:
                    result = (
                        parsed
                        if isinstance(parsed, TargetVerification)
                        else TargetVerification.model_validate(parsed)
                    )
                except ValidationError:
                    return TargetVerification.uncertain(
                        "provider returned an invalid structured result"
                    )
                return apply_verification_policy(bundle, result)
            except retryable_errors as exc:
                if attempt >= self.max_retries:
                    return TargetVerification.uncertain(
                        f"provider error after retry: {type(exc).__name__}"
                    )
            except OpenAIError as exc:
                return TargetVerification.uncertain(
                    f"provider error: {type(exc).__name__}"
                )
        return TargetVerification.uncertain("provider error: exhausted retry loop")

    @staticmethod
    def _create_client() -> OpenAI:
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY is not configured")
        return OpenAI(api_key=api_key)

    @staticmethod
    def _request_input(bundle: CandidateEvidenceBundle) -> list[dict[str, Any]]:
        content: list[dict[str, Any]] = [
            {
                "type": "input_text",
                "text": (
                    f"Target description: {bundle.target_description}\n"
                    f"Candidate ID: {bundle.candidate_id}\n"
                    "For each following labelled view, decide whether the target is "
                    "visible and return a normalized [x1, y1, x2, y2] box when present."
                ),
            }
        ]
        for view in bundle.views:
            content.extend(
                [
                    {
                        "type": "input_text",
                        "text": f"View frame_id={view.frame_id}",
                    },
                    {
                        "type": "input_image",
                        "image_url": (
                            f"data:{view.mime_type};base64,{view.jpeg_base64}"
                        ),
                        "detail": "high",
                    },
                ]
            )
        return [{"role": "user", "content": content}]
