from __future__ import annotations

import base64
import json
import os
from pathlib import Path
from typing import Any, Literal
from urllib import error, request

from pydantic import Field, ValidationError, field_validator

from dimos.experimental.dogops.models import DogOpsModel


GEMINI_API_URL = "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
SUPPORTED_INLINE_IMAGE_MIME_TYPES = {"image/jpeg", "image/png", "image/webp"}


class GeminiImageInspection(DogOpsModel):
    schema_version: int = 1
    ok: bool
    summary: str
    current_description: str
    baseline_description: str | None = None
    changed: bool
    change_summary: str
    change_type: Literal[
        "no_change",
        "object_added",
        "object_removed",
        "moved",
        "damaged",
        "blocked",
        "unclear",
        "other",
    ]
    severity: Literal["info", "p3", "p2", "p1"]
    confidence: float
    observations: list[str] = Field(default_factory=list)
    possible_incident: bool = False
    recommended_action: str = ""
    compared_evidence_ids: list[str] = Field(default_factory=list)
    limitations: list[str] = Field(default_factory=list)

    @field_validator("confidence")
    @classmethod
    def confidence_in_range(cls, value: float) -> float:
        result = float(value)
        if result < 0.0:
            return 0.0
        if result > 1.0:
            return 1.0
        return result


class GeminiVisionResult(DogOpsModel):
    ok: bool
    status: Literal[
        "completed",
        "gemini_unavailable",
        "image_missing",
        "unsupported_mime_type",
        "request_too_large",
        "api_error",
        "invalid_response",
    ]
    message: str
    model: str
    inspection: GeminiImageInspection | None = None
    raw_response: dict[str, Any] | None = None


def gemini_image_inspection_schema() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "schema_version": {"type": "integer"},
            "ok": {"type": "boolean"},
            "summary": {"type": "string"},
            "current_description": {"type": "string"},
            "baseline_description": {"type": ["string", "null"]},
            "changed": {"type": "boolean"},
            "change_summary": {"type": "string"},
            "change_type": {
                "type": "string",
                "enum": [
                    "no_change",
                    "object_added",
                    "object_removed",
                    "moved",
                    "damaged",
                    "blocked",
                    "unclear",
                    "other",
                ],
            },
            "severity": {"type": "string", "enum": ["info", "p3", "p2", "p1"]},
            "confidence": {"type": "number"},
            "observations": {"type": "array", "items": {"type": "string"}},
            "possible_incident": {"type": "boolean"},
            "recommended_action": {"type": "string"},
            "compared_evidence_ids": {"type": "array", "items": {"type": "string"}},
            "limitations": {"type": "array", "items": {"type": "string"}},
        },
        "required": [
            "ok",
            "summary",
            "current_description",
            "changed",
            "change_summary",
            "change_type",
            "severity",
            "confidence",
            "observations",
            "possible_incident",
            "recommended_action",
            "limitations",
        ],
    }


def inspect_images_with_gemini(
    *,
    current_image_path: str | Path,
    current_mime_type: str,
    current_evidence_id: str,
    prompt: str,
    model: str = "gemini-2.5-flash",
    baseline_image_path: str | Path | None = None,
    baseline_mime_type: str | None = None,
    baseline_evidence_id: str | None = None,
    max_image_bytes_inline: int = 20_000_000,
    route_context: dict[str, Any] | None = None,
    api_key: str | None = None,
    timeout_s: float = 30.0,
) -> GeminiVisionResult:
    api_key = api_key if api_key is not None else os.environ.get("GEMINI_API_KEY")
    if not api_key:
        return GeminiVisionResult(
            ok=False,
            status="gemini_unavailable",
            message="GEMINI_API_KEY is not configured",
            model=model,
        )

    current_path = Path(current_image_path)
    if not current_path.exists() or not current_path.is_file():
        return GeminiVisionResult(
            ok=False,
            status="image_missing",
            message="current image evidence file is missing",
            model=model,
        )
    if current_mime_type not in SUPPORTED_INLINE_IMAGE_MIME_TYPES:
        return GeminiVisionResult(
            ok=False,
            status="unsupported_mime_type",
            message=f"unsupported current image MIME type: {current_mime_type}",
            model=model,
        )

    image_parts: list[dict[str, Any]] = []
    try:
        image_parts.append(_inline_image_part(current_path, current_mime_type, max_image_bytes_inline))
        if baseline_image_path is not None:
            baseline_path = Path(baseline_image_path)
            if not baseline_path.exists() or not baseline_path.is_file():
                return GeminiVisionResult(
                    ok=False,
                    status="image_missing",
                    message="baseline image evidence file is missing",
                    model=model,
                )
            baseline_type = baseline_mime_type or "application/octet-stream"
            if baseline_type not in SUPPORTED_INLINE_IMAGE_MIME_TYPES:
                return GeminiVisionResult(
                    ok=False,
                    status="unsupported_mime_type",
                    message=f"unsupported baseline image MIME type: {baseline_type}",
                    model=model,
                )
            image_parts.append(_inline_image_part(baseline_path, baseline_type, max_image_bytes_inline))
    except ValueError as exc:
        return GeminiVisionResult(
            ok=False,
            status="request_too_large",
            message=str(exc),
            model=model,
        )

    request_payload = {
        "contents": [
            {
                "parts": [
                    {
                        "text": _prompt_text(
                            prompt=prompt,
                            has_baseline=baseline_image_path is not None,
                            route_context=route_context or {},
                        )
                    },
                    *image_parts,
                ]
            }
        ],
        "generationConfig": {
            "responseMimeType": "application/json",
            "responseJsonSchema": gemini_image_inspection_schema(),
        },
    }
    http_request = request.Request(
        GEMINI_API_URL.format(model=model),
        data=json.dumps(request_payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "x-goog-api-key": api_key,
        },
        method="POST",
    )
    try:
        with request.urlopen(http_request, timeout=timeout_s) as response:
            raw = json.loads(response.read().decode("utf-8"))
    except (error.HTTPError, error.URLError, TimeoutError, json.JSONDecodeError) as exc:
        return GeminiVisionResult(
            ok=False,
            status="api_error",
            message=f"Gemini request failed: {exc.__class__.__name__}",
            model=model,
        )

    try:
        text = raw["candidates"][0]["content"]["parts"][0]["text"]
        parsed = json.loads(text)
        parsed["compared_evidence_ids"] = [
            item
            for item in [current_evidence_id, baseline_evidence_id]
            if item
        ]
        inspection = GeminiImageInspection.model_validate(parsed)
    except (KeyError, IndexError, TypeError, json.JSONDecodeError, ValidationError) as exc:
        return GeminiVisionResult(
            ok=False,
            status="invalid_response",
            message=f"Gemini returned invalid structured output: {exc.__class__.__name__}",
            model=model,
            raw_response=raw if isinstance(raw, dict) else None,
        )

    return GeminiVisionResult(
        ok=True,
        status="completed",
        message=inspection.summary,
        model=model,
        inspection=inspection,
        raw_response=raw,
    )


def _inline_image_part(path: Path, mime_type: str, max_image_bytes_inline: int) -> dict[str, Any]:
    size = path.stat().st_size
    if size > max_image_bytes_inline:
        raise ValueError(f"image evidence exceeds inline Gemini limit: {size} bytes")
    return {
        "inlineData": {
            "mimeType": mime_type,
            "data": base64.b64encode(path.read_bytes()).decode("ascii"),
        }
    }


def _prompt_text(*, prompt: str, has_baseline: bool, route_context: dict[str, Any]) -> str:
    comparison_instruction = (
        "Compare the current image with the baseline image from the same waypoint."
        if has_baseline
        else "Inspect only the current image because no baseline was available."
    )
    return "\n".join(
        [
            prompt,
            comparison_instruction,
            "Be conservative. Do not infer identities or people.",
            "Do not claim a change unless visible evidence supports it.",
            "Use facility, asset, package, and work-order language.",
            "Return only JSON matching the provided schema.",
            f"Route context JSON: {json.dumps(route_context, sort_keys=True)}",
        ]
    )
