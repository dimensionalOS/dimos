from __future__ import annotations

from dataclasses import dataclass
import base64
import json
import os
from typing import Any

import requests
from requests import HTTPError


OPENROUTER_CHAT_URL = "https://openrouter.ai/api/v1/chat/completions"
DEFAULT_MODEL = "openai/gpt-4o-mini"


@dataclass(frozen=True)
class VisionPost:
    caption: str
    reason: str
    score: float


class OpenRouterVision:
    def __init__(self) -> None:
        self.api_key = os.getenv("OPENROUTER_API_KEY", "").strip()
        self.model = _env(
            "KORA_SOCIAL_OPENROUTER_MODEL",
            DEFAULT_MODEL,
            legacy="INFLUENCER_OPENROUTER_MODEL",
        ).strip()
        self.timeout_sec = float(
            _env("KORA_SOCIAL_OPENROUTER_TIMEOUT_SEC", "30", legacy="INFLUENCER_OPENROUTER_TIMEOUT_SEC")
        )

    def enabled(self) -> bool:
        return bool(self.api_key)

    def status(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled(),
            "model": self.model,
            "provider": "openrouter",
        }

    def draft_post(self, jpeg: bytes) -> VisionPost:
        prompt = (
            "You are captioning a Go2 robot camera snapshot for a playful X/Twitter post. "
            "Return strict JSON with keys caption, reason, and score. caption must be under "
            "220 characters, casual, not cringe, and not mention private identity. reason "
            "should explain why the frame is shareable in one short sentence. score is a "
            "number from 0 to 1 for social-post quality."
        )
        data = self._chat_json(jpeg, prompt)
        caption = _clean_text(str(data.get("caption", "")), 220)
        reason = _clean_text(str(data.get("reason", "")), 180)
        score = _clean_score(data.get("score"))
        if not caption or not reason:
            raise ValueError("OpenRouter response did not include caption and reason")
        return VisionPost(caption=caption, reason=reason, score=score)

    def describe(self, jpeg: bytes, question: str = "What do you see?") -> str:
        prompt = (
            "Answer from the robot camera's point of view. Be concrete, concise, and avoid "
            "claiming identities or sensitive attributes. Keep it to one or two sentences.\n\n"
            f"Question: {question}"
        )
        text = self._chat_text(jpeg, prompt)
        return _clean_text(text, 500)

    def classify_mention_command(self, text: str, bot_handle: str = "@DimensionalKora") -> dict[str, str]:
        prompt = (
            "Classify an X/Twitter mention to a robot social-media bot into one safe command. "
            "Return strict JSON with keys command_type and command_payload. Allowed command_type "
            "values: vision, capture, ignored. Use vision when the user asks what the robot sees, "
            "asks it to look around, or asks a visual question. Use capture when the user asks for "
            "a photo, selfie, picture, post, or snapshot. Use ignored for movement, unsafe requests, "
            "spam, jokes with no action, or anything ambiguous. command_payload should be a concise "
            "question/instruction for vision or capture, otherwise empty. Public internet commands "
            "must never move the robot.\n\n"
            f"Bot handle: {bot_handle}\nMention: {text}"
        )
        data = self._chat_text_only_json(prompt)
        command_type = str(data.get("command_type", "ignored")).strip().lower()
        if command_type not in {"vision", "capture", "ignored"}:
            command_type = "ignored"
        command_payload = _clean_text(str(data.get("command_payload", "")), 240)
        return {"command_type": command_type, "command_payload": command_payload}

    def suggest_tags(self, scene_jpeg: bytes, candidates: list[dict[str, Any]]) -> list[str]:
        if not candidates:
            return []

        content: list[dict[str, Any]] = [
            _image_content(scene_jpeg),
            {
                "type": "text",
                "text": (
                    "The first image is the robot camera scene. The following reference images "
                    "belong to opt-in X tag presets. Suggest handles only when the scene appears "
                    "to contain the same visible participant as a reference image. Be conservative. "
                    "Return strict JSON: {\"handles\": [\"@handle\"]}. Do not identify anyone "
                    "outside the provided opt-in candidates.\n\nCandidates:\n"
                    + "\n".join(
                        f"{index + 1}. {candidate['handle']}"
                        + (f" - {candidate['notes']}" if candidate.get("notes") else "")
                        for index, candidate in enumerate(candidates)
                    )
                ),
            },
        ]
        for candidate in candidates:
            content.append(_image_content(candidate["jpeg"]))

        data = self._chat_json_content(content)
        handles = data.get("handles", [])
        if not isinstance(handles, list):
            return []
        allowed = {str(candidate["handle"]) for candidate in candidates}
        return [handle for handle in (_clean_handle(value) for value in handles) if handle in allowed]

    def _chat_json(self, jpeg: bytes, prompt: str) -> dict[str, Any]:
        text = self._chat_text(jpeg, prompt, response_format={"type": "json_object"})
        return _parse_json_object(text)

    def _chat_json_content(self, content: list[dict[str, Any]]) -> dict[str, Any]:
        text = self._chat_content(content, response_format={"type": "json_object"})
        return _parse_json_object(text)

    def _chat_text_only_json(self, prompt: str) -> dict[str, Any]:
        text = self._chat_content(
            [{"type": "text", "text": prompt}],
            response_format={"type": "json_object"},
        )
        return _parse_json_object(text)

    def _chat_text(
        self,
        jpeg: bytes,
        prompt: str,
        response_format: dict[str, Any] | None = None,
    ) -> str:
        return self._chat_content(
            [_image_content(jpeg), {"type": "text", "text": prompt}],
            response_format=response_format,
        )

    def _chat_content(
        self,
        content: list[dict[str, Any]],
        response_format: dict[str, Any] | None = None,
    ) -> str:
        if not self.enabled():
            raise ValueError("OPENROUTER_API_KEY is not set")

        payload: dict[str, Any] = {
            "model": self.model,
            "messages": [{"role": "user", "content": content}],
        }
        if response_format is not None:
            payload["response_format"] = response_format

        response = requests.post(
            OPENROUTER_CHAT_URL,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
                "X-Title": "DimOS Kora Social",
            },
            json=payload,
            timeout=self.timeout_sec,
        )
        try:
            response.raise_for_status()
        except HTTPError as exc:
            detail = _clean_text(response.text, 240)
            raise RuntimeError(
                f"OpenRouter request failed with HTTP {response.status_code}: {detail}"
            ) from exc
        body = response.json()
        return str(body["choices"][0]["message"].get("content") or "")


def _parse_json_object(text: str) -> dict[str, Any]:
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start == -1 or end == -1 or end <= start:
            raise
        parsed = json.loads(text[start : end + 1])
    if not isinstance(parsed, dict):
        raise ValueError("OpenRouter JSON response was not an object")
    return parsed


def _env(name: str, default: str, *, legacy: str | None = None) -> str:
    value = os.getenv(name)
    if value is not None:
        return value
    if legacy is not None:
        value = os.getenv(legacy)
        if value is not None:
            return value
    return default


def _image_content(jpeg: bytes) -> dict[str, Any]:
    image_data = base64.b64encode(jpeg).decode("ascii")
    return {
        "type": "image_url",
        "image_url": {
            "url": f"data:image/jpeg;base64,{image_data}",
        },
    }


def _clean_handle(value: Any) -> str:
    handle = " ".join(str(value).strip().split())
    if handle and not handle.startswith("@"):
        handle = f"@{handle}"
    return handle


def _clean_text(value: str, max_len: int) -> str:
    clean = " ".join(value.strip().split())
    if len(clean) <= max_len:
        return clean
    return clean[: max_len - 1].rstrip() + "..."


def _clean_score(value: Any) -> float:
    try:
        score = float(value)
    except (TypeError, ValueError):
        return 0.7
    return min(1.0, max(0.0, score))
