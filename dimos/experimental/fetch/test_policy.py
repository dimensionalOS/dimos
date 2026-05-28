from __future__ import annotations

import json
from types import SimpleNamespace
from unittest.mock import patch

import policy


IMAGE_DATA_URL = "data:image/jpeg;base64,abc123"


def _completion(content: str) -> SimpleNamespace:
    return SimpleNamespace(
        choices=[SimpleNamespace(message=SimpleNamespace(content=content))]
    )


def _raw_decision() -> dict[str, object]:
    return {
        "candidate_found": True,
        "confidence": 0.8,
        "target": {
            "bearing": "center",
            "range": "inside_4m",
            "description": "person under a blue umbrella",
            "free_hand_evidence": "hands are visible and empty",
            "busy_signals": ["none"],
        },
        "safety": {"safe_to_approach": True, "stop_reason": ""},
        "offer": {"drink": True, "photo": True},
        "line": "That blue umbrella is doing serious shade work. Drink, photo, or both?",
        "notes": "available person in range",
    }


def test_missing_gemini_key_returns_default_without_client(monkeypatch) -> None:
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    monkeypatch.delenv("GOOGLE_API_KEY", raising=False)

    fetch_policy = policy.FetchPolicy(
        policy.FetchPolicyConfig(
            vision_provider="gemini",
        )
    )

    with patch("policy.OpenAI") as openai_cls:
        decision = fetch_policy.analyze_frame(IMAGE_DATA_URL)

    openai_cls.assert_not_called()
    assert decision["state"] == "search"
    assert decision["notes"] == "GEMINI_API_KEY or GOOGLE_API_KEY is not set"


def test_config_uses_provider_aware_default_model() -> None:
    assert policy.FetchPolicyConfig().model == policy.DEFAULT_OPENAI_VISION_MODEL
    assert (
        policy.FetchPolicyConfig(vision_provider="gemini").model
        == policy.DEFAULT_GEMINI_VISION_MODEL
    )


def test_config_normalizes_reasoning_effort() -> None:
    config = policy.FetchPolicyConfig(vision_provider="gemini", reasoning_effort=" LOW ")

    assert config.reasoning_effort == "low"


def test_config_rejects_unknown_reasoning_effort() -> None:
    try:
        policy.FetchPolicyConfig(vision_provider="gemini", reasoning_effort="maximum")
    except ValueError as exc:
        assert "reasoning_effort must be none" in str(exc)
    else:
        raise AssertionError("Expected unknown reasoning_effort to be rejected")


def test_config_rejects_known_provider_model_mismatch() -> None:
    try:
        policy.FetchPolicyConfig(model="gpt-5-mini", vision_provider="gemini")
    except ValueError as exc:
        assert "appears to be a openai model" in str(exc)
    else:
        raise AssertionError("Expected provider/model mismatch to be rejected")


def test_extract_json_object_ignores_extra_braces_around_response() -> None:
    parsed = policy._extract_json_object(
        'debug {not json} {"candidate_found": false} tail {x}'
    )

    assert parsed == {"candidate_found": False}


def test_invalid_image_url_returns_default_without_client(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "openai-key")

    fetch_policy = policy.FetchPolicy()

    with patch("policy.OpenAI") as openai_cls:
        decision = fetch_policy.analyze_frame("not-an-image")

    openai_cls.assert_not_called()
    assert decision["state"] == "search"
    assert decision["notes"] == "Expected an image data URL"
    assert decision["simulated_cmd_vel"] == {
        "linear_x": 0.0,
        "angular_z": 0.0,
        "duration_s": 0.0,
    }


def test_search_state_turns_in_place_when_no_candidate() -> None:
    raw = _raw_decision()
    raw["candidate_found"] = False
    raw["confidence"] = 0.0
    raw["target"] = {
        "bearing": "unknown",
        "range": "unknown",
        "description": "",
        "free_hand_evidence": "",
        "busy_signals": ["none"],
    }
    raw["safety"] = {"safe_to_approach": False, "stop_reason": "no target visible"}

    decision = policy._normalize_decision(raw, policy.FetchPolicyConfig())

    assert decision["state"] == "search"
    assert decision["action"] == "search"
    assert decision["line"] == ""
    assert decision["simulated_cmd_vel"] == {
        "linear_x": 0.0,
        "angular_z": 0.35,
        "duration_s": 0.8,
    }


def test_gemini_provider_uses_openai_compatible_endpoint(monkeypatch) -> None:
    monkeypatch.setenv("GEMINI_API_KEY", "gemini-key")

    fetch_policy = policy.FetchPolicy(
        policy.FetchPolicyConfig(
            vision_provider="gemini",
        )
    )

    with patch("policy.OpenAI") as openai_cls:
        client = openai_cls.return_value
        client.chat.completions.create.return_value = _completion(
            json.dumps(_raw_decision())
        )

        decision = fetch_policy.analyze_frame(IMAGE_DATA_URL)

    openai_cls.assert_called_once_with(
        api_key="gemini-key",
        base_url=policy.GEMINI_OPENAI_BASE_URL,
        timeout=policy.DEFAULT_REQUEST_TIMEOUT_S,
        max_retries=policy.DEFAULT_MAX_RETRIES,
    )
    call_kwargs = client.chat.completions.create.call_args.kwargs
    assert call_kwargs["model"] == policy.DEFAULT_GEMINI_VISION_MODEL
    assert call_kwargs["reasoning_effort"] == policy.DEFAULT_GEMINI_REASONING_EFFORT
    assert call_kwargs["response_format"] == {"type": "json_object"}
    assert call_kwargs["messages"][0]["content"][0]["type"] == "image_url"
    assert decision["state"] == "greet"
    assert decision["line"]


def test_greet_line_tells_person_to_take_coke_from_back() -> None:
    raw = _raw_decision()
    raw["line"] = "That laptop focus is powerful, but my tiny cooler has entered the chat."

    decision = policy._normalize_decision(raw, policy.FetchPolicyConfig())

    assert decision["state"] == "greet"
    assert "Coke" in decision["line"]
    assert "back" in decision["line"]
    assert "instant photo" in decision["line"]


def test_long_greet_line_keeps_coke_from_back_instruction() -> None:
    raw = _raw_decision()
    raw["line"] = (
        "That bright white shirt and big storytelling stance make you look like "
        "the official master of ceremonies for this entire room, and honestly the "
        "robot dog has never seen more confidence near a kitchen island."
    )

    decision = policy._normalize_decision(raw, policy.FetchPolicyConfig())

    assert decision["state"] == "greet"
    assert "Coke" in decision["line"]
    assert "back" in decision["line"]
    assert "instant photo" in decision["line"]


def test_photo_coaching_line_mentions_coke_and_frame() -> None:
    raw = _raw_decision()
    raw["line"] = "Scoot a little left so the camera can see you."
    raw["photo_ready"] = False

    decision = policy._normalize_decision(
        raw,
        policy.FetchPolicyConfig(),
        interaction_phase="confirm_coke",
    )

    assert decision["state"] == "wait_for_coke"
    assert "Coke" in decision["line"]
    assert "frame" in decision["line"]


def test_photo_coaching_line_keeps_holding_the_coke_wording() -> None:
    raw = _raw_decision()
    raw["line"] = "Hold the Coke in the frame."
    raw["photo_ready"] = False

    decision = policy._normalize_decision(
        raw,
        policy.FetchPolicyConfig(),
        interaction_phase="confirm_coke",
    )

    assert decision["state"] == "wait_for_coke"
    assert decision["line"] == "Hold the Coke in the frame."


def test_photo_ready_line_gives_photographer_cue() -> None:
    raw = _raw_decision()
    raw["line"] = "Perfect."
    raw["photo_ready"] = True
    raw["framing"] = {
        "person_visible": True,
        "coke_visible": True,
        "well_framed": True,
        "notes": "person and can centered",
    }

    decision = policy._normalize_decision(
        raw,
        policy.FetchPolicyConfig(),
        interaction_phase="confirm_coke",
    )

    assert decision["state"] == "photo_ready"
    assert "Coke" in decision["line"]
    assert "Cheers" in decision["line"]


def test_photo_not_ready_without_visible_coke_even_if_framed() -> None:
    raw = _raw_decision()
    raw["line"] = "Perfect."
    raw["photo_ready"] = True
    raw["coke_visible"] = False
    raw["framing"] = {
        "person_visible": True,
        "coke_visible": False,
        "well_framed": True,
        "notes": "person centered but no can visible",
    }

    decision = policy._normalize_decision(
        raw,
        policy.FetchPolicyConfig(),
        interaction_phase="confirm_coke",
    )

    assert decision["state"] == "wait_for_coke"
    assert decision["photo_ready"] is False
    assert decision["action"] == "coach_photo"
    assert "Coke" in decision["line"]
    assert "frame" in decision["line"]


def test_photo_not_ready_when_coke_visible_but_not_framed() -> None:
    raw = _raw_decision()
    raw["line"] = "I can see the Coke."
    raw["photo_ready"] = True
    raw["coke_visible"] = True
    raw["framing"] = {
        "person_visible": True,
        "coke_visible": True,
        "well_framed": False,
        "notes": "can visible but person is cropped",
    }

    decision = policy._normalize_decision(
        raw,
        policy.FetchPolicyConfig(),
        interaction_phase="confirm_coke",
    )

    assert decision["state"] == "wait_for_coke"
    assert decision["photo_ready"] is False
    assert decision["action"] == "coach_photo"
    assert "Coke" in decision["line"]
    assert "frame" in decision["line"]


def test_greet_requires_centered_target() -> None:
    raw = _raw_decision()
    raw["target"] = {
        **raw["target"],  # type: ignore[arg-type]
        "bearing": "left",
        "range": "inside_4m",
    }

    decision = policy._normalize_decision(raw, policy.FetchPolicyConfig())

    assert decision["state"] == "approach"
    assert decision["line"] == ""
    assert decision["simulated_cmd_vel"] == {
        "linear_x": 0.0,
        "angular_z": 0.28,
        "duration_s": 0.45,
    }


def test_photo_not_ready_when_person_on_edge() -> None:
    raw = _raw_decision()
    raw["target"] = {
        **raw["target"],  # type: ignore[arg-type]
        "bearing": "left",
        "range": "inside_4m",
    }
    raw["line"] = "Perfect."
    raw["photo_ready"] = True
    raw["coke_visible"] = True
    raw["framing"] = {
        "person_visible": True,
        "coke_visible": True,
        "well_framed": True,
        "notes": "person is near the edge",
    }

    decision = policy._normalize_decision(
        raw,
        policy.FetchPolicyConfig(),
        interaction_phase="confirm_coke",
    )

    assert decision["state"] == "wait_for_coke"
    assert decision["photo_ready"] is False
    assert decision["action"] == "coach_photo"
    assert "Coke" in decision["line"]
    assert decision["simulated_cmd_vel"] == {
        "linear_x": 0.0,
        "angular_z": 0.28,
        "duration_s": 0.45,
    }


def test_photo_not_ready_overrides_raw_take_photo_action() -> None:
    raw = _raw_decision()
    raw["target"] = {
        **raw["target"],  # type: ignore[arg-type]
        "bearing": "right",
        "range": "inside_4m",
    }
    raw["action"] = "take_photo_dance"
    raw["photo_ready"] = True
    raw["coke_visible"] = True
    raw["framing"] = {
        "person_visible": True,
        "coke_visible": True,
        "well_framed": True,
        "notes": "subject is on the right edge",
    }

    decision = policy._normalize_decision(
        raw,
        policy.FetchPolicyConfig(),
        interaction_phase="confirm_coke",
    )

    assert decision["state"] == "wait_for_coke"
    assert decision["action"] == "coach_photo"
    assert decision["photo_ready"] is False
    assert decision["simulated_cmd_vel"] == {
        "linear_x": 0.0,
        "angular_z": -0.28,
        "duration_s": 0.45,
    }


def test_gemini_retries_without_json_mode_if_unsupported(monkeypatch) -> None:
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    monkeypatch.setenv("GOOGLE_API_KEY", "google-key")

    fetch_policy = policy.FetchPolicy(
        policy.FetchPolicyConfig(
            vision_provider="gemini",
        )
    )

    with patch("policy.OpenAI") as openai_cls:
        client = openai_cls.return_value
        client.chat.completions.create.side_effect = [
            RuntimeError("response_format json_object is not supported"),
            _completion(json.dumps(_raw_decision())),
        ]

        decision = fetch_policy.analyze_frame(IMAGE_DATA_URL)

    calls = client.chat.completions.create.call_args_list
    openai_cls.assert_called_once_with(
        api_key="google-key",
        base_url=policy.GEMINI_OPENAI_BASE_URL,
        timeout=policy.DEFAULT_REQUEST_TIMEOUT_S,
        max_retries=policy.DEFAULT_MAX_RETRIES,
    )
    assert len(calls) == 2
    assert calls[0].kwargs["response_format"] == {"type": "json_object"}
    assert calls[0].kwargs["reasoning_effort"] == policy.DEFAULT_GEMINI_REASONING_EFFORT
    assert "response_format" not in calls[1].kwargs
    assert calls[1].kwargs["reasoning_effort"] == policy.DEFAULT_GEMINI_REASONING_EFFORT
    assert decision["state"] == "greet"


def test_openai_provider_uses_openai_key(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "openai-key")

    fetch_policy = policy.FetchPolicy(
        policy.FetchPolicyConfig(model="gpt-5-mini", vision_provider="openai")
    )

    with patch("policy.OpenAI") as openai_cls:
        client = openai_cls.return_value
        client.chat.completions.create.return_value = _completion(
            json.dumps(_raw_decision())
        )

        decision = fetch_policy.analyze_frame(IMAGE_DATA_URL)

    openai_cls.assert_called_once_with(
        api_key="openai-key",
        timeout=policy.DEFAULT_REQUEST_TIMEOUT_S,
        max_retries=policy.DEFAULT_MAX_RETRIES,
    )
    assert decision["state"] == "greet"


def test_client_cache_includes_model(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "openai-key")

    fetch_policy = policy.FetchPolicy(
        policy.FetchPolicyConfig(model="gpt-5-mini", vision_provider="openai")
    )

    with patch("policy.OpenAI") as openai_cls:
        first_client = openai_cls.return_value
        first_client.chat.completions.create.return_value = _completion(
            json.dumps(_raw_decision())
        )

        fetch_policy.analyze_frame(IMAGE_DATA_URL)
        fetch_policy.config = policy.FetchPolicyConfig(
            model="gpt-5", vision_provider="openai"
        )
        fetch_policy.analyze_frame(IMAGE_DATA_URL)

    assert openai_cls.call_count == 2
