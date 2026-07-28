from __future__ import annotations

import base64
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from openai import OpenAIError
from pydantic import ValidationError
import pytest

from dimos.perception.target_verification import (
    CandidateEvidenceBundle,
    CandidateView,
    OpenAIResponsesVisionVerifier,
    PersistedCandidateLoader,
    TargetVerification,
    VerificationView,
    apply_verification_policy,
)
from dimos.perception.visual_memory import VisualMemory


def _candidate_view(frame_id: str) -> CandidateView:
    return CandidateView(
        frame_id=frame_id,
        jpeg_base64=base64.b64encode(f"jpeg-{frame_id}".encode()).decode(),
        width=640,
        height=480,
    )


def _bundle(*frame_ids: str) -> CandidateEvidenceBundle:
    return CandidateEvidenceBundle(
        candidate_id="candidate-door-1",
        target_description="a real passable indoor door or doorway",
        views=[_candidate_view(frame_id) for frame_id in frame_ids],
    )


def _yes_result(*frame_ids: str) -> TargetVerification:
    return TargetVerification(
        verdict="yes",
        target_type="doorway",
        confidence=0.91,
        passable=True,
        need_more_views=False,
        reason="Multiple views show a framed opening with visible space beyond it.",
        views=[
            VerificationView(
                frame_id=frame_id,
                verdict="yes",
                bbox=(0.18, 0.12, 0.73, 0.94),
            )
            for frame_id in frame_ids
        ],
    )


def test_candidate_bundle_requires_one_to_three_unique_views() -> None:
    with pytest.raises(ValidationError):
        CandidateEvidenceBundle(
            candidate_id="candidate",
            target_description="door",
            views=[],
        )

    with pytest.raises(ValidationError):
        _bundle("f1", "f2", "f3", "f4")

    with pytest.raises(ValidationError, match="frame IDs must be unique"):
        _bundle("f1", "f1")


def test_verification_bbox_must_be_normalized_and_ordered() -> None:
    with pytest.raises(ValidationError):
        VerificationView(
            frame_id="f1",
            verdict="yes",
            bbox=(-0.1, 0.1, 0.8, 0.9),
        )

    with pytest.raises(ValidationError, match="ordered"):
        VerificationView(
            frame_id="f1",
            verdict="yes",
            bbox=(0.8, 0.1, 0.2, 0.9),
        )


def test_policy_requires_two_known_yes_views_with_boxes() -> None:
    bundle = _bundle("f1", "f2", "f3")

    accepted = apply_verification_policy(bundle, _yes_result("f1", "f2"))
    insufficient = apply_verification_policy(bundle, _yes_result("f1"))
    unknown = apply_verification_policy(bundle, _yes_result("f1", "not-in-bundle"))

    assert accepted.verdict == "yes"
    assert insufficient.verdict == "uncertain"
    assert insufficient.need_more_views is True
    assert unknown.verdict == "uncertain"
    assert "unknown frame" in unknown.reason


def test_policy_never_upgrades_no_or_uncertain() -> None:
    bundle = _bundle("f1", "f2")
    for verdict in ("no", "uncertain"):
        result = TargetVerification(
            verdict=verdict,
            target_type="doorway",
            confidence=0.2,
            passable=False,
            need_more_views=verdict == "uncertain",
            reason="Target not confirmed.",
            views=[],
        )

        assert apply_verification_policy(bundle, result).verdict == verdict


def _write_visual_memory(tmp_path: Path) -> Path:
    memory = VisualMemory(output_dir=str(tmp_path))
    memory.add("wide", np.full((1200, 1600, 3), 64, dtype=np.uint8))
    memory.add("small", np.full((240, 320, 3), 128, dtype=np.uint8))
    memory.add("unused", np.full((100, 100, 3), 255, dtype=np.uint8))
    return Path(memory.save())


def test_persisted_loader_selects_and_resizes_only_requested_frames(
    tmp_path: Path,
) -> None:
    memory_path = _write_visual_memory(tmp_path)

    bundle = PersistedCandidateLoader(memory_path).load(
        candidate_id="candidate",
        target_description="doorway",
        frame_ids=["wide", "small"],
    )

    assert [view.frame_id for view in bundle.views] == ["wide", "small"]
    assert all(view.mime_type == "image/jpeg" for view in bundle.views)
    assert max(bundle.views[0].width, bundle.views[0].height) == 1024
    assert max(bundle.views[1].width, bundle.views[1].height) == 320
    decoded = cv2.imdecode(
        np.frombuffer(base64.b64decode(bundle.views[0].jpeg_base64), dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    assert decoded is not None
    assert decoded.shape[:2] == (bundle.views[0].height, bundle.views[0].width)
    assert "unused" not in [view.frame_id for view in bundle.views]


def test_persisted_loader_rejects_missing_frame(tmp_path: Path) -> None:
    memory_path = _write_visual_memory(tmp_path)

    with pytest.raises(ValueError, match="missing"):
        PersistedCandidateLoader(memory_path).load(
            candidate_id="candidate",
            target_description="doorway",
            frame_ids=["missing"],
        )


class _FakeResponses:
    def __init__(self, *outcomes: object) -> None:
        self.outcomes = list(outcomes)
        self.calls: list[dict[str, object]] = []

    def parse(self, **kwargs: object) -> SimpleNamespace:
        self.calls.append(kwargs)
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, BaseException):
            raise outcome
        return SimpleNamespace(output_parsed=outcome)


class _FakeClient:
    def __init__(self, *outcomes: object) -> None:
        self.responses = _FakeResponses(*outcomes)


def test_openai_verifier_sends_only_selected_images_with_structured_output() -> None:
    bundle = _bundle("f1", "f2")
    client = _FakeClient(_yes_result("f1", "f2"))
    verifier = OpenAIResponsesVisionVerifier(
        client=client,
        model="test-vision-model",
        timeout_s=4.0,
    )

    result = verifier.verify(bundle)

    assert result.verdict == "yes"
    assert len(client.responses.calls) == 1
    call = client.responses.calls[0]
    assert call["model"] == "test-vision-model"
    assert call["store"] is False
    assert call["text_format"] is TargetVerification
    assert call["timeout"] == 4.0
    content = call["input"][0]["content"]  # type: ignore[index]
    images = [item for item in content if item["type"] == "input_image"]
    assert len(images) == 2
    assert all(item["image_url"].startswith("data:image/jpeg;base64,") for item in images)
    assert all("unused" not in item["image_url"] for item in images)


def test_openai_verifier_retries_once_then_fails_closed() -> None:
    bundle = _bundle("f1", "f2")
    recovered_client = _FakeClient(TimeoutError("slow"), _yes_result("f1", "f2"))
    recovered = OpenAIResponsesVisionVerifier(
        client=recovered_client,
        max_retries=1,
    ).verify(bundle)

    failed_client = _FakeClient(TimeoutError("slow"), TimeoutError("still slow"))
    failed = OpenAIResponsesVisionVerifier(
        client=failed_client,
        max_retries=1,
    ).verify(bundle)

    assert recovered.verdict == "yes"
    assert len(recovered_client.responses.calls) == 2
    assert failed.verdict == "uncertain"
    assert failed.need_more_views is True
    assert "provider error" in failed.reason
    assert len(failed_client.responses.calls) == 2


def test_openai_verifier_fails_closed_when_response_is_unparsed() -> None:
    result = OpenAIResponsesVisionVerifier(
        client=_FakeClient(None),
        max_retries=0,
    ).verify(_bundle("f1", "f2"))

    assert result.verdict == "uncertain"
    assert "structured result" in result.reason


def test_openai_verifier_fails_closed_on_non_retryable_provider_error() -> None:
    client = _FakeClient(OpenAIError("bad request"))

    result = OpenAIResponsesVisionVerifier(
        client=client,
        max_retries=1,
    ).verify(_bundle("f1", "f2"))

    assert result.verdict == "uncertain"
    assert result.need_more_views is True
    assert "OpenAIError" in result.reason
    assert len(client.responses.calls) == 1


def test_openai_verifier_reports_missing_api_key_without_raising(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)

    result = OpenAIResponsesVisionVerifier().verify(_bundle("f1", "f2"))

    assert result.verdict == "uncertain"
    assert result.need_more_views is True
    assert "OPENAI_API_KEY is not configured" in result.reason
