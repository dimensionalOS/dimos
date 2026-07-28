from __future__ import annotations

import base64
from pathlib import Path
from threading import Event
import time

import numpy as np
import pytest

from dimos.perception.target_verification import (
    CandidateEvidenceBundle,
    CandidateView,
    PersistedCandidateLoader,
    TargetVerification,
    VerificationView,
)
from dimos.perception.vision_gateway import (
    CandidateVisionGateway,
    VisionGatewayBusyError,
    VisionGatewayConflictError,
)
from dimos.perception.visual_memory import VisualMemory


def _view(frame_id: str, content: bytes | None = None) -> CandidateView:
    return CandidateView(
        frame_id=frame_id,
        jpeg_base64=base64.b64encode(content or f"jpeg-{frame_id}".encode()).decode(),
        width=640,
        height=480,
    )


def _bundle(
    candidate_id: str = "candidate-door-1",
    *,
    target: str = "a real indoor doorway",
) -> CandidateEvidenceBundle:
    return CandidateEvidenceBundle(
        candidate_id=candidate_id,
        target_description=target,
        views=[_view("frame-a"), _view("frame-b")],
    )


def _yes() -> TargetVerification:
    return TargetVerification(
        verdict="yes",
        target_type="doorway",
        confidence=0.93,
        passable=True,
        need_more_views=False,
        reason="Two selected views show the same framed opening.",
        views=[
            VerificationView(
                frame_id=frame_id,
                verdict="yes",
                bbox=(0.2, 0.1, 0.8, 0.95),
            )
            for frame_id in ("frame-a", "frame-b")
        ],
    )


class _ImmediateVerifier:
    def __init__(self, result: TargetVerification | BaseException) -> None:
        self.result = result
        self.received: list[CandidateEvidenceBundle] = []

    def verify(self, bundle: CandidateEvidenceBundle) -> TargetVerification:
        self.received.append(bundle)
        if isinstance(self.result, BaseException):
            raise self.result
        return self.result


class _BlockingVerifier:
    def __init__(self, result: TargetVerification) -> None:
        self.result = result
        self.started = Event()
        self.release = Event()
        self.finished = Event()

    def verify(self, bundle: CandidateEvidenceBundle) -> TargetVerification:
        del bundle
        self.started.set()
        self.release.wait(timeout=2.0)
        self.finished.set()
        return self.result


class _Clock:
    def __init__(self) -> None:
        self.value = 100.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def test_submit_is_non_blocking_and_result_has_typed_provider_metadata() -> None:
    verifier = _ImmediateVerifier(_yes())
    gateway = CandidateVisionGateway(
        verifier,
        provider="fake-provider",
        model="fake-vision-v1",
        timeout_s=2.0,
    )

    submitted = gateway.submit(_bundle())
    completed = gateway.wait(submitted.job_id, timeout_s=1.0)

    assert submitted.state == "pending"
    assert submitted.verification is None
    assert "jpeg_base64" not in submitted.model_dump_json()
    assert completed.state == "completed"
    assert completed.verification is not None
    assert completed.verification.verdict == "yes"
    assert completed.verification.confidence == 0.93
    assert completed.verification.views[0].bbox == (0.2, 0.1, 0.8, 0.95)
    assert completed.provider.provider == "fake-provider"
    assert completed.provider.model == "fake-vision-v1"
    assert completed.provider.selected_frame_ids == ["frame-a", "frame-b"]
    assert completed.provider.gateway_invocations == 1
    assert completed.provider.error_code is None
    assert "jpeg_base64" not in completed.model_dump_json()


def test_provider_deadline_does_not_block_submit_and_late_result_is_ignored() -> None:
    verifier = _BlockingVerifier(_yes())
    clock = _Clock()
    gateway = CandidateVisionGateway(
        verifier,
        provider="slow-provider",
        model="slow-model",
        timeout_s=2.0,
        monotonic_clock=clock,
    )

    started_at = time.monotonic()
    submitted = gateway.submit(_bundle())
    submit_elapsed = time.monotonic() - started_at

    assert submit_elapsed < 0.1
    assert verifier.started.wait(timeout=1.0)

    clock.advance(2.1)
    timed_out = gateway.get(submitted.job_id)
    assert timed_out.state == "completed"
    assert timed_out.verification is not None
    assert timed_out.verification.verdict == "uncertain"
    assert timed_out.provider.error_code == "provider_timeout"
    with pytest.raises(VisionGatewayBusyError):
        gateway.submit(_bundle("candidate-door-2"))

    verifier.release.set()
    assert verifier.finished.wait(timeout=1.0)
    still_timed_out = gateway.get(submitted.job_id)
    assert still_timed_out.provider.error_code == "provider_timeout"
    assert still_timed_out.verification == timed_out.verification

    deadline = time.monotonic() + 1.0
    while True:
        try:
            next_job = gateway.submit(_bundle("candidate-door-2"))
            break
        except VisionGatewayBusyError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.01)
    assert next_job.candidate_id == "candidate-door-2"


def test_candidate_retry_is_idempotent_but_conflict_and_busy_fail_closed() -> None:
    verifier = _BlockingVerifier(_yes())
    gateway = CandidateVisionGateway(
        verifier,
        provider="fake",
        model="fake",
        timeout_s=2.0,
        max_inflight=1,
    )
    bundle = _bundle()

    first = gateway.submit(bundle)
    repeated = gateway.submit(bundle)

    assert first.job_id == repeated.job_id
    with pytest.raises(VisionGatewayConflictError):
        gateway.submit(_bundle(target="a whiteboard"))
    with pytest.raises(VisionGatewayBusyError):
        gateway.submit(_bundle("candidate-door-2"))

    verifier.release.set()
    assert gateway.wait(first.job_id, timeout_s=1.0).state == "completed"


def test_provider_exception_becomes_sanitized_uncertain_result() -> None:
    gateway = CandidateVisionGateway(
        _ImmediateVerifier(RuntimeError("secret provider response")),
        provider="fake",
        model="fake",
        timeout_s=2.0,
    )

    submitted = gateway.submit(_bundle())
    completed = gateway.wait(submitted.job_id, timeout_s=1.0)

    assert completed.verification is not None
    assert completed.verification.verdict == "uncertain"
    assert completed.provider.error_code == "provider_exception"
    serialized = completed.model_dump_json()
    assert "RuntimeError" in serialized
    assert "secret provider response" not in serialized


def _memory_path(tmp_path: Path) -> Path:
    memory = VisualMemory(output_dir=str(tmp_path))
    memory.add("selected-a", np.full((480, 640, 3), 32, dtype=np.uint8))
    memory.add("selected-b", np.full((720, 1280, 3), 64, dtype=np.uint8))
    memory.add("not-selected", np.full((100, 100, 3), 255, dtype=np.uint8))
    return Path(memory.save())


def test_saved_frame_replay_sends_only_selected_views_and_stores_no_images(
    tmp_path: Path,
) -> None:
    bundle = PersistedCandidateLoader(_memory_path(tmp_path)).load(
        candidate_id="candidate-replay-1",
        target_description="an indoor doorway",
        frame_ids=["selected-a", "selected-b"],
    )
    result = TargetVerification(
        verdict="no",
        target_type="whiteboard",
        confidence=0.88,
        passable=False,
        need_more_views=False,
        reason="The selected views show a whiteboard.",
        views=[],
    )
    verifier = _ImmediateVerifier(result)
    gateway = CandidateVisionGateway(
        verifier,
        provider="replay",
        model="fixture",
        timeout_s=2.0,
    )

    submitted = gateway.submit(bundle)
    completed = gateway.wait(submitted.job_id, timeout_s=1.0)

    assert [view.frame_id for view in verifier.received[0].views] == [
        "selected-a",
        "selected-b",
    ]
    assert completed.provider.selected_frame_ids == ["selected-a", "selected-b"]
    assert completed.verification is not None
    assert completed.verification.verdict == "no"
    serialized = completed.model_dump_json()
    assert "not-selected" not in serialized
    assert "jpeg_base64" not in serialized
    assert "data:image" not in serialized
