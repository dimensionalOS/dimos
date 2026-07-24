from __future__ import annotations

from io import StringIO
import json
from pathlib import Path

import numpy as np

from dimos.perception.offline_target_verification import run
from dimos.perception.target_verification import (
    CandidateEvidenceBundle,
    TargetVerification,
)
from dimos.perception.visual_memory import VisualMemory


def _memory_path(tmp_path: Path) -> Path:
    memory = VisualMemory(output_dir=str(tmp_path))
    memory.add("frame-a", np.full((480, 640, 3), 64, dtype=np.uint8))
    memory.add("frame-b", np.full((720, 1280, 3), 128, dtype=np.uint8))
    memory.add("never-selected", np.full((100, 100, 3), 255, dtype=np.uint8))
    return Path(memory.save())


def _arguments(memory_path: Path) -> list[str]:
    return [
        "--memory",
        str(memory_path),
        "--candidate-id",
        "candidate-1",
        "--target",
        "a real indoor doorway",
        "--frame",
        "frame-a",
        "--frame",
        "frame-b",
    ]


def test_cli_defaults_to_local_dry_run_without_constructing_verifier(
    tmp_path: Path,
) -> None:
    stdout = StringIO()
    stderr = StringIO()

    def forbidden_factory(_model: str) -> object:
        raise AssertionError("dry run must not construct an OpenAI verifier")

    status = run(
        _arguments(_memory_path(tmp_path)),
        verifier_factory=forbidden_factory,
        stdout=stdout,
        stderr=stderr,
    )
    payload = json.loads(stdout.getvalue())

    assert status == 0
    assert stderr.getvalue() == ""
    assert payload["mode"] == "dry_run"
    assert payload["upload_authorized"] is False
    assert [frame["frame_id"] for frame in payload["selected_frames"]] == [
        "frame-a",
        "frame-b",
    ]
    assert "never-selected" not in stdout.getvalue()
    assert "jpeg_base64" not in stdout.getvalue()
    assert "data:image" not in stdout.getvalue()


def test_cli_send_uses_selected_bundle_and_prints_sanitized_result(
    tmp_path: Path,
) -> None:
    stdout = StringIO()
    received: list[CandidateEvidenceBundle] = []

    class FakeVerifier:
        def verify(self, bundle: CandidateEvidenceBundle) -> TargetVerification:
            received.append(bundle)
            return TargetVerification(
                verdict="no",
                target_type="doorway",
                confidence=0.12,
                passable=False,
                need_more_views=False,
                reason="The views show a whiteboard, not a doorway.",
                views=[],
            )

    status = run(
        [*_arguments(_memory_path(tmp_path)), "--send", "--model", "test-model"],
        verifier_factory=lambda model: FakeVerifier()
        if model == "test-model"
        else (_ for _ in ()).throw(AssertionError("wrong model")),
        stdout=stdout,
        stderr=StringIO(),
    )
    payload = json.loads(stdout.getvalue())

    assert status == 0
    assert len(received) == 1
    assert [view.frame_id for view in received[0].views] == ["frame-a", "frame-b"]
    assert payload["mode"] == "send_requested"
    assert payload["upload_authorized"] is True
    assert payload["verification"]["verdict"] == "no"
    assert "jpeg_base64" not in stdout.getvalue()


def test_cli_output_file_never_contains_image_bytes(tmp_path: Path) -> None:
    output_path = tmp_path / "result.json"
    status = run(
        [*_arguments(_memory_path(tmp_path)), "--output", str(output_path)],
        verifier_factory=lambda _model: (_ for _ in ()).throw(
            AssertionError("dry run must not construct verifier")
        ),
        stdout=StringIO(),
        stderr=StringIO(),
    )

    written = output_path.read_text()
    assert status == 0
    assert json.loads(written)["mode"] == "dry_run"
    assert "jpeg_base64" not in written
    assert "data:image" not in written


def test_cli_returns_nonzero_for_missing_memory_or_frame(tmp_path: Path) -> None:
    for arguments, expected in (
        (
            _arguments(tmp_path / "missing.pkl"),
            "visual memory file not found",
        ),
        (
            [
                *_arguments(_memory_path(tmp_path)),
                "--frame",
                "missing-frame",
            ],
            "candidate frame is missing",
        ),
    ):
        stderr = StringIO()
        status = run(
            arguments,
            stdout=StringIO(),
            stderr=stderr,
        )
        assert status == 2
        assert expected in stderr.getvalue()
