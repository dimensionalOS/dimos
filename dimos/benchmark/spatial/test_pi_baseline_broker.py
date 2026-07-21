# Copyright 2026 Dimensional Inc.
import base64
from io import BytesIO
from pathlib import Path
import subprocess

from PIL import Image
import pytest

from dimos.benchmark.spatial.models import AnswerType
from dimos.benchmark.spatial.pi_baseline.broker import (
    IMAGE_READ_RETRY_ERROR,
    POST_IMAGE_POLICY_ERROR,
    PRE_IMAGE_POLICY_ERROR,
    BrokerLimits,
    CaseBroker,
    ImageReadRetryError,
    PostImagePolicyViolationError,
    PreImagePolicyViolationError,
)
from dimos.benchmark.spatial.pi_baseline.transaction import AnswerTransaction


class _Case:
    class _Request:
        def __init__(self, workspace_dir: Path) -> None:
            self.workspace_dir = workspace_dir

    def __init__(self, root: Path) -> None:
        self.request = self._Request(root)

    def exec(
        self, command: str, *, controller_deadline: float | None = None
    ) -> subprocess.CompletedProcess[str]:
        del controller_deadline
        return subprocess.CompletedProcess(command, 7, "out-" + command, "err")


def _png(width: int = 2, height: int = 3) -> bytes:
    output = BytesIO()
    Image.new("RGBA", (width, height)).save(output, format="PNG")
    return output.getvalue()


def test_tools_are_bounded_and_submission_is_immutable(tmp_path: Path) -> None:
    (tmp_path / "ok.png").write_bytes(_png())
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
        BrokerLimits(max_output_bytes=3),
    )
    assert broker.sandbox_exec("printf") == {"stdout": "out", "stderr": "err", "exit_code": 7}
    image = broker.read_generated_image("ok.png")
    assert base64.b64decode(image["data"]) == _png()
    broker.commit_image_read(delivered=True)
    assert broker.submit_answer(True).accepted
    assert not broker.submit_answer(False).accepted
    assert len(broker.audit) == 4


def test_dispatch_routes_valid_pre_image_tools_before_barrier(tmp_path: Path) -> None:
    (tmp_path / "ok.png").write_bytes(_png())
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    assert broker.dispatch("sandbox_exec", {"command": "printf"})["exit_code"] == 7
    image = broker.dispatch("read_generated_image", {"path": "ok.png"})
    assert image["mime"] == "image/png"


def test_encouraged_state_table_two_sandboxes_then_image_then_submit(tmp_path: Path) -> None:
    (tmp_path / "ok.png").write_bytes(_png())
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    broker.dispatch("sandbox_exec", {"command": "one"})
    broker.dispatch("sandbox_exec", {"command": "two"})
    with pytest.raises(PreImagePolicyViolationError, match=PRE_IMAGE_POLICY_ERROR):
        broker.dispatch("sandbox_exec", {"command": "three"})
    with pytest.raises(PreImagePolicyViolationError, match=PRE_IMAGE_POLICY_ERROR):
        broker.dispatch("submit_answer", {"answer": True})
    image = broker.dispatch("read_generated_image", {"path": "ok.png"})
    assert image["mime"] == "image/png"
    broker.commit_image_read(delivered=True)
    assert broker.dispatch("submit_answer", {"answer": True}).accepted


def test_encouraged_sandbox_failure_consumes_attempt_and_failed_image_is_terminal(
    tmp_path: Path,
) -> None:
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    with pytest.raises(ValueError):
        broker.dispatch("sandbox_exec", {})
    broker.dispatch("sandbox_exec", {"command": "two"})
    with pytest.raises(ImageReadRetryError, match=IMAGE_READ_RETRY_ERROR):
        broker.dispatch("read_generated_image", {"path": "missing.png"})
    with pytest.raises(PreImagePolicyViolationError, match=PRE_IMAGE_POLICY_ERROR):
        broker.dispatch("read_generated_image", {"path": "missing-again.png"})


def test_image_failure_category_is_allowlisted_and_redacted(tmp_path: Path) -> None:
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    with pytest.raises(ImageReadRetryError) as first:
        broker.dispatch("read_generated_image", {"path": "missing.png"})
    assert first.value.category == "artifact_not_found_or_unsafe"
    with pytest.raises(PreImagePolicyViolationError) as second:
        broker.dispatch("read_generated_image", {"path": "missing-again.png"})
    telemetry = broker.policy_telemetry(second.value.failure_kind, second.value.category)
    rendered = telemetry.model_dump(mode="json")
    assert rendered["image_failure_category"] == "artifact_not_found_or_unsafe"
    assert all(
        key not in repr(rendered) for key in ("path", "command", "exception", "bytes", "dimensions")
    )


def test_image_path_and_limits_reject_traversal_symlinks_and_dimensions(tmp_path: Path) -> None:
    (tmp_path / "ok.png").write_bytes(_png(10, 10))
    (tmp_path / "large.png").write_bytes(_png(100, 100))
    (tmp_path / "link.png").symlink_to(tmp_path / "ok.png")
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
        BrokerLimits(max_image_pixels=100),
    )
    for path in ("/work/ok.png", "/tmp/ok.png", "../ok.png", "link.png"):
        with pytest.raises(ValueError):
            broker.read_generated_image(path)
    with pytest.raises(ValueError):
        broker.read_generated_image("large.png")


def test_unknown_tool_is_rejected() -> None:
    broker = CaseBroker(
        "case",
        _Case(Path.cwd()),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )
    with pytest.raises(ValueError, match="unknown tool"):
        broker.dispatch("unknown", {})


def test_exact_arguments_and_utf8_command_bound_are_enforced(tmp_path: Path) -> None:
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )
    with pytest.raises(ValueError, match="exact schema"):
        broker.dispatch("sandbox_exec", {"command": "true", "extra": True})
    with pytest.raises(ValueError):
        broker.sandbox_exec("é" * 2049)


def test_encouraged_submission_is_blocked_without_successful_image_read(tmp_path: Path) -> None:
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    with pytest.raises(ValueError, match="visualization_required_before_submission"):
        broker.submit_answer(True)
    assert broker.transaction.prediction is None
    assert broker.audit[-1]["outcome"] == "policy_violation"


def test_forbidden_image_attempt_is_rejected_and_marks_run_noncompliant(tmp_path: Path) -> None:
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )
    with pytest.raises(ValueError, match="visualization_forbidden"):
        broker.read_generated_image("image.png")
    assert not broker.compliant
    assert broker.audit[-1]["outcome"] == "policy_violation"


def test_image_must_decode_and_delivery_must_be_committed_before_unlocking(tmp_path: Path) -> None:
    (tmp_path / "truncated.png").write_bytes(_png()[:-4])
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    with pytest.raises(ValueError, match="fully decodable"):
        broker.read_generated_image("truncated.png")
    (tmp_path / "ok.png").write_bytes(_png())
    broker.read_generated_image("ok.png")
    broker.commit_image_read(delivered=False)
    with pytest.raises(ValueError, match="visualization_required_before_submission"):
        broker.submit_answer(True)


def test_oversized_image_is_rejected_without_unlocking(tmp_path: Path) -> None:
    (tmp_path / "large.png").write_bytes(_png(20, 20))
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
        BrokerLimits(max_image_pixels=10),
    )
    with pytest.raises(ValueError, match="dimensions"):
        broker.read_generated_image("large.png")
    with pytest.raises(ValueError, match="visualization_required_before_submission"):
        broker.submit_answer(True)


def test_encouraged_image_requires_one_submission_and_then_closes_policy_phase(
    tmp_path: Path,
) -> None:
    (tmp_path / "ok.png").write_bytes(_png())

    class CountingCase(_Case):
        def exec(
            self, command: str, *, controller_deadline: float | None = None
        ) -> subprocess.CompletedProcess[str]:
            del controller_deadline
            raise AssertionError("sandbox_exec must not execute after image read")

    broker = CaseBroker(
        "case",
        CountingCase(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    broker.read_generated_image("ok.png")
    broker.commit_image_read(delivered=True)
    with pytest.raises(PostImagePolicyViolationError, match=POST_IMAGE_POLICY_ERROR):
        broker.dispatch("sandbox_exec", {"command": "true"})
    with pytest.raises(PostImagePolicyViolationError, match=POST_IMAGE_POLICY_ERROR):
        broker.dispatch("read_generated_image", {"path": "missing.png"})
    assert broker.dispatch("submit_answer", {"answer": True}).accepted
    with pytest.raises(PostImagePolicyViolationError, match=POST_IMAGE_POLICY_ERROR):
        broker.dispatch("submit_answer", {"answer": False})


def test_invalid_submission_consumes_the_one_post_image_attempt(tmp_path: Path) -> None:
    (tmp_path / "ok.png").write_bytes(_png())
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    broker.read_generated_image("ok.png")
    broker.commit_image_read(delivered=True)
    with pytest.raises(PostImagePolicyViolationError, match=POST_IMAGE_POLICY_ERROR):
        broker.dispatch("submit_answer", {"answer": 1})
    with pytest.raises(PostImagePolicyViolationError, match=POST_IMAGE_POLICY_ERROR):
        broker.dispatch("submit_answer", {"answer": True})
    assert broker.transaction.prediction is None


@pytest.mark.parametrize("arguments", [{}, {"answer": True, "extra": 1}, {"answer": "true"}])
def test_malformed_post_image_submission_is_terminal_and_never_retries(
    tmp_path: Path, arguments: dict[str, object]
) -> None:
    (tmp_path / "ok.png").write_bytes(_png())
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    broker.read_generated_image("ok.png")
    broker.commit_image_read(delivered=True)
    with pytest.raises(PostImagePolicyViolationError, match=POST_IMAGE_POLICY_ERROR):
        broker.dispatch("submit_answer", arguments)
    with pytest.raises(PostImagePolicyViolationError, match=POST_IMAGE_POLICY_ERROR):
        broker.dispatch("submit_answer", {"answer": True})
    assert broker.transaction.prediction is None
    assert broker.audit[-1]["outcome"] == POST_IMAGE_POLICY_ERROR
