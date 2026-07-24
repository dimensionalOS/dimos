# Copyright 2026 Dimensional Inc.
import pytest

from dimos.benchmark.spatial.models import AnswerType
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory
from dimos.benchmark.spatial.pi_baseline.transaction import AnswerTransaction


def test_transaction_accepts_first_valid_value_and_returns_receipts_only() -> None:
    transaction = AnswerTransaction("instance-1", AnswerType.BOOLEAN)
    invalid = transaction.submit(1)
    accepted = transaction.submit(True)
    late = transaction.submit(False)
    assert not invalid.accepted
    assert accepted.accepted and not late.accepted
    assert accepted.instance_id == "instance-1"
    assert transaction.prediction is not None and transaction.prediction.value is True


def test_durable_prediction_file_wins_repeated_submission(tmp_path) -> None:
    path = tmp_path / "private" / "prediction.json"
    first = AnswerTransaction("instance-1", AnswerType.BOOLEAN, path)
    assert first.submit(True).accepted
    assert path.read_bytes()
    second = AnswerTransaction("instance-1", AnswerType.BOOLEAN, path)
    assert not second.submit(False).accepted
    assert second.prediction is not None and second.prediction.value is True


def test_prediction_symlink_in_pinned_private_leaf_fails_closed(tmp_path) -> None:
    private_path = tmp_path / "private"
    private_path.mkdir()
    escaped = tmp_path / "escaped-prediction.json"
    escaped.write_bytes(b"must remain unchanged")
    private = PinnedDirectory.open(private_path)
    try:
        (private_path / "prediction.provisional.v1.json").symlink_to(escaped)

        with pytest.raises(OSError):
            AnswerTransaction("instance-1", AnswerType.BOOLEAN, private)

        assert escaped.read_bytes() == b"must remain unchanged"
        assert (private_path / "prediction.provisional.v1.json").is_symlink()
    finally:
        private.close()
