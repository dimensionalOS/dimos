# Copyright 2026 Dimensional Inc.
"""Focused tests for evidence-manifest commit authority."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import stat
from types import SimpleNamespace
from typing import Any, cast

import pytest

from dimos.benchmark.spatial import utilities
from dimos.benchmark.spatial.models import AnswerType
import dimos.benchmark.spatial.pi_baseline.evidence as evidence_module
from dimos.benchmark.spatial.pi_baseline.evidence import (
    EvidenceArtifact,
    EvidenceIdentityContext,
    EvidenceManifest,
    EvidencePublishDisposition,
    load_committed_result,
    publish_evidence_manifest_noreplace,
    write_evidence_manifest,
)
from dimos.benchmark.spatial.pi_baseline.records import Prediction
from dimos.benchmark.spatial.pi_baseline.scoring import PrivateScore
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory


def _artifact(path: str, data: bytes) -> EvidenceArtifact:
    return EvidenceArtifact(
        path=path, size_bytes=len(data), sha256=hashlib.sha256(data).hexdigest()
    )


def _bundle(tmp_path: Path) -> tuple[PinnedDirectory, PinnedDirectory, EvidenceManifest]:
    public_path = tmp_path / "public"
    private_path = tmp_path / "private"
    public_path.mkdir()
    private_path.mkdir()
    public = PinnedDirectory.open(public_path)
    private = PinnedDirectory.open(private_path)
    public.write_bytes("case.v1.json", b"public")
    prediction = Prediction.typed("instance", AnswerType.BOOLEAN, True)
    score = PrivateScore(
        instance_id="instance",
        answer_type=AnswerType.BOOLEAN,
        value=True,
        run_id="run",
        case_id="instance",
        mode="mode",
        release_id="release",
        scorer_revision="revision",
        outcome="correct",
        scored_at=datetime(2026, 1, 1, tzinfo=timezone.utc),
    )
    private.write_bytes("prediction.v1.json", prediction.model_dump_json().encode())
    private.write_bytes("score.v1.json", score.model_dump_json().encode())
    manifest = EvidenceManifest(
        public=(_artifact("case.v1.json", b"public"),),
        private=(
            _artifact("prediction.v1.json", prediction.model_dump_json().encode()),
            _artifact("score.v1.json", score.model_dump_json().encode()),
        ),
    )
    return public, private, manifest


def _identity() -> EvidenceIdentityContext:
    return EvidenceIdentityContext("run", "instance", "mode", "release", "revision")


def _close(public: PinnedDirectory, private: PinnedDirectory) -> None:
    public.close()
    private.close()


def _manifest_with(manifest: EvidenceManifest, **changes: object) -> EvidenceManifest:
    return manifest.model_copy(update=changes)


def _replace_private(
    private: PinnedDirectory,
    manifest: EvidenceManifest,
    *,
    prediction: Prediction | None = None,
    score: PrivateScore | None = None,
) -> EvidenceManifest:
    prediction = prediction or Prediction.typed("instance", AnswerType.BOOLEAN, True)
    score = score or PrivateScore(
        instance_id="instance",
        answer_type=AnswerType.BOOLEAN,
        value=True,
        run_id="run",
        case_id="instance",
        mode="mode",
        release_id="release",
        scorer_revision="revision",
        outcome="correct",
        scored_at=datetime(2026, 1, 1, tzinfo=timezone.utc),
    )
    prediction_bytes = prediction.model_dump_json().encode()
    score_bytes = score.model_dump_json().encode()
    private.write_bytes("prediction.v1.json", prediction_bytes)
    private.write_bytes("score.v1.json", score_bytes)
    return _manifest_with(
        manifest,
        private=(
            _artifact("prediction.v1.json", prediction_bytes),
            _artifact("score.v1.json", score_bytes),
        ),
    )


def test_orphans_are_invisible_until_marker_and_commit_is_noreplace(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        identity = _identity()
        assert load_committed_result(public, private, expected_identity=identity) is None
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=identity
            )
            is EvidencePublishDisposition.COMMITTED
        )
        result = load_committed_result(public, private, expected_identity=identity)
        assert result is not None and result.prediction.instance_id == "instance"
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=identity
            )
            is EvidencePublishDisposition.ALREADY_EXISTING
        )
        changed = manifest.model_copy(update={"public": ()})
        assert (
            publish_evidence_manifest_noreplace(
                private, changed, public_root=public, expected_identity=identity
            )
            is EvidencePublishDisposition.CONFLICT
        )
    finally:
        public.close()
        private.close()


def test_tampering_and_symlink_are_rejected(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
        private.write_bytes("score.v1.json", b"tampered")
        with pytest.raises(ValueError):
            load_committed_result(public, private, expected_identity=_identity())
    finally:
        public.close()
        private.close()


def test_normal_commit_has_exact_identity_and_canonical_marker(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
        result = load_committed_result(public, private, expected_identity=_identity())
        assert result is not None
        assert result.prediction == Prediction.typed("instance", AnswerType.BOOLEAN, True)
        assert result.score.instance_id == "instance"
        assert result.score.case_id == "instance"
        assert result.score.run_id == "run"
        assert result.score.mode == "mode"
        assert result.score.release_id == "release"
        assert result.score.scorer_revision == "revision"
        assert (
            tmp_path / "private" / "evidence-manifest.v1.json"
        ).read_bytes() == utilities.canonical_json(manifest.model_dump(mode="json")) + b"\n"
        info = os.stat(tmp_path / "private" / "evidence-manifest.v1.json")
        assert stat.S_IMODE(info.st_mode) == 0o600
        assert info.st_uid == os.geteuid() and info.st_nlink == 1
    finally:
        _close(public, private)


@pytest.mark.parametrize("field", ["instance_id", "answer_type", "value", "case_id"])
def test_prediction_score_mismatches_are_rejected(tmp_path: Path, field: str) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        prediction = (
            Prediction.typed("other", AnswerType.BOOLEAN, True)
            if field in {"instance_id", "case_id"}
            else Prediction.typed("instance", AnswerType.INTEGER, 1)
            if field == "answer_type"
            else Prediction.typed("instance", AnswerType.BOOLEAN, False)
        )
        score = PrivateScore(
            instance_id="instance",
            answer_type=AnswerType.BOOLEAN,
            value=True,
            run_id="run",
            case_id="other" if field == "case_id" else "instance",
            mode="mode",
            release_id="release",
            scorer_revision="revision",
            outcome="correct",
            scored_at=datetime(2026, 1, 1, tzinfo=timezone.utc),
        )
        bad = _replace_private(private, manifest, prediction=prediction, score=score)
        with pytest.raises(ValueError, match="identities or values"):
            publish_evidence_manifest_noreplace(
                private, bad, public_root=public, expected_identity=_identity()
            )
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
    finally:
        _close(public, private)


@pytest.mark.parametrize("field", ["run_id", "case_id", "mode", "release_id", "scorer_revision"])
def test_expected_identity_mismatches_are_rejected(tmp_path: Path, field: str) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        values = {
            "run_id": "other",
            "case_id": "other",
            "mode": "other",
            "release_id": "other",
            "scorer_revision": "other",
        }
        expected = _identity().__class__(**{**_identity().__dict__, field: values[field]})
        with pytest.raises(ValueError, match="expected identity"):
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=expected
            )
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
    finally:
        _close(public, private)


def test_duplicate_paths_and_bad_prediction_score_paths_are_prelink_rejections(
    tmp_path: Path,
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        duplicate = _manifest_with(manifest, private=(*manifest.private, manifest.private[0]))
        nested_prediction = _manifest_with(
            manifest, private=(_artifact("nested/prediction.v1.json", b"x"), manifest.private[1])
        )
        nested_score = _manifest_with(
            manifest, private=(manifest.private[0], _artifact("nested/score.v1.json", b"x"))
        )
        wrong_prediction = _manifest_with(
            manifest, private=(_artifact("prediction.json", b"x"), manifest.private[1])
        )
        wrong_score = _manifest_with(
            manifest, private=(manifest.private[0], _artifact("score.json", b"x"))
        )
        for bad in (
            duplicate,
            nested_prediction,
            nested_score,
            wrong_prediction,
            wrong_score,
            _manifest_with(
                manifest, public=(manifest.public[0], _artifact("prediction.v1.json", b"x"))
            ),
            _manifest_with(
                manifest, private=(*manifest.private, _artifact("evidence-manifest.v1.json", b"x"))
            ),
        ):
            with pytest.raises(ValueError):
                publish_evidence_manifest_noreplace(
                    private, bad, public_root=public, expected_identity=_identity()
                )
            assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
    finally:
        _close(public, private)


@pytest.mark.parametrize(
    "public_name, data", [("oracle.txt", b"safe"), ("clean.txt", b"pi-score secret")]
)
def test_unsafe_public_evidence_is_rejected(tmp_path: Path, public_name: str, data: bytes) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        public.write_bytes(public_name, data)
        bad = _manifest_with(manifest, public=(_artifact(public_name, data),))
        with pytest.raises(ValueError, match="public|private score|oracle"):
            publish_evidence_manifest_noreplace(
                private, bad, public_root=public, expected_identity=_identity()
            )
    finally:
        _close(public, private)


@pytest.mark.parametrize("kind", ["symlink", "hardlink"])
def test_non_owned_artifacts_are_rejected(tmp_path: Path, kind: str) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        path = tmp_path / "public" / "case.v1.json"
        path.unlink()
        if kind == "symlink":
            path.symlink_to(tmp_path / "private" / "score.v1.json")
        else:
            os.link(tmp_path / "private" / "score.v1.json", path)
        with pytest.raises((ValueError, OSError)):
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
    finally:
        _close(public, private)


@pytest.mark.parametrize("operation", ["open", "write", "fsync", "rename"])
def test_prelink_failures_leave_no_commit_or_temp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, operation: str
) -> None:
    public, private, manifest = _bundle(tmp_path)
    target = (
        evidence_module._rename_noreplace
        if operation == "rename"
        else getattr(evidence_module.os, operation)
    )

    def fail(*args: object, **kwargs: object) -> object:
        raise OSError(operation)

    if operation == "rename":
        monkeypatch.setattr(evidence_module, "_rename_noreplace", fail)
    else:
        monkeypatch.setattr(evidence_module.os, operation, fail)
    try:
        if operation == "fsync":
            assert (
                publish_evidence_manifest_noreplace(
                    private, manifest, public_root=public, expected_identity=_identity()
                )
                is EvidencePublishDisposition.DURABILITY_UNKNOWN
            )
        else:
            with pytest.raises(OSError):
                publish_evidence_manifest_noreplace(
                    private, manifest, public_root=public, expected_identity=_identity()
                )
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
        assert not list((tmp_path / "private").glob(".evidence-manifest.v1.json.tmp-*"))
    finally:
        if operation == "rename":
            monkeypatch.setattr(evidence_module, "_rename_noreplace", target)
        else:
            monkeypatch.setattr(evidence_module.os, operation, target)
        _close(public, private)


@pytest.mark.parametrize("failure", ["unlink", "parent_fsync"])
def test_postlink_failures_are_durability_unknown_without_rollback(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure: str
) -> None:
    public, private, manifest = _bundle(tmp_path)
    original_unlink, original_fsync = evidence_module.os.unlink, evidence_module.os.fsync
    unlink_calls: list[str] = []

    def fail_unlink(path: str, *args: object, **kwargs: object) -> None:
        unlink_calls.append(str(path))
        if str(path).startswith(".evidence-manifest.v1.json.tmp-"):
            raise OSError("unlink")
        original_unlink(path, *args, **kwargs)

    original_rename = evidence_module._rename_noreplace
    linearized = False

    def recording_rename(*args: Any, **kwargs: Any) -> Any:
        nonlocal linearized
        result = original_rename(*args, **kwargs)
        linearized = True
        return result

    def fail_fsync(fd: int) -> None:
        if failure == "parent_fsync" and linearized and fd == private.fd:
            raise OSError("parent fsync")
        original_fsync(fd)

    monkeypatch.setattr(evidence_module.os, "unlink", fail_unlink)
    monkeypatch.setattr(evidence_module.os, "fsync", fail_fsync)
    if failure == "parent_fsync":
        monkeypatch.setattr(evidence_module, "_rename_noreplace", recording_rename)
    try:
        disposition = publish_evidence_manifest_noreplace(
            private, manifest, public_root=public, expected_identity=_identity()
        )
        assert disposition is (
            EvidencePublishDisposition.COMMITTED
            if failure == "unlink"
            else EvidencePublishDisposition.DURABILITY_UNKNOWN
        )
        assert load_committed_result(public, private, expected_identity=_identity()) is not None
        assert (tmp_path / "private" / "evidence-manifest.v1.json").exists()
        marker_info = os.stat(tmp_path / "private" / "evidence-manifest.v1.json")
        assert marker_info.st_nlink == 1
        assert not list((tmp_path / "private").glob(".evidence-manifest.v1.json.tmp-*"))
        if failure == "unlink":
            assert unlink_calls == []
    finally:
        _close(public, private)


def test_marker_must_be_canonical_regular_owned_and_valid(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    marker = tmp_path / "private" / "evidence-manifest.v1.json"
    try:
        marker.write_bytes(json.dumps(manifest.model_dump(mode="json"), indent=2).encode())
        marker.chmod(0o600)
        with pytest.raises(ValueError, match="canonical"):
            load_committed_result(public, private, expected_identity=_identity())
        marker.unlink()
        marker.write_bytes(b"not json")
        marker.chmod(0o600)
        with pytest.raises(ValueError, match="invalid evidence manifest"):
            load_committed_result(public, private, expected_identity=_identity())
        marker.unlink()
        marker.symlink_to(tmp_path / "public" / "case.v1.json")
        with pytest.raises(OSError):
            load_committed_result(public, private, expected_identity=_identity())
    finally:
        _close(public, private)


@pytest.mark.parametrize("hook", ["read", "fstat"])
def test_read_mutation_is_rejected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, hook: str
) -> None:
    public, private, manifest = _bundle(tmp_path)
    mutated = False
    if hook == "read":
        original_read = evidence_module.os.read

        def mutating_read(fd: int, size: int) -> bytes:
            nonlocal mutated
            data = original_read(fd, size)
            if data and not mutated:
                mutated = True
                public.write_bytes("case.v1.json", b"mutated")
            return data

        monkeypatch.setattr(evidence_module.os, "read", mutating_read)
    else:
        original_fstat = evidence_module.os.fstat
        target_fd: int | None = None
        calls = 0

        def mutating_fstat(fd: int) -> object:
            nonlocal calls, target_fd, mutated
            if target_fd == fd:
                calls += 1
            result = original_fstat(fd)
            if (
                target_fd is None
                and stat.S_ISREG(result.st_mode)
                and result.st_size == len(b"public")
            ):
                target_fd = fd
            elif target_fd == fd and calls == 2 and not mutated:
                mutated = True
                public.write_bytes("case.v1.json", b"mutated")
                result = SimpleNamespace(
                    **{
                        field: getattr(result, field)
                        for field in (
                            "st_mode",
                            "st_uid",
                            "st_nlink",
                            "st_size",
                            "st_dev",
                            "st_ino",
                            "st_ctime_ns",
                        )
                    },
                    st_mtime_ns=result.st_mtime_ns + 1,
                )
            return result

        monkeypatch.setattr(evidence_module.os, "fstat", mutating_fstat)
    try:
        with pytest.raises(ValueError, match="changed while being read|tampered"):
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
    finally:
        _close(public, private)


def test_missing_artifact_after_commit_invalidates_load(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
        (tmp_path / "public" / "case.v1.json").unlink()
        with pytest.raises((ValueError, OSError)):
            load_committed_result(public, private, expected_identity=_identity())
    finally:
        _close(public, private)


def test_artifact_size_bound(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        path = tmp_path / "public" / "case.v1.json"
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        try:
            os.ftruncate(fd, evidence_module.MAX_EVIDENCE_ARTIFACT_BYTES + 1)
        finally:
            os.close(fd)
        bad = _manifest_with(manifest, public=(_artifact("case.v1.json", b"0"),))
        with pytest.raises(ValueError, match="unsafe evidence artifact|size"):
            publish_evidence_manifest_noreplace(
                private, bad, public_root=public, expected_identity=_identity()
            )
    finally:
        _close(public, private)


def test_no_replace_publication_has_one_link_observation_and_survives_interrupt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _bundle(tmp_path)
    marker = tmp_path / "private" / "evidence-manifest.v1.json"
    original_rename = evidence_module._rename_noreplace
    observations: list[int] = []

    def observe_marker() -> None:
        if not marker.exists():
            return
        fd = os.open(marker, os.O_RDONLY | os.O_NOFOLLOW)
        try:
            observations.append(os.fstat(fd).st_nlink)
        finally:
            os.close(fd)

    def instrumented_rename(*args: Any, **kwargs: Any) -> Any:
        assert not marker.exists()
        observe_marker()
        original_rename(*args, **kwargs)
        observe_marker()

    monkeypatch.setattr(evidence_module, "_rename_noreplace", instrumented_rename)
    try:
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
        assert observations == [1]
        assert load_committed_result(public, private, expected_identity=_identity()) is not None
    finally:
        _close(public, private)

    interrupted_root = tmp_path / "interrupted"
    interrupted_root.mkdir()
    public, private, manifest = _bundle(interrupted_root)
    marker = tmp_path / "interrupted" / "private" / "evidence-manifest.v1.json"

    def interrupt_after_link(*args: Any, **kwargs: Any) -> Any:
        original_rename(*args, **kwargs)
        observe_marker()
        raise KeyboardInterrupt

    monkeypatch.setattr(evidence_module, "_rename_noreplace", interrupt_after_link)
    try:
        with pytest.raises(KeyboardInterrupt):
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
        assert observations[-1] == 1
        assert load_committed_result(public, private, expected_identity=_identity()) is not None
    finally:
        _close(public, private)


def test_legacy_manifest_writer_cannot_touch_authoritative_marker(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    marker = tmp_path / "private" / "evidence-manifest.v1.json"
    try:
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
        original = marker.read_bytes()
        with pytest.raises((ValueError, FileExistsError)):
            write_evidence_manifest(
                private, manifest, name=evidence_module.AUTHORITATIVE_MANIFEST_NAME
            )
        assert marker.read_bytes() == original
        write_evidence_manifest(private, manifest, name="evidence-manifest.v1.provisional.json")
        assert (tmp_path / "private" / "evidence-manifest.v1.provisional.json").exists()
    finally:
        _close(public, private)


@pytest.mark.parametrize(
    "identity",
    [
        None,
        EvidenceIdentityContext("", "instance", "mode", "release", "revision"),
        EvidenceIdentityContext("run", "", "mode", "release", "revision"),
        EvidenceIdentityContext("run", "instance", "", "release", "revision"),
        EvidenceIdentityContext("run", "instance", "mode", "", "revision"),
        EvidenceIdentityContext("run", "instance", "mode", "release", ""),
        EvidenceIdentityContext("other", "instance", "mode", "release", "revision"),
    ],
)
def test_publication_requires_complete_matching_identity(
    tmp_path: Path, identity: EvidenceIdentityContext | None
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        with pytest.raises(ValueError, match="identity"):
            publish_evidence_manifest_noreplace(
                private,
                manifest,
                public_root=public,
                expected_identity=cast("EvidenceIdentityContext", identity),
            )
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
    finally:
        _close(public, private)


def test_committed_reads_require_complete_matching_identity(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
        for identity in (
            None,
            EvidenceIdentityContext("", "instance", "mode", "release", "revision"),
            EvidenceIdentityContext("run", "instance", "mode", "release", "other"),
        ):
            with pytest.raises(ValueError, match="identity"):
                load_committed_result(
                    public, private, expected_identity=cast("EvidenceIdentityContext", identity)
                )
        assert load_committed_result(public, private, expected_identity=_identity()) is not None
    finally:
        _close(public, private)


@pytest.mark.parametrize(
    "public_name",
    [
        "NATIVE-SESSION.JSONL",
        "RECEIPT.V1.JSON",
        "PROMPT.TXT",
        "CONTEXT.JSON",
        "SYSTEM-PROMPT.TXT",
        "CREDENTIALS.JSON",
        "SECRET.TXT",
        "TOKEN.TXT",
        "PRIVATE-SCORE.JSON",
        "FAILURE-EVIDENCE.JSON",
        "TOOL-TRANSCRIPT.JSON",
        "PROTOCOL-AUDIT.JSON",
        "LOGS.TXT",
        "EXPORT.HTML",
    ],
)
def test_public_projection_allowlist_rejects_private_categories(
    tmp_path: Path, public_name: str
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        public.write_bytes(public_name, b"safe-looking projection")
        bad = _manifest_with(manifest, public=(_artifact(public_name, b"safe-looking projection"),))
        with pytest.raises(ValueError, match="public|private|evidence"):
            publish_evidence_manifest_noreplace(
                private, bad, public_root=public, expected_identity=_identity()
            )
    finally:
        _close(public, private)


def test_public_projection_allowlist_accepts_approved_result_projection(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    data = b'{"case_id":"instance","status":"complete"}'
    try:
        public.write_bytes("case.v1.json", data)
        approved = _manifest_with(manifest, public=(_artifact("case.v1.json", data),))
        assert (
            publish_evidence_manifest_noreplace(
                private, approved, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
    finally:
        _close(public, private)


def test_manifest_count_and_aggregate_limits_are_checked_before_admission(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _bundle(tmp_path)
    extra = b"approved"
    public.write_bytes("case.v1.json", extra)
    public.write_bytes("map.lcm", extra)
    expanded = _manifest_with(
        manifest,
        public=(_artifact("case.v1.json", extra), _artifact("map.lcm", extra)),
    )
    try:
        monkeypatch.setattr(evidence_module, "MAX_EVIDENCE_ARTIFACT_COUNT", 3, raising=False)
        with pytest.raises(ValueError, match="count|artifact"):
            publish_evidence_manifest_noreplace(
                private, expanded, public_root=public, expected_identity=_identity()
            )
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
    finally:
        _close(public, private)

    aggregate_root = tmp_path / "aggregate"
    aggregate_root.mkdir()
    public, private, manifest = _bundle(aggregate_root)
    try:
        total = sum(item.size_bytes for item in (*manifest.public, *manifest.private))
        monkeypatch.setattr(evidence_module, "MAX_EVIDENCE_TOTAL_BYTES", total, raising=False)
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
    finally:
        _close(public, private)


def test_non_result_artifacts_are_read_in_bounded_chunks(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _bundle(tmp_path)
    data = b"x" * 2048
    public.write_bytes("map.lcm", data)
    manifest = _manifest_with(manifest, public=(_artifact("map.lcm", data),))
    original_read = evidence_module.os.read
    requests: list[int] = []

    def bounded_read(fd: int, size: int) -> bytes:
        requests.append(size)
        return original_read(fd, size)

    monkeypatch.setattr(evidence_module.os, "read", bounded_read)
    try:
        publish_evidence_manifest_noreplace(
            private, manifest, public_root=public, expected_identity=_identity()
        )
        assert requests and max(requests) <= 1024 * 1024
    finally:
        _close(public, private)


def test_admission_retains_only_prediction_and_score_bytes(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    sidecar = b'{"verification":"complete"}'
    private.write_bytes("verification.v1.json", sidecar)
    manifest = _manifest_with(
        manifest, private=(*manifest.private, _artifact("verification.v1.json", sidecar))
    )
    seen: list[tuple[str, bool]] = []
    original_snapshot = evidence_module._read_owned_snapshot

    def instrumented_snapshot(root: PinnedDirectory, relative: str, *, retain: bool) -> Any:
        seen.append((relative, retain))
        return original_snapshot(root, relative, retain=retain)

    try:
        public_snapshot = original_snapshot(public, "case.v1.json", retain=False)
        private_sidecar = original_snapshot(private, "verification.v1.json", retain=False)
        private_prediction = original_snapshot(private, "prediction.v1.json", retain=True)
        assert public_snapshot.data == b""
        assert private_sidecar.data == b""
        assert private_prediction.data
        assert public_snapshot.sha256 == hashlib.sha256(b"public").hexdigest()
        assert private_sidecar.size_bytes == len(sidecar)
        monkeypatch = pytest.MonkeyPatch()
        monkeypatch.setattr(evidence_module, "_read_owned_snapshot", instrumented_snapshot)
        try:
            prediction, score = evidence_module._admit_manifest(
                public, private, manifest, _identity()
            )
        finally:
            monkeypatch.undo()
        assert prediction.instance_id == score.instance_id == "instance"
        assert dict(seen) == {
            "case.v1.json": False,
            "prediction.v1.json": True,
            "score.v1.json": True,
            "verification.v1.json": False,
        }
    finally:
        _close(public, private)


def test_build_artifacts_and_publish_preflight_before_reads_or_serialization(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _bundle(tmp_path)
    public.write_bytes("map.lcm", b"map")
    try:
        monkeypatch.setattr(evidence_module, "MAX_EVIDENCE_ARTIFACT_COUNT", 1, raising=False)
        monkeypatch.setattr(evidence_module, "MAX_EVIDENCE_ARTIFACTS", 1, raising=False)
        monkeypatch.setattr(
            evidence_module,
            "_artifacts",
            lambda *args, **kwargs: pytest.fail("artifact reads occurred before count preflight"),
        )
        with pytest.raises(ValueError, match="count|artifact"):
            evidence_module.build_evidence_manifest(
                tmp_path / "public",
                tmp_path / "private",
                public_artifacts=("case.v1.json", "map.lcm"),
                private_artifacts=(),
            )

        monkeypatch.setattr(
            evidence_module, "canonical_json", lambda *_: pytest.fail("serialized before preflight")
        )
        with pytest.raises(ValueError, match="count|artifact"):
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
    finally:
        _close(public, private)

    boundary_root = tmp_path / "boundary"
    boundary_root.mkdir()
    public, private, _ = _bundle(boundary_root)
    try:
        public.write_bytes("map.lcm", b"map")
        count = 3
        actual_total = len(b"public") + len(b"map")
        monkeypatch.setattr(evidence_module, "MAX_EVIDENCE_ARTIFACT_COUNT", count, raising=False)
        monkeypatch.setattr(evidence_module, "MAX_EVIDENCE_ARTIFACTS", count, raising=False)
        monkeypatch.setattr(
            evidence_module, "MAX_EVIDENCE_TOTAL_BYTES", actual_total, raising=False
        )
        built = evidence_module.build_evidence_manifest(
            boundary_root / "public",
            boundary_root / "private",
            public_artifacts=("case.v1.json", "map.lcm"),
            private_artifacts=(),
        )
        assert sum(item.size_bytes for item in built.public) == actual_total
    finally:
        _close(public, private)


@pytest.mark.parametrize("failure_kind", ["artifact", "artifact_directory"])
def test_artifact_durability_failures_prevent_marker(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure_kind: str
) -> None:
    public, private, manifest = _bundle(tmp_path)
    original_fsync = evidence_module.os.fsync
    public_path = str(tmp_path / "public")
    artifact_path = str(tmp_path / "public" / "case.v1.json")

    def failing_fsync(fd: int) -> None:
        target = os.readlink(f"/proc/self/fd/{fd}")
        if (failure_kind == "artifact" and target == artifact_path) or (
            failure_kind == "artifact_directory" and target == public_path
        ):
            raise OSError(failure_kind)
        original_fsync(fd)

    monkeypatch.setattr(evidence_module.os, "fsync", failing_fsync)
    try:
        outcome: EvidencePublishDisposition | None = None
        try:
            outcome = publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
        except OSError:
            pass
        assert outcome is not EvidencePublishDisposition.COMMITTED
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
    finally:
        _close(public, private)


def test_nested_private_artifact_directory_is_durable_before_linearization(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _bundle(tmp_path)
    private.mkdir("sidecars")
    data = b'{"state":"complete"}'
    private.write_relative("sidecars/verification.v1.json", data)
    manifest = _manifest_with(
        manifest,
        private=(*manifest.private, _artifact("sidecars/verification.v1.json", data)),
    )
    original_fsync = evidence_module.os.fsync
    events: list[str] = []

    def recording_fsync(fd: int) -> None:
        target = os.readlink(f"/proc/self/fd/{fd}")
        if target.endswith("case.v1.json"):
            events.append("public-artifact")
        elif target.endswith("prediction.v1.json"):
            events.append("prediction")
        elif target.endswith("score.v1.json"):
            events.append("score")
        elif target.endswith("sidecars/verification.v1.json"):
            events.append("nested-artifact")
        elif target.endswith("/public"):
            events.append("public-directory")
        elif target.endswith("/private"):
            events.append("private-directory")
        elif target.endswith("/sidecars"):
            events.append("nested-directory")
        elif ".evidence-manifest.v1.json.tmp-" in target:
            events.append("marker-temp")
        original_fsync(fd)

    monkeypatch.setattr(evidence_module.os, "fsync", recording_fsync)
    try:
        publish_evidence_manifest_noreplace(
            private, manifest, public_root=public, expected_identity=_identity()
        )
        assert "nested-artifact" in events
        assert "nested-directory" in events
        assert events.index("nested-artifact") < events.index("marker-temp")
        assert events.index("nested-directory") < events.index("marker-temp")
    finally:
        _close(public, private)


def test_identical_retry_fsyncs_marker_directory_and_reports_durability_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
        original_fsync = evidence_module.os.fsync
        fsyncs: list[int] = []

        def existing_rename(*args: Any, **kwargs: Any) -> None:
            raise FileExistsError

        def recording_fsync(fd: int) -> None:
            fsyncs.append(fd)
            original_fsync(fd)

        monkeypatch.setattr(evidence_module, "_rename_noreplace", existing_rename)
        monkeypatch.setattr(evidence_module.os, "fsync", recording_fsync)
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.ALREADY_EXISTING
        )
        assert private.fd in fsyncs

        def fail_marker_directory(fd: int) -> None:
            if fd == private.fd:
                raise OSError("marker directory")
            original_fsync(fd)

        monkeypatch.setattr(evidence_module.os, "fsync", fail_marker_directory)
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.DURABILITY_UNKNOWN
        )
        assert load_committed_result(public, private, expected_identity=_identity()) is not None
    finally:
        _close(public, private)


def test_committed_requires_ordered_artifact_temp_and_marker_directory_durability(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _bundle(tmp_path)
    original_fsync = evidence_module.os.fsync
    original_rename = evidence_module._rename_noreplace
    events: list[str] = []

    def recording_fsync(fd: int) -> None:
        target = os.readlink(f"/proc/self/fd/{fd}")
        if target.endswith("case.v1.json"):
            events.append("artifact")
        elif target.endswith("/public") or target.endswith("/private"):
            events.append("directory")
        elif ".evidence-manifest.v1.json.tmp-" in target:
            events.append("marker-temp")
        original_fsync(fd)

    def recording_rename(*args: Any, **kwargs: Any) -> Any:
        events.append("linearize")
        return original_rename(*args, **kwargs)

    monkeypatch.setattr(evidence_module.os, "fsync", recording_fsync)
    monkeypatch.setattr(evidence_module, "_rename_noreplace", recording_rename)
    try:
        outcome = publish_evidence_manifest_noreplace(
            private, manifest, public_root=public, expected_identity=_identity()
        )
        assert outcome is EvidencePublishDisposition.COMMITTED
        assert events.index("marker-temp") < events.index("linearize")
        assert events[-1] == "directory"
        assert events.index("artifact") < events.index("linearize")
    finally:
        _close(public, private)


def _multi_level_sidecar_bundle(
    tmp_path: Path,
) -> tuple[PinnedDirectory, PinnedDirectory, EvidenceManifest]:
    public, private, manifest = _bundle(tmp_path)
    private.mkdir("a")
    a_directory = private.open_relative("a")
    try:
        a_directory.mkdir("b")
    finally:
        a_directory.close()
    data = b'{"verification":"complete"}'
    private.write_relative("a/b/sidecar.json", data)
    return (
        public,
        private,
        _manifest_with(
            manifest,
            private=(*manifest.private, _artifact("a/b/sidecar.json", data)),
        ),
    )


def test_multi_level_private_sidecar_syncs_each_unique_directory_in_chain(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _multi_level_sidecar_bundle(tmp_path)
    original_fsync = evidence_module.os.fsync
    original_rename = evidence_module._rename_noreplace
    events: list[tuple[str, str, tuple[int, int]]] = []
    private_root = str(tmp_path / "private")
    required = {
        private_root,
        str(tmp_path / "private" / "a"),
        str(tmp_path / "private" / "a" / "b"),
    }

    def recording_fsync(fd: int) -> None:
        target = os.readlink(f"/proc/self/fd/{fd}")
        info = os.fstat(fd)
        if stat.S_ISDIR(info.st_mode):
            events.append(("directory", target, (info.st_dev, info.st_ino)))
        elif ".evidence-manifest.v1.json.tmp-" in target:
            events.append(("marker-temp", target, (info.st_dev, info.st_ino)))
        original_fsync(fd)

    def recording_rename(*args: Any, **kwargs: Any) -> Any:
        events.append(("linearize", "", (0, 0)))
        return original_rename(*args, **kwargs)

    monkeypatch.setattr(evidence_module.os, "fsync", recording_fsync)
    monkeypatch.setattr(evidence_module, "_rename_noreplace", recording_rename)
    try:
        assert (
            publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
            is EvidencePublishDisposition.COMMITTED
        )
        directory_events = [event for event in events if event[0] == "directory"]
        private_directory_events = [
            event for event in directory_events if event[1].startswith(private_root)
        ]
        assert {event[1] for event in private_directory_events} == required
        assert len({event[2] for event in private_directory_events}) == len(required)
        positions = {
            path: next(index for index, event in enumerate(events) if event[1] == path)
            for path in required
        }
        assert positions[private_root] < positions[str(tmp_path / "private" / "a")]
        assert (
            positions[str(tmp_path / "private" / "a")]
            < positions[str(tmp_path / "private" / "a" / "b")]
        )
        marker_position = next(
            index for index, event in enumerate(events) if event[0] == "marker-temp"
        )
        assert all(position < marker_position for position in positions.values())
    finally:
        _close(public, private)


@pytest.mark.parametrize("failure_directory", ["", "a", "a/b"])
def test_multi_level_private_directory_fsync_failure_prevents_commit(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_directory: str,
) -> None:
    public, private, manifest = _multi_level_sidecar_bundle(tmp_path)
    original_fsync = evidence_module.os.fsync
    failure_path = str(tmp_path / "private" / failure_directory)

    def failing_fsync(fd: int) -> None:
        target = os.readlink(f"/proc/self/fd/{fd}")
        info = os.fstat(fd)
        if stat.S_ISDIR(info.st_mode) and target == failure_path:
            raise OSError(f"directory fsync: {failure_directory or 'root'}")
        original_fsync(fd)

    monkeypatch.setattr(evidence_module.os, "fsync", failing_fsync)
    try:
        outcome: EvidencePublishDisposition | None = None
        try:
            outcome = publish_evidence_manifest_noreplace(
                private, manifest, public_root=public, expected_identity=_identity()
            )
        except OSError:
            pass
        assert outcome is not EvidencePublishDisposition.COMMITTED
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
        assert not list((tmp_path / "private").glob(".evidence-manifest.v1.json.tmp-*"))
    finally:
        _close(public, private)
