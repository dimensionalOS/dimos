# Copyright 2026 Dimensional Inc.
from pathlib import Path

import pytest

from dimos.benchmark.spatial.pi_baseline.evidence import build_evidence_manifest


def test_evidence_hashes_separate_public_and_private_artifacts(tmp_path: Path) -> None:
    public = tmp_path / "public"
    private = tmp_path / "private"
    public.mkdir()
    private.mkdir()
    case = public / "case.v1.json"
    score = private / "score.json"
    case.write_text('{"record_type":"case"}')
    score.write_text('{"record_type":"pi-score"}')
    case.chmod(0o600)
    score.chmod(0o600)
    manifest = build_evidence_manifest(
        public,
        private,
        public_artifacts=("case.v1.json",),
        private_artifacts=("score.json",),
        required_public=("case.v1.json",),
        required_private=("score.json",),
    )
    assert manifest.public[0].sha256 and manifest.private[0].sha256
    assert manifest.public[0].path == "case.v1.json"


def test_evidence_rejects_private_bytes_and_missing_required_files(tmp_path: Path) -> None:
    public = tmp_path / "public"
    private = tmp_path / "private"
    public.mkdir()
    private.mkdir()
    leak = public / "leak.json"
    leak.write_text('{"record_type":"pi-score"}')
    leak.chmod(0o600)
    with pytest.raises(ValueError, match="not an approved projection"):
        build_evidence_manifest(
            public, private, public_artifacts=("leak.json",), private_artifacts=()
        )
    with pytest.raises(ValueError, match="missing"):
        build_evidence_manifest(
            public,
            private,
            public_artifacts=(),
            private_artifacts=(),
            required_public=("case.v1.json",),
        )
