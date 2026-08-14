import hashlib

import pytest

from dimos.benchmark.libero_pro.assets import LiberoAssetError, _verify
from dimos.benchmark.libero_pro.models import AssetReference


def test_asset_verification_accepts_exact_size_and_digest(tmp_path) -> None:
    payload = b"verified"
    path = tmp_path / "task.bddl"
    path.write_bytes(payload)
    reference = AssetReference(
        repository_path="bddl/task.bddl",
        sha256=hashlib.sha256(payload).hexdigest(),
        size_bytes=len(payload),
    )

    _verify(path, reference)


def test_asset_verification_rejects_wrong_digest(tmp_path) -> None:
    path = tmp_path / "task.bddl"
    path.write_bytes(b"tampered")
    reference = AssetReference(
        repository_path="bddl/task.bddl",
        sha256="0" * 64,
        size_bytes=8,
    )

    with pytest.raises(LiberoAssetError, match="digest mismatch"):
        _verify(path, reference)
