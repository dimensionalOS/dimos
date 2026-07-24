# Copyright 2026 Dimensional Inc.
"""Focused checks for the offline, package-local YCB fixture."""

from pathlib import Path

import pytest

from .fixture import LFS_POINTER_PREFIX, _load_arrays


def test_lfs_pointer_fails_before_numpy_load(tmp_path: Path) -> None:
    path = tmp_path / "scene.npz"
    path.write_bytes(LFS_POINTER_PREFIX + b"oid sha256:fixture\nsize 37623\n")

    with pytest.raises(ValueError, match="Git-LFS pointer.*git lfs pull"):
        _load_arrays(path)
