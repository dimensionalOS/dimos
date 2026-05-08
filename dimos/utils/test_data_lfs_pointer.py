# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Fast unit tests for Git LFS pointer detection (no network, no LFS pull)."""

from pathlib import Path

import pytest

from dimos.utils.data import _is_lfs_pointer_file


def test_is_lfs_pointer_true_for_valid_pointer(tmp_path: Path) -> None:
    path = tmp_path / "sample.tar.gz"
    path.write_text(
        "version https://git-lfs.github.com/spec/v1\n"
        "oid sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
        "size 42\n",
        encoding="utf-8",
    )
    assert _is_lfs_pointer_file(path) is True


def test_is_lfs_pointer_false_when_file_too_large(tmp_path: Path) -> None:
    path = tmp_path / "big.bin"
    path.write_bytes(b"x" * 1025)
    assert _is_lfs_pointer_file(path) is False


def test_is_lfs_pointer_false_for_plain_text(tmp_path: Path) -> None:
    path = tmp_path / "readme.txt"
    path.write_text("hello world\n", encoding="utf-8")
    assert _is_lfs_pointer_file(path) is False


def test_is_lfs_pointer_false_for_missing_file(tmp_path: Path) -> None:
    path = tmp_path / "does_not_exist"
    assert _is_lfs_pointer_file(path) is False


def test_is_lfs_pointer_false_for_invalid_utf8_small_file(tmp_path: Path) -> None:
    path = tmp_path / "binary.bin"
    path.write_bytes(b"\xff\xfe" * 50)
    assert _is_lfs_pointer_file(path) is False
