# Copyright 2025 Dimensional Inc.
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

import hashlib
import os
import subprocess

import pytest

from dimos.utils import data


@pytest.mark.heavy
def test_pull_file() -> None:
    repo_root = data._get_repo_root()
    test_file_name = "cafe.jpg"
    test_file_compressed = data._get_lfs_dir() / (test_file_name + ".tar.gz")
    test_file_decompressed = data._get_data_dir() / test_file_name

    # delete decompressed test file if it exists
    if test_file_decompressed.exists():
        test_file_decompressed.unlink()

    # delete lfs archive file if it exists
    if test_file_compressed.exists():
        test_file_compressed.unlink()

    assert not test_file_compressed.exists()
    assert not test_file_decompressed.exists()

    # pull the lfs file reference from git
    env = os.environ.copy()
    env["GIT_LFS_SKIP_SMUDGE"] = "1"
    subprocess.run(
        ["git", "checkout", "HEAD", "--", test_file_compressed],
        cwd=repo_root,
        env=env,
        check=True,
        capture_output=True,
    )

    # ensure we have a pointer file from git (small ASCII text file)
    assert test_file_compressed.exists()
    assert test_file_compressed.stat().st_size < 200

    # trigger a data file pull
    assert data.get_data(test_file_name) == test_file_decompressed

    # validate data is received
    assert test_file_compressed.exists()
    assert test_file_decompressed.exists()

    # validate hashes
    with test_file_compressed.open("rb") as f:
        assert test_file_compressed.stat().st_size > 200
        compressed_sha256 = hashlib.sha256(f.read()).hexdigest()
        assert (
            compressed_sha256 == "b8cf30439b41033ccb04b09b9fc8388d18fb544d55b85c155dbf85700b9e7603"
        )

    with test_file_decompressed.open("rb") as f:
        decompressed_sha256 = hashlib.sha256(f.read()).hexdigest()
        assert (
            decompressed_sha256
            == "55d451dde49b05e3ad386fdd4ae9e9378884b8905bff1ca8aaea7d039ff42ddd"
        )


@pytest.mark.heavy
def test_pull_dir() -> None:
    repo_root = data._get_repo_root()
    test_dir_name = "ab_lidar_frames"
    test_dir_compressed = data._get_lfs_dir() / (test_dir_name + ".tar.gz")
    test_dir_decompressed = data._get_data_dir() / test_dir_name

    # delete decompressed test directory if it exists
    if test_dir_decompressed.exists():
        for item in test_dir_decompressed.iterdir():
            item.unlink()
        test_dir_decompressed.rmdir()

    # delete lfs archive file if it exists
    if test_dir_compressed.exists():
        test_dir_compressed.unlink()

    # pull the lfs file reference from git
    env = os.environ.copy()
    env["GIT_LFS_SKIP_SMUDGE"] = "1"
    subprocess.run(
        ["git", "checkout", "HEAD", "--", test_dir_compressed],
        cwd=repo_root,
        env=env,
        check=True,
        capture_output=True,
    )

    # ensure we have a pointer file from git (small ASCII text file)
    assert test_dir_compressed.exists()
    assert test_dir_compressed.stat().st_size < 200

    # trigger a data file pull
    assert data.get_data(test_dir_name) == test_dir_decompressed
    assert test_dir_compressed.stat().st_size > 200

    # validate data is received
    assert test_dir_compressed.exists()
    assert test_dir_decompressed.exists()

    for [file, expected_hash] in zip(
        sorted(test_dir_decompressed.iterdir()),
        [
            "6c3aaa9a79853ea4a7453c7db22820980ceb55035777f7460d05a0fa77b3b1b3",
            "456cc2c23f4ffa713b4e0c0d97143c27e48bbe6ef44341197b31ce84b3650e74",
        ],
        strict=False,
    ):
        with file.open("rb") as f:
            sha256 = hashlib.sha256(f.read()).hexdigest()
            assert sha256 == expected_hash
