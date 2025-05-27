import hashlib
import os
import subprocess
from dimos.utils import testing


def test_pull_file():
    repo_root = testing._get_repo_root()
    test_file_name = "cafe.jpg"
    test_file_compressed = testing._get_lfs_dir() / (test_file_name + ".tar.gz")
    test_file_decompressed = testing._get_data_dir() / test_file_name

    # delete decompressed test file if it exists
    if test_file_decompressed.exists():
        test_file_compressed.unlink()

    # delete lfs archive file if it exists
    if test_file_compressed.exists():
        test_file_compressed.unlink()

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
    test_file_compressed.stat().st_size < 200

    # trigger a data file pull
    assert testing.testData(test_file_name) == test_file_decompressed

    # validate data is received
    assert test_file_compressed.exists()
    assert test_file_decompressed.exists()

    # validate hashes
    with test_file_compressed.open("rb") as f:
        compressed_sha256 = hashlib.sha256(f.read()).hexdigest()
        assert compressed_sha256 == "cdfd708d66e6dd5072ed7636fc10fb97754f8d14e3acd6c3553663e27fc96065"

    with test_file_decompressed.open("rb") as f:
        decompressed_sha256 = hashlib.sha256(f.read()).hexdigest()
        assert decompressed_sha256 == "55d451dde49b05e3ad386fdd4ae9e9378884b8905bff1ca8aaea7d039ff42ddd"


def test_pull_dir():
    repo_root = testing._get_repo_root()
    test_dir_name = "ab_lidar_frames"
    test_dir_compressed = testing._get_lfs_dir() / (test_dir_name + ".tar.gz")
    test_dir_decompressed = testing._get_data_dir() / test_dir_name

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
    test_dir_compressed.stat().st_size < 200

    # trigger a data file pull
    assert testing.testData(test_dir_name) == test_dir_decompressed

    # validate data is received
    assert test_dir_compressed.exists()
    assert test_dir_decompressed.exists()

    assert len(list(test_dir_decompressed.iterdir())) == 2
