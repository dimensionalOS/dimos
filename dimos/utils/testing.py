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

import subprocess
import tarfile
import glob
import os
import pickle
from functools import cache
from pathlib import Path
from typing import Union, Iterator, TypeVar, Generic, Optional, Any, Type, Callable

from reactivex import operators as ops
from reactivex import interval, from_iterable
from reactivex.observable import Observable


def _check_git_lfs_available() -> None:
    try:
        subprocess.run(["git", "lfs", "version"], capture_output=True, check=True, text=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        raise RuntimeError(
            "Git LFS is not installed. Please install git-lfs to use test data utilities.\n"
            "Installation instructions: https://git-lfs.github.io/"
        )
    return True


@cache
def _get_repo_root() -> Path:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"], capture_output=True, check=True, text=True
        )
        return Path(result.stdout.strip())
    except subprocess.CalledProcessError:
        raise RuntimeError("Not in a Git repository")


@cache
def _get_data_dir() -> Path:
    return _get_repo_root() / "tests" / "data"


@cache
def _get_lfs_dir() -> Path:
    return _get_data_dir() / ".lfs"


def _is_lfs_pointer_file(file_path: Path) -> bool:
    try:
        # LFS pointer files are small (typically < 200 bytes) and start with specific text
        if file_path.stat().st_size > 1024:  # LFS pointers are much smaller
            return False

        with open(file_path, "r", encoding="utf-8") as f:
            first_line = f.readline().strip()
            return first_line.startswith("version https://git-lfs.github.com/spec/")

    except (UnicodeDecodeError, OSError):
        return False


def _lfs_pull(file_path: Path, repo_root: Path) -> None:
    try:
        relative_path = file_path.relative_to(repo_root)

        subprocess.run(
            ["git", "lfs", "pull", "--include", str(relative_path)],
            cwd=repo_root,
            check=True,
            capture_output=True,
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to pull LFS file {file_path}: {e}")


def _pull_lfs_archive(filename: Union[str, Path]) -> Path:
    # Check Git LFS availability first
    _check_git_lfs_available()

    # Find repository root
    repo_root = _get_repo_root()

    # Construct path to test data file
    file_path = _get_lfs_dir() / (filename + ".tar.gz")

    # Check if file exists
    if not file_path.exists():
        raise FileNotFoundError(
            f"Test file '{filename}' not found at {file_path}. "
            f"Make sure the file is committed to Git LFS in the tests/data directory."
        )

    # If it's an LFS pointer file, ensure LFS is set up and pull the file
    if _is_lfs_pointer_file(file_path):
        _lfs_pull(file_path, repo_root)

        # Verify the file was actually downloaded
        if _is_lfs_pointer_file(file_path):
            raise RuntimeError(
                f"Failed to download LFS file '{filename}'. The file is still a pointer after attempting to pull."
            )

    return file_path


def _decompress_archive(filename: Union[str, Path]) -> Path:
    target_dir = _get_data_dir()
    filename_path = Path(filename)
    with tarfile.open(filename_path, "r:gz") as tar:
        tar.extractall(target_dir)
    return target_dir / filename_path.name.replace(".tar.gz", "")


def testData(filename: Union[str, Path]) -> Path:
    """
    Get the path to a test data, downloading from LFS if needed.

    This function will:
    1. Check that Git LFS is available
    2. Locate the file in the tests/data directory
    3. Initialize Git LFS if needed
    4. Download the file from LFS if it's a pointer file
    5. Return the Path object to the actual file or dir

    Args:
        filename: Name of the test file (e.g., "lidar_sample.bin")

    Returns:
        Path: Path object to the test file

    Raises:
        RuntimeError: If Git LFS is not available or LFS operations fail
        FileNotFoundError: If the test file doesn't exist

    Usage:
        # As string path
        file_path = str(testFile("sample.bin"))

        # As context manager for file operations
        with testFile("sample.bin").open('rb') as f:
            data = f.read()
    """
    data_dir = _get_data_dir()
    file_path = data_dir / filename

    # already pulled and decompressed, return it directly
    if file_path.exists():
        return file_path

    return _decompress_archive(_pull_lfs_archive(filename))


T = TypeVar("T")


class SensorReplay(Generic[T]):
    """Generic sensor data replay utility.

    Args:
        name: The name of the test dataset
        autocast: Optional function that takes unpickled data and returns a processed result.
                  For example: lambda data: LidarMessage.from_msg(data)
    """

    def __init__(self, name: str, autocast: Optional[Callable[[Any], T]] = None):
        self.root_dir = testData(name)
        self.autocast = autocast
        self.cnt = 0

    def load(self, *names: Union[int, str]) -> Union[T, Any, list[T], list[Any]]:
        if len(names) == 1:
            return self.load_one(names[0])
        return list(map(lambda name: self.load_one(name), names))

    def load_one(self, name: Union[int, str, Path]) -> Union[T, Any]:
        if isinstance(name, int):
            full_path = self.root_dir / f"/{name:03d}.pickle"
        elif isinstance(name, Path):
            full_path = self.root_dir / f"/{name}.pickle"
        else:
            full_path = name

        with open(full_path, "rb") as f:
            data = pickle.load(f)
            if self.autocast:
                return self.autocast(data)
            return data

    def iterate(self) -> Iterator[Union[T, Any]]:
        pattern = os.path.join(self.root_dir, "*")
        for file_path in sorted(glob.glob(pattern)):
            yield self.load_one(file_path)

    def stream(self, rate_hz: float = 10.0) -> Observable[Union[T, Any]]:
        sleep_time = 1.0 / rate_hz

        return from_iterable(self.iterate()).pipe(
            ops.zip(interval(sleep_time)),
            ops.map(lambda x: x[0] if isinstance(x, tuple) else x),
        )

    def save_stream(self, observable: Observable[Union[T, Any]]) -> Observable[int]:
        return observable.pipe(ops.map(lambda frame: self.save_one(frame)))

    def save(self, *frames) -> int:
        [self.save_one(frame) for frame in frames]
        return self.cnt

    def save_one(self, frame) -> int:
        file_name = f"/{self.cnt:03d}.pickle"
        full_path = self.root_dir + file_name

        self.cnt += 1

        if os.path.isfile(full_path):
            raise Exception(f"file {full_path} exists")

        # Convert to raw message if frame has a raw_msg attribute
        if hasattr(frame, "raw_msg"):
            frame = frame.raw_msg

        with open(full_path, "wb") as f:
            pickle.dump(frame, f)

        return self.cnt
