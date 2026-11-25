# Copyright 2026 Dimensional Inc.
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

"""Publish a complete LeRobot directory while preserving an earlier dataset."""

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from tempfile import TemporaryDirectory


@contextmanager
def staged_dataset(destination: Path) -> Iterator[Path]:
    """Replace the destination only after every staged build step succeeds."""
    if destination.is_symlink() or (
        destination.exists()
        and (
            not destination.is_dir()
            or (any(destination.iterdir()) and not (destination / "meta" / "info.json").is_file())
        )
    ):
        raise FileExistsError(f"{destination} already exists and is not a LeRobot dataset")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix=f".{destination.name}-", dir=destination.parent) as temporary:
        staged_path = Path(temporary) / "dataset"
        yield staged_path
        previous = Path(temporary) / "previous"
        if destination.exists():
            destination.rename(previous)
        try:
            staged_path.rename(destination)
        except BaseException:
            if previous.exists():
                previous.rename(destination)
            raise
