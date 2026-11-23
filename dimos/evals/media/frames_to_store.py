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

"""Write ordered RGB frames into a DimOS ``color_image`` sqlite store.

Used to turn OpenEQA episode histories (and similar frame packs) into the same
``recording.db`` shape ``Dataset`` / ``dimos_agent`` already consume. This is
preprocessing: the eval loop opens the store; it does not decode video.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Sequence
import re
import tarfile
from pathlib import Path

import cv2
import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

_RGB_NAME = re.compile(r"(?:^|/)(\d+)-rgb\.(png|jpg|jpeg)$", re.IGNORECASE)


def frame_index(path: str | Path) -> int | None:
    """OpenEQA-style ``00042-rgb.png`` index, or None if the name does not match."""
    match = _RGB_NAME.search(str(path).replace("\\", "/"))
    return int(match.group(1)) if match else None


def sorted_frame_paths(paths: Iterable[Path]) -> list[Path]:
    """RGB frame paths ordered by their numeric index."""
    ranked: list[tuple[int, Path]] = []
    for path in paths:
        index = frame_index(path)
        if index is not None:
            ranked.append((index, path))
    ranked.sort(key=lambda item: item[0])
    return [path for _, path in ranked]


def _bgr_from_bytes(payload: bytes) -> np.ndarray:
    array = np.frombuffer(payload, dtype=np.uint8)
    bgr = cv2.imdecode(array, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("cv2.imdecode failed")
    return bgr


def _bgr_from_path(path: Path) -> np.ndarray:
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"could not read image {path}")
    return bgr


def iter_tar_rgb_bgr(tar_path: Path) -> Iterator[tuple[int, np.ndarray]]:
    """Yield ``(frame_index, BGR array)`` from an OpenEQA episode ``.tar``."""
    with tarfile.open(tar_path, "r") as archive:
        members = []
        for member in archive.getmembers():
            if not member.isfile():
                continue
            index = frame_index(member.name)
            if index is None:
                continue
            members.append((index, member))
        members.sort(key=lambda item: item[0])
        for index, member in members:
            handle = archive.extractfile(member)
            if handle is None:
                raise RuntimeError(f"missing member {member.name} in {tar_path}")
            yield index, _bgr_from_bytes(handle.read())


def iter_dir_rgb_bgr(directory: Path) -> Iterator[tuple[int, np.ndarray]]:
    """Yield ``(frame_index, BGR array)`` from a directory of ``NNNNN-rgb.png`` files."""
    frames = sorted_frame_paths(directory.rglob("*"))
    if not frames:
        raise FileNotFoundError(f"no NNNNN-rgb.png frames under {directory}")
    for path in frames:
        index = frame_index(path)
        assert index is not None
        yield index, _bgr_from_path(path)


def write_color_image_store(
    frames: Sequence[tuple[int, np.ndarray]] | Iterator[tuple[int, np.ndarray]],
    out: Path,
    *,
    hz: float = 1.0,
    frame_id: str = "camera",
) -> int:
    """Write BGR frames to ``out`` as ``color_image``. Returns the frame count.

    Timestamps are ``index / hz`` seconds from the first frame's index origin
    (OpenEQA indices start at 0, so ``t = index / hz``).
    """
    if hz <= 0:
        raise ValueError("hz must be > 0")
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.unlink()

    written = 0
    with SqliteStore(path=str(out)) as store:
        color = store.stream("color_image", Image)
        for index, bgr in frames:
            ts = float(index) / hz
            color.append(
                Image.from_opencv(bgr, format=ImageFormat.BGR, frame_id=frame_id, ts=ts),
                ts=ts,
            )
            written += 1
    if written == 0:
        raise RuntimeError(f"no frames written to {out}")
    return written


def tar_to_store(tar_path: Path, out: Path, *, hz: float = 1.0) -> int:
    """OpenEQA episode ``.tar`` → ``recording.db`` with a ``color_image`` stream."""
    return write_color_image_store(iter_tar_rgb_bgr(Path(tar_path)), out, hz=hz)


def dir_to_store(directory: Path, out: Path, *, hz: float = 1.0) -> int:
    """Directory of ``NNNNN-rgb.png`` → ``recording.db``."""
    return write_color_image_store(iter_dir_rgb_bgr(Path(directory)), out, hz=hz)
