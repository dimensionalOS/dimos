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

"""File compression for upload artifacts. Every supported library exposes the same
`open(path, mode)` API, so an algorithm is a table entry, not a class. The id is
stored as the upload's `content_encoding`; decode is selected by that stamp."""

from __future__ import annotations

from collections.abc import Callable
import importlib
from pathlib import Path
import shutil
from typing import Any

from dimos.constants import CODEC_LIBS


def _lib(codec_id: str) -> Any:
    libs = CODEC_LIBS
    if codec_id not in libs:
        raise ValueError(f"unknown codec {codec_id!r}; known: {sorted(libs)}")
    return importlib.import_module(libs[codec_id][0])


def suffix(codec_id: str) -> str:
    return CODEC_LIBS[codec_id][1] if codec_id else ""


_CHUNK = 1 << 20  # coarse enough to be free, fine enough for a live bar


def compress(
    codec_id: str, src: Path, dst: Path, progress: Callable[[int, int], None] | None = None
) -> None:
    """Stream src into dst through the codec.

    `progress(done, total)` gets input bytes consumed, so the caller can draw a
    real bar for what used to be a silent pause on large recordings.
    """
    with src.open("rb") as i, _lib(codec_id).open(dst, "wb") as o:
        if progress is None:
            shutil.copyfileobj(i, o)
            return
        total, done = src.stat().st_size, 0
        while chunk := i.read(_CHUNK):
            o.write(chunk)
            done += len(chunk)
            progress(done, total)


def decompress(codec_id: str, src: Path, dst: Path) -> None:
    with _lib(codec_id).open(src, "rb") as i, dst.open("wb") as o:
        shutil.copyfileobj(i, o)
