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

import importlib
from pathlib import Path
import shutil

from dimos.core.global_config import global_config


def _lib(codec_id: str):  # type: ignore[no-untyped-def]
    libs = global_config.dimos_codec_libs
    if codec_id not in libs:
        raise ValueError(f"unknown codec {codec_id!r}; known: {sorted(libs)}")
    return importlib.import_module(libs[codec_id][0])


def suffix(codec_id: str) -> str:
    return global_config.dimos_codec_libs[codec_id][1] if codec_id else ""


def compress(codec_id: str, src: Path, dst: Path) -> None:
    with src.open("rb") as i, _lib(codec_id).open(dst, "wb") as o:
        shutil.copyfileobj(i, o)


def decompress(codec_id: str, src: Path, dst: Path) -> None:
    with _lib(codec_id).open(src, "rb") as i, dst.open("wb") as o:
        shutil.copyfileobj(i, o)
