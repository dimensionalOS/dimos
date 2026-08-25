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

"""Streaming file codecs for upload artifacts.

Compression algorithms are interchangeable behind one class: every supported
library exposes the same `open(path, mode)` file API, so the algorithm is data,
not a subclass. Ids double as the wire `content_encoding`."""

from __future__ import annotations

import importlib
from pathlib import Path
import shutil
from typing import Protocol


class FileCodec(Protocol):
    id: str
    suffix: str

    def encode(self, src: Path, dst: Path) -> None: ...
    def decode(self, src: Path, dst: Path) -> None: ...


class NoCodec:
    id, suffix = "", ""

    def encode(self, src: Path, dst: Path) -> None:
        shutil.copyfile(src, dst)

    def decode(self, src: Path, dst: Path) -> None:
        shutil.copyfile(src, dst)


class Compression:
    _algos = {
        "lz4": ("lz4.frame", ".lz4"),
        "gzip": ("gzip", ".gz"),
        "bz2": ("bz2", ".bz2"),
        "xz": ("lzma", ".xz"),
    }

    def __init__(self, algo: str) -> None:
        module, self.suffix = self._algos[algo]
        self.id = algo
        self._lib = importlib.import_module(module)

    def encode(self, src: Path, dst: Path) -> None:
        with src.open("rb") as i, self._lib.open(dst, "wb") as o:
            shutil.copyfileobj(i, o)

    def decode(self, src: Path, dst: Path) -> None:
        with self._lib.open(src, "rb") as i, dst.open("wb") as o:
            shutil.copyfileobj(i, o)


def codec(codec_id: str) -> FileCodec:
    if not codec_id:
        return NoCodec()
    if codec_id not in Compression._algos:
        raise ValueError(f"unknown codec {codec_id!r}; known: {sorted(Compression._algos)}")
    return Compression(codec_id)
