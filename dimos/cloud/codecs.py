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

"""Streaming file codecs for upload artifacts. Ids match the memory codec ids
(`lz4`, ...) so a recording's wire encoding reads the same everywhere."""

from __future__ import annotations

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


class Lz4Codec:
    id, suffix = "lz4", ".lz4"

    def encode(self, src: Path, dst: Path) -> None:
        import lz4.frame

        with src.open("rb") as i, lz4.frame.open(dst, "wb") as o:
            shutil.copyfileobj(i, o)

    def decode(self, src: Path, dst: Path) -> None:
        import lz4.frame

        with lz4.frame.open(src, "rb") as i, dst.open("wb") as o:
            shutil.copyfileobj(i, o)


CODECS: dict[str, type[FileCodec]] = {"": NoCodec, "lz4": Lz4Codec}


def codec(codec_id: str) -> FileCodec:
    try:
        return CODECS[codec_id]()
    except KeyError:
        raise ValueError(f"unknown codec {codec_id!r}; known: {sorted(CODECS)}") from None
