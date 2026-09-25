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

"""compress streams and reports; the bar that draws it needs both."""

import os
from pathlib import Path

from dimos.cloud import codecs


def test_compress_reports_input_bytes_and_roundtrips(tmp_path: Path) -> None:
    src = tmp_path / "rec.db"
    src.write_bytes(os.urandom(3 * 2**20 + 123))  # not a whole number of chunks
    dst = tmp_path / "rec.db.lz4"
    ticks: list[tuple[int, int]] = []
    codecs.compress("lz4", src, dst, lambda done, total: ticks.append((done, total)))
    total = src.stat().st_size
    assert [t for _, t in ticks] == [total] * len(ticks), "total must be the input size"
    dones = [d for d, _ in ticks]
    assert dones == sorted(dones) and dones[-1] == total and len(dones) == 4
    back = tmp_path / "back.db"
    codecs.decompress("lz4", dst, back)
    assert back.read_bytes() == src.read_bytes()


def test_compress_without_progress_is_unchanged(tmp_path: Path) -> None:
    src = tmp_path / "a"
    src.write_bytes(b"x" * 1000)
    codecs.compress("gzip", src, tmp_path / "a.gz")
    codecs.decompress("gzip", tmp_path / "a.gz", tmp_path / "b")
    assert (tmp_path / "b").read_bytes() == b"x" * 1000
