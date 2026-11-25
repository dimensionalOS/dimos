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

import pytest

from dimos.msgs.files.FileInfo import FileInfo
from dimos.msgs.helpers import resolve_msg_type

_SHA256 = "sha256:" + "ab" * 32


@pytest.mark.parametrize("content", [None, b"", b"\x00mesh bytes\xff"])
@pytest.mark.parametrize("sha256", [None, _SHA256])
@pytest.mark.parametrize("mtime", [None, 1764000000.25])
def test_every_field_survives_a_round_trip(
    content: bytes | None, sha256: str | None, mtime: float | None
) -> None:
    sent = FileInfo("meshes/base.obj", sha256, mtime, content, ts=12.5)

    received = FileInfo.lcm_decode(sent.lcm_encode())

    assert (received.uri, received.sha256, received.mtime, received.content, received.ts) == (
        "meshes/base.obj",
        sha256,
        mtime,
        content,
        12.5,
    )


def test_a_header_and_an_empty_file_differ_on_the_wire() -> None:
    header = FileInfo("empty.txt", _SHA256).lcm_encode()
    empty = FileInfo("empty.txt", _SHA256, content=b"").lcm_encode()

    assert header != empty
    assert FileInfo.lcm_decode(header).content is None
    assert FileInfo.lcm_decode(empty).content == b""


def test_the_layout_is_the_one_the_controller_reads() -> None:
    encoded = FileInfo("a", _SHA256, 2.0, b"xy", ts=1.0).lcm_encode()

    # f64 ts, u8 flags (sha256 | mtime | content), u32 uri length, uri, u16 hash length, hash, f64 mtime, content.
    assert encoded[8] == 0b111
    assert encoded[9:13] == (1).to_bytes(4, "little")
    assert encoded[13:14] == b"a"
    assert encoded[14:16] == len(_SHA256).to_bytes(2, "little")
    assert encoded[-2:] == b"xy"


def test_its_name_resolves_to_it() -> None:
    assert resolve_msg_type("files.FileInfo") is FileInfo
