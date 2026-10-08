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

"""MCAP-specific recording reader; requires the optional learning dependencies."""

from pathlib import Path
from typing import cast

from mcap.reader import make_reader

from dimos.memory.codecs.base import codec_from_id
from dimos.memory.store.mcap import McapStore, StreamCodec


def open_recording(path: Path) -> McapStore:
    """Open a native MCAP recording with codecs from its stream metadata."""
    return McapStore(path=str(path), codecs=recording_codecs(path))


def recording_codecs(path: Path) -> dict[str, StreamCodec]:
    """Load native codecs from a trusted recording's message-type metadata."""
    with path.open("rb") as file:
        summary = make_reader(file).get_summary()
    codecs: dict[str, StreamCodec] = {}
    if summary is None:
        return codecs
    for channel in summary.channels.values():
        payload_type = channel.metadata.get("dimos.payload_type")
        if payload_type and channel.message_encoding in {"jpeg", "lcm", "lz4+lcm"}:
            try:
                codecs[channel.topic] = cast(
                    "StreamCodec", codec_from_id(channel.message_encoding, payload_type)
                )
            except (ImportError, AttributeError) as exc:
                raise ImportError(
                    f"Cannot decode MCAP stream {channel.topic!r}: install the package "
                    f"providing {payload_type!r} in the dataset reader environment"
                ) from exc
    return codecs
