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

from dimos.memory.codecs.base import codec_from_id
from dimos.memory.codecs.json import JsonCodec
from dimos.msgs.std_msgs.String import String


@pytest.mark.parametrize(
    "text", ['{"label":"拿起积木 🦾","counter":9007199254740993}', "[1,2,null]", '"label"']
)
def test_json_storage_preserves_text_without_a_transport_envelope(text):
    codec = codec_from_id("json", "dimos.msgs.std_msgs.String.String")
    encoded = codec.encode(String(text))
    assert encoded == text.encode("utf-8")
    assert codec.decode(encoded).data == text


@pytest.mark.parametrize("data", [b"not json", b'{"ts":NaN}', b'{"ts":Infinity}', b"\xff"])
def test_json_codec_rejects_invalid_documents(data):
    with pytest.raises(ValueError):
        JsonCodec().decode(data)


def test_json_codec_does_not_accept_other_wire_types():
    with pytest.raises(TypeError, match="requires std_msgs.String"):
        codec_from_id("json", "dimos.msgs.sensor_msgs.Image.Image")
