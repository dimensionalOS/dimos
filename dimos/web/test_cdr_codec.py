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

"""Generated messages advertise the same schema to recordings and browsers."""

import base64
import json
import subprocess
import sys

from dimos_generated.geometry_msgs.msg import Point, PoseStamped
from dimos_generated.sensor_msgs.msg import Image
import pytest

from dimos.web.cdr_codec import check_cdr_params, default_encoding, encode_cdr_v1, export_schema
from dimos.web.relay_bridge.gen_cdr_fixtures import build_messages, build_vectors
from dimos.web.relay_bridge.locate import find_web_dir


def test_default_encoding() -> None:
    assert default_encoding(PoseStamped, "rx") == "geometry_msgs/msg/PoseStamped.cdr.v1"
    assert default_encoding(PoseStamped, "tx") == "json.v1"
    assert default_encoding(dict, "rx") == "json.v1"
    with pytest.raises(ValueError, match="jpeg.v1"):
        default_encoding(Image, "rx")


def test_schema_and_declared_type() -> None:
    schema = export_schema(PoseStamped)
    assert schema["type"] == "geometry_msgs/msg/PoseStamped"
    assert "MSG: std_msgs/Header" in schema["definition"]
    params = {"cdr": schema}
    check_cdr_params(params)
    msg = PoseStamped()
    assert encode_cdr_v1(msg, params) == msg.encode()
    with pytest.raises(ValueError, match="declared channel type"):
        encode_cdr_v1(Point(), params)
    for bad in [{}, {"cdr": {}}, {"cdr": {"type": "p.T", "definition": ""}}]:
        with pytest.raises(ValueError, match="params"):
            check_cdr_params(bad)
    with pytest.raises(ValueError, match="no generated CDR schema"):
        export_schema(dict)


def test_browser_fixtures_match_generated_codecs() -> None:
    expected = json.loads((find_web_dir() / "shared/fixtures/cdr_frames.json").read_text())
    assert {"vectors": build_vectors()} == expected
    for (_, msg), vector in zip(build_messages(), expected["vectors"], strict=True):
        assert type(msg).decode(base64.b64decode(vector["payload_b64"])) == msg
        assert type(msg).decode(base64.b64decode(vector["big_endian_b64"])) == msg


def test_codec_import_is_light() -> None:
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import dimos.web.cdr_codec; "
            "assert 'numpy' not in sys.modules; assert 'dimos_generated' not in sys.modules",
        ],
        check=True,
    )
