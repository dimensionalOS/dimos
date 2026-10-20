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

"""Web CDR schemas and bytes come from the installed owner, not another generator."""

import base64
import json
import pickle
import subprocess
import sys
from typing import Any

from dimos_cdrgen.registry import decode, encode
import pytest

from dimos.messages.geometry_msgs.msg import Point, PoseStamped
from dimos.web.cdr_codec import check_cdr_params, encode_cdr_v1, export_schema
from dimos.web.cockpit import Channel, cockpit
from dimos.web.codecs import resolve_encoder
from dimos.web.relay_bridge.gen_cdr_fixtures import build_messages, build_vectors
from dimos.web.relay_bridge.locate import find_web_dir


def test_cdr_channel_embeds_schema_and_preserves_owned_encoder() -> None:
    channel = Channel(
        "pose", PoseStamped, encoding="geometry_msgs/msg/PoseStamped.cdr.v1", params={"note": "x"}
    )
    blueprint = cockpit(channels=[channel])
    atom = blueprint.blueprints[0]
    wire = atom.kwargs["manifest"]["channels"][0]
    assert wire["params"] == {"note": "x", "cdr": export_schema(PoseStamped)}
    assert "MSG: std_msgs/Header" in wire["params"]["cdr"]["definition"]
    assert "MSG: builtin_interfaces/Time" in wire["params"]["cdr"]["definition"]
    assert dict(channel.params) == {"note": "x"}
    spec = pickle.loads(pickle.dumps(blueprint)).blueprints[0].kwargs["channels"][0]
    msg = build_messages()[0][1]
    assert spec.encoder is encode_cdr_v1
    assert spec.encoder(msg, spec.params) == encode(msg)
    assert next(stream for stream in atom.streams if stream.name == "pose").type is PoseStamped


def test_cdr_channel_rejects_foreign_type_and_reserved_schema() -> None:
    with pytest.raises(ValueError, match="does not match"):
        resolve_encoder("geometry_msgs/msg/Point.cdr.v1", PoseStamped)
    with pytest.raises(ValueError, match="reserved"):
        cockpit(
            channels=[
                Channel(
                    "pose",
                    PoseStamped,
                    encoding="geometry_msgs/msg/PoseStamped.cdr.v1",
                    params={"cdr": {}},
                )
            ]
        )
    with pytest.raises(ValueError, match="declared channel type"):
        encode_cdr_v1(Point(x=0, y=0, z=0), {"cdr": export_schema(PoseStamped)})
    with pytest.raises(ValueError, match="no generated CDR schema"):
        export_schema(dict)


@pytest.mark.parametrize("params", [{}, {"cdr": {}}, {"cdr": {"type": "p.T", "definition": ""}}])
def test_missing_schema_is_rejected(params: dict[str, Any]) -> None:
    with pytest.raises(ValueError, match="params"):
        check_cdr_params(params)


def test_browser_vectors_are_actual_generated_messages_in_both_byte_orders() -> None:
    vectors = json.loads((find_web_dir() / "shared/fixtures/cdr_frames.json").read_text())[
        "vectors"
    ]
    assert vectors == build_vectors()
    for (_, msg), vector in zip(build_messages(), vectors, strict=True):
        for key, little in [("payload_b64", True), ("big_endian_b64", False)]:
            payload = base64.b64decode(vector[key])
            restored = decode(payload, type(msg))
            assert type(restored) is type(msg)
            assert encode(restored, little_endian=little) == payload


def test_codec_import_has_no_generated_or_numpy_side_effects() -> None:
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import dimos.web.cdr_codec; assert 'numpy' not in sys.modules; assert 'dimos.messages' not in sys.modules",
        ],
        check=True,
    )
