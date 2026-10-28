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

"""Generic <msg_name>.lcm.v1 codec: the default rule, schema export, the
encoder's fingerprint guard, and the golden vectors shared with
web/sdk/src/decoders/lcm.test.ts."""

import base64
import json
import subprocess
import sys
from typing import Any

from dimos_lcm import (
    geometry_msgs as lcm_geometry_msgs,
    sensor_msgs as lcm_sensor_msgs,
    std_msgs as lcm_std_msgs,
    visualization_msgs as lcm_visualization_msgs,
)
import pytest

from dimos.messages.geometry_msgs.msg import PoseStamped
from dimos.messages.sensor_msgs.msg import Image
from dimos.web.codecs import default_encoding
from dimos.web.lcm_codec import (
    check_lcm_params,
    encode_lcm_v1,
    export_schema,
    schema_class_for,
)
from dimos.web.relay_bridge.gen_lcm_fixtures import build_messages, walk_value
from dimos.web.relay_bridge.locate import find_web_dir

with open(find_web_dir() / "shared" / "fixtures" / "lcm_frames.json") as f:
    VECTORS: list[dict[str, Any]] = json.load(f)["vectors"]
MESSAGES = dict(build_messages())


class _Bare:
    msg_name = "Bare"

    def lcm_encode(self) -> bytes:
        return b""


def test_default_encoding():
    assert default_encoding(PoseStamped, "rx") == "geometry_msgs/msg/PoseStamped.cdr.v1"
    assert default_encoding(PoseStamped, "tx") == "geometry_msgs/msg/PoseStamped.cdr.v1"
    assert default_encoding(Image, "rx") == "sensor_msgs/msg/Image.cdr.v1"
    assert default_encoding(dict, "rx") == "json.v1"
    assert default_encoding(lcm_visualization_msgs.MarkerArray, "rx") == "json.v1"


@pytest.mark.parametrize("message_type", [PoseStamped, Image, dict, _Bare])
def test_schema_class_for_rejects_non_foreign_messages(message_type):
    with pytest.raises(ValueError):
        schema_class_for(message_type)


def test_export_schema_dims() -> None:
    scan = export_schema(lcm_sensor_msgs.LaserScan)["structs"]["sensor_msgs.LaserScan"]
    assert ["ranges", "float", ["ranges_length"]] in scan
    imu = export_schema(lcm_sensor_msgs.Imu)["structs"]["sensor_msgs.Imu"]
    assert ["orientation_covariance", "double", [9]] in imu
    assert export_schema(lcm_std_msgs.Empty)["structs"] == {"std_msgs.Empty": []}


def test_encode_lcm_v1_guards_the_fingerprint() -> None:
    params = {"lcm": export_schema(lcm_geometry_msgs.PoseStamped)}
    check_lcm_params(params)
    msg = MESSAGES["pose_stamped"]
    assert encode_lcm_v1(msg, params) == msg.lcm_encode()
    with pytest.raises(ValueError, match="wrote fingerprint"):
        encode_lcm_v1(lcm_geometry_msgs.Pose(), params)
    with pytest.raises(ValueError, match="raw generated foreign schema"):
        encode_lcm_v1(object(), params)
    for bad in ({}, {"lcm": {"type": "x", "fp": "zz", "structs": {}}}, {"lcm": {"type": "x"}}):
        with pytest.raises(ValueError, match=r"need params\['lcm'\]"):
            check_lcm_params(bad)


@pytest.mark.parametrize("vec", VECTORS, ids=lambda v: v["name"])
def test_golden_vector(vec: dict[str, Any]) -> None:
    msg = MESSAGES[vec["name"]]
    generated = schema_class_for(type(msg))
    assert vec["encoding"] == vec["schema"]["type"] + ".lcm.v1"
    assert export_schema(generated) == vec["schema"]
    payload = base64.b64decode(vec["payload_b64"])
    assert encode_lcm_v1(msg, {"lcm": vec["schema"]}) == payload
    assert walk_value(generated.lcm_decode(payload), generated) == vec["value"]


def test_golden_vectors_cover_every_message() -> None:
    assert [v["name"] for v in VECTORS] == list(MESSAGES)


def test_import_stays_light() -> None:
    # cockpit.py and codecs.py import this module at module scope; the LCM
    # classes load only when a schema is resolved.
    code = (
        "import sys; import dimos.web.lcm_codec; "
        "assert 'numpy' not in sys.modules; "
        "assert 'dimos_lcm' not in sys.modules; "
        "assert 'dimos.web.relay_bridge.relay_bridge_module' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
