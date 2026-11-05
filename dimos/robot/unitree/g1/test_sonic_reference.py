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

import hashlib
import json
from pathlib import Path
import struct

import pytest

from dimos.robot.unitree.g1.sonic_reference import encode_command, encode_token_reference


@pytest.fixture
def golden():
    return json.loads((Path(__file__).parent / "test_data/sonic_reference_golden.json").read_text())


def test_command_bytes_match_installed_streamer(golden):
    for case in golden["commands"]:
        packet = encode_command(start=case["start"], stop=case["stop"], planner=case["planner"])
        assert hashlib.sha256(packet).hexdigest() == case["sha256"]
        assert len(packet) == case["bytes"]


@pytest.mark.parametrize("planner", [False, True])
def test_mode_only_command_does_not_imply_start_or_stop(planner):
    packet = encode_command(start=False, stop=False, planner=planner)
    assert packet[:7] == b"command"
    assert packet[1287:] == bytes([0, 0, int(planner)])
    assert json.loads(packet[7:1287].rstrip(b"\0")) == {
        "v": 1,
        "endian": "le",
        "count": 1,
        "fields": [
            {"name": "start", "dtype": "u8", "shape": [1]},
            {"name": "stop", "dtype": "u8", "shape": [1]},
            {"name": "planner", "dtype": "u8", "shape": [1]},
        ],
    }


def test_contradictory_command_is_rejected():
    with pytest.raises(ValueError, match="start.*stop"):
        encode_command(start=True, stop=True, planner=True)


@pytest.mark.parametrize("field", ["start", "stop", "planner"])
@pytest.mark.parametrize("value", [0, 1, "false", None])
def test_command_flags_require_booleans(field, value):
    args = {"start": False, "stop": False, "planner": True, field: value}
    with pytest.raises(ValueError, match=field):
        encode_command(**args)


def test_token_reference_matches_installed_streamer_without_hand_permutation(golden):
    case = golden["pose"]
    packet = encode_token_reference(
        token=case["token"],
        left_hand=case["left"],
        right_hand=case["right"],
        frame_index=case["frame_index"],
    )
    assert hashlib.sha256(packet).hexdigest() == case["sha256"]
    assert len(packet) == 1604
    assert packet[:4] == b"pose"
    assert json.loads(packet[4:1284].rstrip(b"\0")) == {
        "v": 4,
        "endian": "le",
        "count": 1,
        "fields": [
            {"name": "token_state", "dtype": "f32", "shape": [1, 64]},
            {"name": "frame_index", "dtype": "i64", "shape": [1]},
            {"name": "left_hand_joints", "dtype": "f32", "shape": [1, 7]},
            {"name": "right_hand_joints", "dtype": "f32", "shape": [1, 7]},
        ],
    }
    assert struct.unpack_from("<64f", packet, 1284) == tuple(case["token"])
    assert struct.unpack_from("<q", packet, 1540) == (case["frame_index"],)
    assert struct.unpack_from("<7f", packet, 1548) == tuple(case["left"])
    assert struct.unpack_from("<7f", packet, 1576) == tuple(case["right"])


@pytest.fixture
def reference_args():
    return {"token": [0.0] * 64, "left_hand": [0.0] * 7, "right_hand": [0.0] * 7, "frame_index": 0}


@pytest.mark.parametrize("field", ["token", "left_hand", "right_hand"])
@pytest.mark.parametrize("delta", [-1, 1])
def test_reference_rejects_wrong_dimensions(reference_args, field, delta):
    reference_args[field] = [0.0] * (len(reference_args[field]) + delta)
    with pytest.raises(ValueError, match=field):
        encode_token_reference(**reference_args)


@pytest.mark.parametrize("field", ["token", "left_hand", "right_hand"])
@pytest.mark.parametrize(
    "value", [float("nan"), float("inf"), -float("inf"), 1e40, True, "0", None, [0]]
)
def test_reference_rejects_invalid_float32_values(reference_args, field, value):
    reference_args[field][0] = value
    with pytest.raises(ValueError, match=field):
        encode_token_reference(**reference_args)


@pytest.mark.parametrize("index", [-1, 2**63, 1.0, True, "1", None])
def test_reference_rejects_invalid_frame_index(reference_args, index):
    reference_args["frame_index"] = index
    with pytest.raises(ValueError, match="frame_index"):
        encode_token_reference(**reference_args)


@pytest.mark.parametrize("index", [0, 2**63 - 1])
def test_reference_preserves_int64_boundaries(reference_args, index):
    reference_args["frame_index"] = index
    packet = encode_token_reference(**reference_args)
    assert struct.unpack_from("<q", packet, 1540) == (index,)
