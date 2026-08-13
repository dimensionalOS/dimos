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

"""Golden-fixture tests keeping manifest.py behavior identical to manifest.ts."""

import json

import pytest

from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.manifest import ManifestError, parse_manifest

with open(find_web_dir() / "shared" / "fixtures" / "manifests.json") as f:
    VECTORS = json.load(f)["vectors"]

VALID = [v for v in VECTORS if "manifest" in v]
INVALID = [v for v in VECTORS if "error" in v]


@pytest.mark.parametrize("vector", VALID, ids=[v["name"] for v in VALID])
def test_valid_manifest_normalizes_per_golden(vector):
    assert parse_manifest(vector["data"]).model_dump() == vector["manifest"]


@pytest.mark.parametrize("vector", INVALID, ids=[v["name"] for v in INVALID])
def test_invalid_manifest_rejected_with_pinned_code(vector):
    with pytest.raises(ManifestError) as exc_info:
        parse_manifest(vector["data"])
    assert exc_info.value.code == vector["error"]


def test_every_vector_is_classified():
    assert VALID and INVALID
    assert len(VALID) + len(INVALID) == len(VECTORS)


def test_huge_int_max_hz_rejected_like_ts():
    # Not a golden vector: gen.ts cannot emit a number beyond float64
    # (JSON.stringify(Infinity) is null). Python parses such a JSON integer
    # exactly while JS JSON.parse yields Infinity; both sides must reject it
    # with the same code (manifest_test.ts pins the JS half).
    data = {
        "channels": [
            {"ch": "odom", "encoding": "pose.json.v1", "delivery": "reliable", "maxHz": 10**400}
        ]
    }
    with pytest.raises(ManifestError) as exc_info:
        parse_manifest(data)
    assert exc_info.value.code == "invalid_max_hz"
