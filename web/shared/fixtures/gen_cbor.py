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

"""Generate browser data-projection fixtures with the real pZenoh CBOR encoder."""

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import cbor2

from dimos.protocol.encode import cbor


@dataclass
class Reading:
    label: str
    score: float
    samples: list[int]


plain = {"label": "robot β", "flags": [True, False, None], "position": {"x": 1.25, "y": -3}}
reading = Reading("sensor", 0.75, [1, 2, 3])
vectors = [
    {"name": "json_like", "hex": cbor.encode(plain).hex(), "value": plain},
    {"name": "tuple", "hex": cbor.encode((1, "a", None)).hex(), "value": [1, "a", None]},
    {
        "name": "custom_dataclass",
        "hex": cbor.encode(reading).hex(),
        "value": {
            "$type": f"{Reading.__module__}:{Reading.__qualname__}",
            "fields": asdict(reading),
        },
    },
    {
        "name": "class_reference_is_opaque",
        "hex": cbor.encode(Reading).hex(),
        "opaque": True,
    },
    {
        "name": "function_reference_is_opaque",
        "hex": cbor2.dumps(cbor2.CBORTag(4052, "untrusted.module:execute")).hex(),
        "opaque": True,
    },
    {
        "name": "custom_decoder_is_opaque",
        "hex": cbor2.dumps(cbor2.CBORTag(4054, ["untrusted.module:Class", {}])).hex(),
        "opaque": True,
    },
]
Path(__file__).with_name("topic_cbor.json").write_text(json.dumps(vectors, indent=2) + "\n")
