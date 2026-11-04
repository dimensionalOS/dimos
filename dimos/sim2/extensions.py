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

"""Shared envelope validation for CBOR properties and backend-defined commands."""

from collections.abc import Sequence
from io import BytesIO

import cbor2
from dimos_lcm.control_msgs import CborCommand
from dimos_lcm.sim_msgs import CborExtension


def validate_cbor_entries(
    entries: Sequence[CborExtension | CborCommand], entity_ids: set[str]
) -> None:
    """Reject ambiguous addressing or malformed CBOR; payload semantics belong to its schema."""
    seen: set[tuple[str, str]] = set()
    for prop in entries:
        identity = (prop.entity_id, prop.key)
        if prop.entity_id not in entity_ids or not prop.key or not prop.schema:
            raise ValueError("CBOR entry requires a known entity, property key and schema")
        if identity in seen:
            raise ValueError("duplicate entity/key in CBOR entries")
        seen.add(identity)
        if prop.value_length != len(prop.value):
            raise ValueError("CBOR byte count mismatch")
        payload = BytesIO(bytes(prop.value))
        try:
            cbor2.CBORDecoder(payload).decode()
        except cbor2.CBORDecodeError as error:
            raise ValueError("invalid CBOR") from error
        if payload.read(1):
            raise ValueError("CBOR entry must contain exactly one CBOR value")
