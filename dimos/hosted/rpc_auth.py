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

"""Ed25519 authentication for Host control RPC payloads."""

from __future__ import annotations

import base64
import os
import time

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey,
    Ed25519PublicKey,
)

_MAGIC = b"dimos-host-rpc-ed25519-v1\0"
_SIGNATURE_SIZE = 64
_TIME_SIZE = 8
_MAX_AGE_SECONDS = 300
_CLIENT_SIGNING_ENV = "DIMOS_HOST_CLIENT_SIGNING_KEY"
_CLIENT_VERIFY_ENV = "DIMOS_HOST_CLIENT_VERIFY_KEY"
_SERVER_SIGNING_ENV = "DIMOS_HOST_SERVER_SIGNING_KEY"
_SERVER_VERIFY_ENV = "DIMOS_HOST_SERVER_VERIFY_KEY"


def _key_bytes(name: str) -> bytes:
    value = os.environ.get(name)
    if not value:
        raise ValueError(f"{name} must contain a base64-encoded raw Ed25519 key")
    try:
        key = base64.b64decode(value, validate=True)
    except ValueError as exc:
        raise ValueError(f"{name} must be valid base64") from exc
    if len(key) != 32:
        raise ValueError(f"{name} must decode to 32 bytes")
    return key


class HostRpcAuth:
    """Sign outgoing messages and verify incoming messages before unpickling."""

    def __init__(
        self,
        *,
        signing_key: Ed25519PrivateKey,
        verification_key: Ed25519PublicKey,
    ) -> None:
        self._signing_key = signing_key
        self._verification_key = verification_key

    @classmethod
    def from_host_env(cls) -> HostRpcAuth:
        return cls(
            signing_key=Ed25519PrivateKey.from_private_bytes(_key_bytes(_SERVER_SIGNING_ENV)),
            verification_key=Ed25519PublicKey.from_public_bytes(_key_bytes(_CLIENT_VERIFY_ENV)),
        )

    @classmethod
    def from_client_env(cls) -> HostRpcAuth:
        return cls(
            signing_key=Ed25519PrivateKey.from_private_bytes(_key_bytes(_CLIENT_SIGNING_ENV)),
            verification_key=Ed25519PublicKey.from_public_bytes(_key_bytes(_SERVER_VERIFY_ENV)),
        )

    def seal(self, scope: str, name: str, payload: bytes) -> bytes:
        timestamp = int(time.time()).to_bytes(_TIME_SIZE, "big")
        body = _MAGIC + timestamp + payload
        signature = self._signing_key.sign(self._context(scope, name) + body)
        return _MAGIC + timestamp + signature + payload

    def open(self, scope: str, name: str, payload: bytes) -> bytes:
        header_size = len(_MAGIC) + _TIME_SIZE + _SIGNATURE_SIZE
        if len(payload) < header_size or not payload.startswith(_MAGIC):
            raise ValueError("Unauthenticated Host RPC payload")
        timestamp_start = len(_MAGIC)
        timestamp_end = timestamp_start + _TIME_SIZE
        timestamp_bytes = payload[timestamp_start:timestamp_end]
        timestamp = int.from_bytes(timestamp_bytes, "big")
        if abs(time.time() - timestamp) > _MAX_AGE_SECONDS:
            raise ValueError("Host RPC payload has expired")
        signature = payload[timestamp_end:header_size]
        content = payload[header_size:]
        body = _MAGIC + timestamp_bytes + content
        try:
            self._verification_key.verify(signature, self._context(scope, name) + body)
        except InvalidSignature as exc:
            raise ValueError("Invalid Host RPC signature") from exc
        return content

    @staticmethod
    def _context(scope: str, name: str) -> bytes:
        scope_bytes = scope.encode("utf-8")
        name_bytes = name.encode("utf-8")
        return (
            len(scope_bytes).to_bytes(2, "big")
            + scope_bytes
            + len(name_bytes).to_bytes(2, "big")
            + name_bytes
        )
