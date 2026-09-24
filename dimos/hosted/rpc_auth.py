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

"""Shared-key authentication for Host control RPC payloads."""

from __future__ import annotations

import base64
import hashlib
import hmac
import os
import time

_MAGIC = b"dimos-host-rpc-hmac-v1\0"
_MAC_SIZE = hashlib.sha256().digest_size
_TIME_SIZE = 8
_MAX_AGE_SECONDS = 300
_KEY_ENV = "DIMOS_HOST_CONTROL_KEY"


class HostRpcAuth:
    """Bind each request or response to its RPC name and a short validity window."""

    def __init__(self, key: bytes) -> None:
        if len(key) < 32:
            raise ValueError("Host control key must contain at least 32 random bytes")
        self._key = key

    @classmethod
    def from_host_env(cls) -> HostRpcAuth:
        return cls._from_env()

    @classmethod
    def from_client_env(cls) -> HostRpcAuth:
        return cls._from_env()

    @classmethod
    def _from_env(cls) -> HostRpcAuth:
        value = os.environ.get(_KEY_ENV)
        if not value:
            raise ValueError(f"{_KEY_ENV} must be set to a base64-encoded random key")
        try:
            key = base64.b64decode(value, validate=True)
        except ValueError as exc:
            raise ValueError(f"{_KEY_ENV} must be valid base64") from exc
        return cls(key)

    def seal(self, scope: str, name: str, payload: bytes) -> bytes:
        timestamp = int(time.time()).to_bytes(_TIME_SIZE, "big")
        body = _MAGIC + timestamp + payload
        tag = hmac.new(self._key, self._context(scope, name) + body, hashlib.sha256).digest()
        return _MAGIC + timestamp + tag + payload

    def open(self, scope: str, name: str, payload: bytes) -> bytes:
        header_size = len(_MAGIC) + _TIME_SIZE + _MAC_SIZE
        if len(payload) < header_size or not payload.startswith(_MAGIC):
            raise ValueError("Unauthenticated Host RPC payload")
        timestamp_start = len(_MAGIC)
        timestamp_end = timestamp_start + _TIME_SIZE
        timestamp_bytes = payload[timestamp_start:timestamp_end]
        timestamp = int.from_bytes(timestamp_bytes, "big")
        if abs(time.time() - timestamp) > _MAX_AGE_SECONDS:
            raise ValueError("Host RPC payload has expired")
        tag = payload[timestamp_end:header_size]
        content = payload[header_size:]
        body = _MAGIC + timestamp_bytes + content
        expected = hmac.new(self._key, self._context(scope, name) + body, hashlib.sha256).digest()
        if not hmac.compare_digest(tag, expected):
            raise ValueError("Invalid Host RPC authentication tag")
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
