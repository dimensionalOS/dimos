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

"""Repository-scoped access control and expiring download signatures."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import hmac
import json
from pathlib import Path
import time
from typing import Any, Literal
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

AccessMode = Literal["read", "write"]


def _token_digest(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


def _matches_repository(pattern: str, owner: str, repository: str) -> bool:
    try:
        pattern_owner, pattern_repository = pattern.split("/", 1)
    except ValueError:
        return False
    return pattern_owner in {"*", owner} and pattern_repository in {"*", repository}


@dataclass(frozen=True)
class RepositoryPrincipal:
    """One bearer-token identity with repository-scoped permissions."""

    name: str
    token_sha256: str
    read: tuple[str, ...] = ()
    write: tuple[str, ...] = ()

    def permits(self, mode: AccessMode, owner: str, repository: str) -> bool:
        patterns = self.write if mode == "write" else (*self.read, *self.write)
        return any(_matches_repository(pattern, owner, repository) for pattern in patterns)


class RepositoryAccessPolicy:
    """Static token policy loaded from a root-readable JSON file."""

    def __init__(self, principals: tuple[RepositoryPrincipal, ...]) -> None:
        self.principals = principals

    @classmethod
    def from_file(cls, path: str | Path) -> RepositoryAccessPolicy:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("principals"), list):
            raise ValueError("ACL file must contain a principals array")
        principals: list[RepositoryPrincipal] = []
        for raw in data["principals"]:
            if not isinstance(raw, dict):
                raise ValueError("every ACL principal must be an object")
            name = raw.get("name")
            digest = raw.get("token_sha256")
            read = raw.get("read", [])
            write = raw.get("write", [])
            if (
                not isinstance(name, str)
                or not name
                or not isinstance(digest, str)
                or len(digest) != 64
                or not all(character in "0123456789abcdef" for character in digest)
                or not isinstance(read, list)
                or not isinstance(write, list)
                or not all(isinstance(item, str) for item in (*read, *write))
            ):
                raise ValueError("invalid ACL principal")
            principals.append(
                RepositoryPrincipal(
                    name=name,
                    token_sha256=digest,
                    read=tuple(read),
                    write=tuple(write),
                )
            )
        return cls(tuple(principals))

    def authorize(
        self,
        token: str,
        *,
        mode: AccessMode,
        owner: str,
        repository: str,
    ) -> bool:
        supplied = _token_digest(token)
        permitted = False
        for principal in self.principals:
            token_matches = hmac.compare_digest(supplied, principal.token_sha256)
            permitted |= token_matches and principal.permits(mode, owner, repository)
        return permitted


def _signature_payload(path: str, expires: int) -> bytes:
    return f"GET\n{path}\n{expires}".encode()


def sign_download_path(path: str, *, secret: str, expires: int) -> str:
    """Return a hex HMAC for one object path and absolute expiry timestamp."""
    if not path.startswith("/") or "?" in path or "#" in path:
        raise ValueError("signed path must be an absolute URL path without query or fragment")
    if expires <= 0:
        raise ValueError("expires must be a positive Unix timestamp")
    return hmac.new(secret.encode(), _signature_payload(path, expires), hashlib.sha256).hexdigest()


def verify_download_signature(
    path: str,
    *,
    secret: str,
    expires: str | None,
    signature: str | None,
    now: float | None = None,
) -> bool:
    """Validate an unexpired object download signature."""
    if expires is None or signature is None:
        return False
    try:
        expiry = int(expires)
    except ValueError:
        return False
    if expiry < int(time.time() if now is None else now):
        return False
    expected = sign_download_path(path, secret=secret, expires=expiry)
    return hmac.compare_digest(signature, expected)


def create_signed_download_url(
    url: str,
    *,
    secret: str,
    expires_in_seconds: int = 3600,
    now: float | None = None,
) -> str:
    """Attach an expiring read-only HMAC signature to an object URL."""
    if expires_in_seconds < 1:
        raise ValueError("expires_in_seconds must be positive")
    parsed = urlsplit(url)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError("download URL must be an absolute HTTP(S) URL")
    expiry = int(time.time() if now is None else now) + expires_in_seconds
    signature = sign_download_path(parsed.path, secret=secret, expires=expiry)
    query: dict[str, Any] = dict(parse_qsl(parsed.query, keep_blank_values=True))
    query.update({"expires": str(expiry), "signature": signature})
    return urlunsplit(
        (parsed.scheme, parsed.netloc, parsed.path, urlencode(query), parsed.fragment)
    )
