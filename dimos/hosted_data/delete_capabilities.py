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

"""Hashed, per-object deletion capabilities for anonymous uploaders."""

from __future__ import annotations

import hashlib
import hmac
import json
import os
from pathlib import Path
import secrets
import tempfile
from threading import Lock
from typing import Protocol

class DeleteAccessPolicy(Protocol):
    """Small policy surface required for administrator deletion."""

    def authorize(
        self,
        token: str,
        *,
        mode: str,
        owner: str,
        repository: str,
    ) -> bool: ...

class DeleteCapabilityStore:
    """Persist hashed delete capabilities without storing bearer secrets."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._lock = Lock()

    def _path(self, owner: str, repository: str, object_id: str) -> Path:
        return self.root / owner / repository / f"{object_id}.json"

    def issue(self, owner: str, repository: str, object_id: str) -> str:
        """Create a new capability, retaining at most eight valid digests."""
        token = secrets.token_urlsafe(32)
        digest = hashlib.sha256(token.encode()).hexdigest()
        path = self._path(owner, repository, object_id)
        with self._lock:
            digests: list[str] = []
            if path.is_file():
                raw = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(raw, list):
                    digests = [
                        str(item) for item in raw if isinstance(item, str) and len(item) == 64
                    ]
            digests = ([*digests, digest])[-8:]
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary: Path | None = None
            try:
                with tempfile.NamedTemporaryFile(
                    mode="w",
                    encoding="utf-8",
                    prefix=f".{object_id}.",
                    suffix=".json.part",
                    dir=path.parent,
                    delete=False,
                ) as target:
                    temporary = Path(target.name)
                    json.dump(digests, target, sort_keys=True)
                os.replace(temporary, path)
                temporary = None
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
        return token

    def verify(self, owner: str, repository: str, object_id: str, token: str) -> bool:
        """Return whether a presented capability belongs to the object."""
        if not token or len(token) > 256:
            return False
        path = self._path(owner, repository, object_id)
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return False
        if not isinstance(raw, list):
            return False
        supplied = hashlib.sha256(token.encode()).hexdigest()
        return any(isinstance(item, str) and hmac.compare_digest(supplied, item) for item in raw)

    def authorize(
        self,
        *,
        owner: str,
        repository: str,
        object_id: str,
        authorization: str,
        capability: str,
        admin_token: str | None,
        access_policy: DeleteAccessPolicy | None,
    ) -> bool:
        """Authorize an administrator or the anonymous uploader capability."""
        bearer = (
            authorization.removeprefix("Bearer ")
            if authorization.startswith("Bearer ")
            else ""
        )
        if admin_token is not None and hmac.compare_digest(
            authorization,
            f"Bearer {admin_token}",
        ):
            return True
        if bearer and access_policy is not None and access_policy.authorize(
            bearer,
            mode="write",
            owner=owner,
            repository=repository,
        ):
            return True
        return self.verify(owner, repository, object_id, capability)
    def revoke(self, owner: str, repository: str, object_id: str) -> None:
        """Remove all capabilities after an object is deleted."""
        with self._lock:
            self._path(owner, repository, object_id).unlink(missing_ok=True)
