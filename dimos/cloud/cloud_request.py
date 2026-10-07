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

"""How bytes and JSON reach the cloud. Swap the implementation, keep the callers."""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
import shutil
from typing import Any, Protocol, cast
import urllib.error
import urllib.request


class CloudRequest(Protocol):
    def request(
        self, method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]: ...
    def put(self, url: str, body: bytes) -> None: ...
    def download(
        self, url: str, dst: Path, progress: Callable[[int, int], None] | None = None
    ) -> None: ...


class HttpCloudRequest:
    """urllib + Bearer key. 401 and network faults surface as RuntimeError with a
    message the CLI can print verbatim."""

    def __init__(self, base_url: str, key: str, timeout: float) -> None:
        self.base_url, self.key, self.timeout = base_url.rstrip("/"), key, timeout

    def request(self, method: str, path: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        req = urllib.request.Request(
            self.base_url + path,
            data=json.dumps(body).encode() if body is not None else None,
            method=method,
            headers={"Authorization": f"Bearer {self.key}", "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return cast("dict[str, Any]", json.load(r))
        except urllib.error.HTTPError as e:
            if e.code == 401:
                raise RuntimeError("API key invalid or revoked — run `dimos login`") from e
            raise RuntimeError(f"{method} {path}: {e.code} {e.read().decode()[:300]}") from e
        except (urllib.error.URLError, TimeoutError) as e:
            raise RuntimeError(f"{method} {path}: {e}") from e

    def put(self, url: str, body: bytes) -> None:
        with urllib.request.urlopen(
            urllib.request.Request(url, data=body, method="PUT"), timeout=self.timeout
        ):
            pass

    def download(
        self, url: str, dst: Path, progress: Callable[[int, int], None] | None = None
    ) -> None:
        with urllib.request.urlopen(url, timeout=self.timeout) as r, dst.open("wb") as f:
            if progress is None:
                shutil.copyfileobj(r, f)
                return
            total = int(r.headers.get("Content-Length") or 0)
            done = 0
            while chunk := r.read(1 << 20):
                f.write(chunk)
                done += len(chunk)
                progress(done, total)
