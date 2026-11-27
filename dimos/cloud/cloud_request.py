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

from collections.abc import Callable, Iterable, Iterator
import json
from pathlib import Path
import re
import shutil
import threading
from typing import Any, Protocol, cast
import urllib.error
import urllib.request

import requests

# split each chunk into blocks of this size
PUT_BLOCK = 1 << 20


class CloudRequest(Protocol):
    def request(
        self, method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]: ...
    def put(self, url: str, body: bytes | memoryview) -> None: ...
    def download(
        self, url: str, dst: Path, progress: Callable[[int, int], None] | None = None
    ) -> None: ...


class _Blocks:
    """A PUT body that requests can size and send one block at a time."""

    def __init__(self, body: bytes | memoryview) -> None:
        self.view = memoryview(body)

    def __len__(self) -> int:
        return len(self.view)

    def __iter__(self) -> Iterator[memoryview]:
        # split the body in to blocks so each gets its own timeout
        return (self.view[i : i + PUT_BLOCK] for i in range(0, len(self.view), PUT_BLOCK))


class HttpCloudRequest:
    """urllib + Bearer key. 401 and network faults surface as RuntimeError with a
    message the CLI can print verbatim."""

    def __init__(self, base_url: str, key: str, timeout: float) -> None:
        self.base_url, self.key, self.timeout = base_url.rstrip("/"), key, timeout
        self.put_sessions = threading.local()

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

    def put(self, url: str, body: bytes | memoryview) -> None:
        # one session per thread, so an upload worker sends all its parts over one connection
        session = getattr(self.put_sessions, "session", None)
        if session is None:
            session = self.put_sessions.session = requests.Session()
        try:
            r = session.put(
                url,
                data=cast("Iterable[bytes]", _Blocks(body)),
                # the default form content-type makes some S3-compatible servers parse the body
                headers={"Content-Type": "application/octet-stream"},
                timeout=self.timeout,
            )
        except requests.RequestException as e:
            # requests puts the signed URL in its message, so report the error underneath
            cause: BaseException = e
            while cause.__context__ is not None:
                cause = cause.__context__
            raise OSError(f"PUT failed: {cause}") from e
        if not r.ok:
            # S3 names the reason in an XML body; the rest of it can hold signing details
            code = re.search(r"<Code>(.*?)</Code>", r.text)
            message = re.search(r"<Message>(.*?)</Message>", r.text)
            detail = f" ({code[1]}: {message[1] if message else ''})"[:300] if code else ""
            raise OSError(f"HTTP Error {r.status_code}: {r.reason}{detail}")

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
