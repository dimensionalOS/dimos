# Copyright 2025-2026 Dimensional Inc.
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

"""No-movement Go2 LAN verification helpers."""

from __future__ import annotations

import base64
import binascii
from dataclasses import dataclass
import json
import time
from typing import Any, Protocol

import requests

CON_NOTIFY_PORT = 9991
CON_NOTIFY_PATH = "/con_notify"
DEFAULT_VERIFY_TIMEOUT_S = 1.0


class HttpResponse(Protocol):
    status_code: int
    reason: str | None
    text: str


class HttpPost(Protocol):
    def __call__(
        self,
        url: str,
        *,
        timeout: float,
        allow_redirects: bool,
    ) -> HttpResponse: ...


@dataclass(frozen=True)
class Go2VerifyStatus:
    """Structured result for probing a Go2 WebRTC signaling endpoint."""

    robot_ip: str
    url: str
    ok: bool
    status_code: int | None
    reason: str | None
    error: str | None
    elapsed_s: float
    timeout_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot_ip": self.robot_ip,
            "url": self.url,
            "ok": self.ok,
            "status_code": self.status_code,
            "reason": self.reason,
            "error": self.error,
            "elapsed_s": self.elapsed_s,
            "timeout_s": self.timeout_s,
        }

    def summary_lines(self) -> tuple[str, ...]:
        if self.ok:
            return (f"Go2 WebRTC endpoint reachable at {self.url} (HTTP {self.status_code}).",)
        if self.status_code is not None:
            if self.error:
                return (
                    f"Go2 WebRTC endpoint responded at {self.url} "
                    f"with HTTP {self.status_code} {self.reason or ''}: {self.error}".rstrip()
                    + ".",
                )
            return (
                f"Go2 WebRTC endpoint responded at {self.url} "
                f"with HTTP {self.status_code} {self.reason or ''}".rstrip()
                + ".",
            )
        return (f"Go2 WebRTC endpoint unreachable at {self.url}: {self.error}",)


def con_notify_url(robot_ip: str) -> str:
    """Build the Go2 WebRTC signaling health probe URL."""
    return f"http://{robot_ip.strip()}:{CON_NOTIFY_PORT}{CON_NOTIFY_PATH}"


def _con_notify_payload_error(text: str) -> str | None:
    try:
        decoded = base64.b64decode(text, validate=True).decode("utf-8")
        payload = json.loads(decoded)
    except (binascii.Error, UnicodeDecodeError, json.JSONDecodeError):
        return "unexpected con_notify response payload"

    if not isinstance(payload, dict):
        return "unexpected con_notify response payload"
    data1 = payload.get("data1")
    if not isinstance(data1, str) or not data1:
        return "unexpected con_notify response payload"
    return None


def verify_robot_ip(
    robot_ip: str,
    *,
    timeout_s: float = DEFAULT_VERIFY_TIMEOUT_S,
    http_post: HttpPost | None = None,
) -> Go2VerifyStatus:
    """Verify that a Go2 is reachable on LAN without sending movement commands.

    This only performs an HTTP POST probe against the robot's WebRTC signaling
    endpoint. It does not open a robot control session or publish commands.
    """
    url = con_notify_url(robot_ip)
    post = http_post or requests.post
    started = time.monotonic()
    try:
        response = post(url, timeout=timeout_s, allow_redirects=False)
    except requests.RequestException as e:
        return Go2VerifyStatus(
            robot_ip=robot_ip,
            url=url,
            ok=False,
            status_code=None,
            reason=None,
            error=str(e),
            elapsed_s=time.monotonic() - started,
            timeout_s=timeout_s,
        )

    status_code = response.status_code
    error = None
    if 200 <= status_code < 300:
        error = _con_notify_payload_error(response.text)
    return Go2VerifyStatus(
        robot_ip=robot_ip,
        url=url,
        ok=200 <= status_code < 300 and error is None,
        status_code=status_code,
        reason=response.reason or None,
        error=error,
        elapsed_s=time.monotonic() - started,
        timeout_s=timeout_s,
    )
