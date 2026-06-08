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

from dataclasses import dataclass
import time
from typing import Any, Protocol

import requests

CON_NOTIFY_PORT = 9991
CON_NOTIFY_PATH = "/con_notify"
DEFAULT_VERIFY_TIMEOUT_S = 1.0


class HttpGet(Protocol):
    def __call__(self, url: str, *, timeout: float) -> requests.Response: ...


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
            return (
                f"Go2 WebRTC endpoint responded at {self.url} "
                f"with HTTP {self.status_code} {self.reason or ''}".rstrip()
                + ".",
            )
        return (f"Go2 WebRTC endpoint unreachable at {self.url}: {self.error}",)


def con_notify_url(robot_ip: str) -> str:
    """Build the Go2 WebRTC signaling health probe URL."""
    return f"http://{robot_ip.strip()}:{CON_NOTIFY_PORT}{CON_NOTIFY_PATH}"


def verify_robot_ip(
    robot_ip: str,
    *,
    timeout_s: float = DEFAULT_VERIFY_TIMEOUT_S,
    http_get: HttpGet | None = None,
) -> Go2VerifyStatus:
    """Verify that a Go2 is reachable on LAN without sending movement commands.

    This only performs an HTTP GET probe against the robot's WebRTC signaling
    endpoint. It does not open a robot control session or publish commands.
    """
    url = con_notify_url(robot_ip)
    get = http_get or requests.get
    started = time.monotonic()
    try:
        response = get(url, timeout=timeout_s)
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
    return Go2VerifyStatus(
        robot_ip=robot_ip,
        url=url,
        ok=200 <= status_code < 400,
        status_code=status_code,
        reason=response.reason or None,
        error=None,
        elapsed_s=time.monotonic() - started,
        timeout_s=timeout_s,
    )
