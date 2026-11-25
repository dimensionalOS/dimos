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

"""Low-rate application probes; RTT is never misreported as a Wi-Fi-hop measurement."""

from __future__ import annotations

from dataclasses import asdict
import time
from typing import Any
from urllib.parse import urlsplit

import requests

from dimos.diagnostics.clock import clock_domain
from dimos.diagnostics.model import ClockMapping


def clock_reply() -> dict[str, Any]:
    received = time.monotonic_ns()
    return {
        "schema": "dimos.clock/v1",
        "clock_domain": clock_domain(),
        "received_ns": received,
        "sent_ns": time.monotonic_ns(),
    }


def exchange(session: requests.Session, url: str, timeout: float = 2.0) -> dict[str, Any]:
    """Probe the explicit URL. Authentication/TLS configuration belongs to the caller.

    Bounds include HTTP, proxy, OS scheduling and network time. Do not divide RTT
    by two: outbound and return paths need not be symmetric.
    """
    parsed = urlsplit(url)
    target = (
        f"{parsed.scheme}://{parsed.hostname}"
        + (f":{parsed.port}" if parsed.port else "")
        + parsed.path
    )
    domain = clock_domain()
    started = time.monotonic_ns()
    try:
        response = session.get(url, timeout=timeout, allow_redirects=False)
        ended = time.monotonic_ns()
        response.raise_for_status()
        if len(response.content) > 8192:
            raise ValueError("clock response exceeds limit")
        value = response.json()
        if value.get("schema") != "dimos.clock/v1":
            raise ValueError("unsupported clock probe")
        mapping = ClockMapping.exchange(
            domain,
            value["clock_domain"],
            started,
            value["received_ns"],
            value["sent_ns"],
            ended,
            valid_for_ns=5_000_000_000,
        )
        return {
            "event": "network.probe",
            "status": "ok",
            "clock_domain": domain,
            "mono_ns": ended,
            "rtt_ns": ended - started,
            "server_work_ns": value["sent_ns"] - value["received_ns"],
            "mapping": asdict(mapping),
            "scope": "HTTP application round trip",
            "target": target,
        }
    except (requests.RequestException, ValueError, KeyError, TypeError) as error:
        return {
            "event": "network.probe",
            "status": "error",
            "clock_domain": domain,
            "mono_ns": time.monotonic_ns(),
            "reason": type(error).__name__,
        }
