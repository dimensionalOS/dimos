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

"""Pre-flight health check for the manipulation benchmark.

Confirms the pick-and-place RPC server is up and that a scan detects objects,
before a (long) benchmark run. Run as a module::

    python -m dimos.manipulation.eval.health_check

With no live sim it prints a clean ``FAIL`` dict and exits 1 — never a traceback.
"""

from __future__ import annotations

import json
import re
import sys
import threading
import time
from typing import Any

from dimos.manipulation.eval.runner import normalize_skill_result
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Staging before the scan. The sim boots at qpos0 (camera at the ceiling) and
# go_init returns to that captured pose, so an un-staged scan sees nothing. We
# home to the observation pose, recapture it as the go_init target, then dwell so
# perception accumulates enough frames to promote objects to permanent.
_GO_HOME_RETRIES = 5
_SCAN_WARMUP_S = 3.0


def _probe_state(client: Any, timeout_s: float) -> str | None:
    """Call ``get_state()`` in a daemon thread; return the state or ``None`` on timeout."""
    result: dict[str, Any] = {}

    def _run() -> None:
        try:
            result["state"] = client.get_state()
        except Exception as exc:
            result["error"] = exc

    thread = threading.Thread(target=_run, name="health-check-probe", daemon=True)
    thread.start()
    thread.join(timeout_s)
    return result.get("state")


def _count_objects(message: str) -> int:
    """Parse the object count from a scan message ('Detected N object(s):')."""
    match = re.search(r"Detected\s+(\d+)\s+object", message)
    return int(match.group(1)) if match else 0


def run_health_check(timeout_s: float = 30.0) -> dict[str, Any]:
    """Probe the manipulation server and a single scan; return a status dict.

    Keys: ``status`` (OK | WARN | FAIL), ``module_state``, ``scan_success``,
    ``objects_detected``, ``scan_time_ms``, ``startup_time_ms``, ``warnings``.
    """
    start = time.perf_counter()
    warnings: list[str] = []

    try:
        from dimos.core.rpc_client import RPCClient
        from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
    except Exception as exc:
        return {
            "status": "FAIL",
            "reason": "import_error",
            "module_state": None,
            "scan_success": False,
            "objects_detected": 0,
            "scan_time_ms": 0.0,
            "startup_time_ms": round((time.perf_counter() - start) * 1000.0, 1),
            "warnings": [f"could not import manipulation stack: {exc!r}"],
        }

    client = RPCClient(None, PickAndPlaceModule)

    # Retry get_state() with ~1s backoff until the deadline.
    deadline = start + timeout_s
    state: str | None = None
    while time.perf_counter() < deadline:
        remaining = deadline - time.perf_counter()
        state = _probe_state(client, min(2.0, max(0.1, remaining)))
        if state is not None:
            break
        if time.perf_counter() < deadline:
            time.sleep(min(1.0, max(0.0, deadline - time.perf_counter())))

    startup_ms = round((time.perf_counter() - start) * 1000.0, 1)
    if state is None:
        return {
            "status": "FAIL",
            "reason": "no_response",
            "module_state": None,
            "scan_success": False,
            "objects_detected": 0,
            "scan_time_ms": 0.0,
            "startup_time_ms": startup_ms,
            "warnings": [f"no response from manipulation server within {timeout_s:.0f}s"],
        }

    # Stage the arm before scanning (mirror BenchmarkRunner.prepare()): home to the
    # observation pose, capture it as the go_init target, then dwell. Never raises —
    # a staging failure degrades to a warning so the dict stays clean.
    home_ok = False
    for _ in range(_GO_HOME_RETRIES):
        try:
            home = normalize_skill_result(client.go_home())
        except Exception as exc:
            warnings.append(f"go_home raised: {exc!r}")
            continue
        if home["success"]:
            home_ok = True
            break
    if home_ok:
        try:
            client.set_init_joints_to_current()
        except Exception as exc:
            warnings.append(f"set_init_joints_to_current failed: {exc!r}")
    else:
        warnings.append("go_home did not succeed; scanning from current pose")
    time.sleep(_SCAN_WARMUP_S)

    # Time a single scan and count detected objects.
    scan_t0 = time.perf_counter()
    try:
        scan = normalize_skill_result(client.scan_objects())
        scan_time_ms = round((time.perf_counter() - scan_t0) * 1000.0, 1)
        scan_success = bool(scan["success"])
        objects_detected = _count_objects(scan["message"]) if scan_success else 0
    except Exception as exc:
        scan_time_ms = round((time.perf_counter() - scan_t0) * 1000.0, 1)
        scan_success = False
        objects_detected = 0
        warnings.append(f"scan_objects raised: {exc!r}")

    if not scan_success:
        warnings.append("scan_objects did not succeed")
    elif objects_detected == 0:
        warnings.append("scan succeeded but detected 0 objects")

    status = "OK" if (scan_success and objects_detected > 0) else "WARN"
    return {
        "status": status,
        "module_state": state,
        "scan_success": scan_success,
        "objects_detected": objects_detected,
        "scan_time_ms": scan_time_ms,
        "startup_time_ms": startup_ms,
        "warnings": warnings,
    }


def main() -> int:
    print(
        "manipulation health check — ensure the sim is running:\n"
        "  dimos run xarm-perception-sim-agent\n",
        file=sys.stderr,
    )
    result = run_health_check()
    print(json.dumps(result, indent=2))
    return 0 if result["status"] in ("OK", "WARN") else 1


if __name__ == "__main__":
    sys.exit(main())
