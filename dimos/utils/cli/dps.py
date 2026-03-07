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

"""dps — Non-interactive process list over LCM (like `docker ps`).

Waits for one dtop resource_stats message and prints a table.

Usage:
    dps [--topic /dimos/resource_stats] [--timeout 5]
"""

from __future__ import annotations

import sys
import threading
from typing import Any

from rich.console import Console
from rich.table import Table

from dimos.protocol.pubsub.impl.lcmpubsub import PickleLCM, Topic


def _fmt_pct(v: float) -> str:
    return f"{v:.0f}%"


def _fmt_mem(v: float) -> str:
    mb = v / 1048576
    if mb >= 1024:
        return f"{mb / 1024:.1f}G"
    return f"{mb:.0f}M"


def _fmt_secs(v: float) -> str:
    if v >= 3600:
        return f"{v / 3600:.1f}h"
    if v >= 60:
        return f"{v / 60:.1f}m"
    return f"{v:.1f}s"


def ps(topic: str = "/dimos/resource_stats", timeout: float = 5.0) -> None:
    """Wait for one LCM message and print a process table."""
    lcm = PickleLCM(autoconf=True)
    result: dict[str, Any] = {}
    event = threading.Event()

    def on_msg(msg: dict[str, Any], _topic: str) -> None:
        nonlocal result
        result = msg
        event.set()

    lcm.subscribe(Topic(topic), on_msg)
    lcm.start()

    if not event.wait(timeout):
        lcm.stop()
        Console(stderr=True).print(
            f"[red]No dtop message within {timeout:.0f}s. Is --dtop enabled?[/red]"
        )
        sys.exit(1)

    lcm.stop()

    table = Table(show_header=True, header_style="bold", padding=(0, 1))
    table.add_column("PID", style="dim")
    table.add_column("Role")
    table.add_column("Modules")
    table.add_column("CPU", justify="right")
    table.add_column("Mem", justify="right")
    table.add_column("Threads", justify="right")
    table.add_column("FDs", justify="right")
    table.add_column("User", justify="right")
    table.add_column("Sys", justify="right")

    coord = result.get("coordinator", {})
    table.add_row(
        str(coord.get("pid", "")),
        "[cyan]coordinator[/cyan]",
        "",
        _fmt_pct(coord.get("cpu_percent", 0)),
        _fmt_mem(coord.get("pss", 0)),
        str(int(coord.get("num_threads", 0))),
        str(int(coord.get("num_fds", 0))),
        _fmt_secs(coord.get("cpu_time_user", 0)),
        _fmt_secs(coord.get("cpu_time_system", 0)),
    )

    for w in result.get("workers", []):
        alive = w.get("alive", False)
        wid = w.get("worker_id", "?")
        role_style = "green" if alive else "red"
        modules = ", ".join(w.get("modules", []))
        table.add_row(
            str(w.get("pid", "")),
            f"[{role_style}]worker {wid}[/{role_style}]",
            modules,
            _fmt_pct(w.get("cpu_percent", 0)),
            _fmt_mem(w.get("pss", 0)),
            str(int(w.get("num_threads", 0))),
            str(int(w.get("num_fds", 0))),
            _fmt_secs(w.get("cpu_time_user", 0)),
            _fmt_secs(w.get("cpu_time_system", 0)),
        )

    Console().print(table)


def main() -> None:
    topic = "/dimos/resource_stats"
    timeout = 5.0
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--topic" and i + 1 < len(args):
            topic = args[i + 1]
            i += 2
        elif args[i] == "--timeout" and i + 1 < len(args):
            timeout = float(args[i + 1])
            i += 2
        else:
            i += 1
    ps(topic=topic, timeout=timeout)


if __name__ == "__main__":
    main()
