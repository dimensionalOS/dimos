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

"""Read one process resource snapshot from inside a benchmark container."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import psutil


def snapshot(process_name: str) -> dict[str, float | int]:
    process = next(
        process for process in psutil.process_iter(["name"]) if process.info["name"] == process_name
    )
    cpu = process.cpu_times()
    switches = process.num_ctx_switches()
    peak_rss = process.memory_info().rss
    for line in Path(f"/proc/{process.pid}/status").read_text().splitlines():
        if line.startswith("VmHWM:"):
            peak_rss = int(line.split()[1]) * 1024
            break
    return {
        "cpu_seconds": cpu.user + cpu.system,
        "peak_rss_bytes": peak_rss,
        "voluntary_context_switches": switches.voluntary,
        "involuntary_context_switches": switches.involuntary,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("process_name")
    args = parser.parse_args()
    print(json.dumps(snapshot(args.process_name), sort_keys=True))


if __name__ == "__main__":
    main()
