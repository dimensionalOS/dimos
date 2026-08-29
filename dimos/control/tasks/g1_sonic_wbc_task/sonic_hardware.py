# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Read-only Jetson performance checks required before SONIC control."""

from __future__ import annotations

import re
import subprocess


def _output(command: list[str]) -> str:
    try:
        return subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"could not run {' '.join(command)}: {exc}") from exc


def ensure_sonic_max_performance() -> None:
    """Fail unless the Jetson is in MAXN with CPU/GPU clocks locked."""
    nvpmodel = _output(["nvpmodel", "-q"])
    if "NV Power Mode: MAXN" not in nvpmodel:
        raise RuntimeError("SONIC requires Jetson MAXN mode. Run `sudo nvpmodel -m 0`, then retry.")

    clocks = _output(["jetson_clocks", "--show"])
    cpu_matches = re.findall(r"cpu\d+[^\n]*MinFreq=(\d+)[^\n]*MaxFreq=(\d+)", clocks, re.IGNORECASE)
    gpu_match = re.search(r"GPU[^\n]*MinFreq=(\d+)[^\n]*MaxFreq=(\d+)", clocks, re.IGNORECASE)
    unlocked: list[str] = []
    if not cpu_matches or any(minimum != maximum for minimum, maximum in cpu_matches):
        unlocked.append("CPU")
    if gpu_match is None or gpu_match.group(1) != gpu_match.group(2):
        unlocked.append("GPU")
    if unlocked:
        raise RuntimeError(
            "SONIC requires locked Jetson clocks for CPU/GPU. Run `sudo jetson_clocks`, "
            f"then retry (unlocked: {', '.join(unlocked)})."
        )
