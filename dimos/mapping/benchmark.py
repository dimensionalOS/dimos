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

"""Run an odometry method over a recording and measure whether it could keep up.

    python -m dimos.mapping.benchmark <recording_dir> --method cuvslam

The question these numbers answer is not "how accurate is it" but "could this
have run on the robot". So the headline is the real-time factor -- wall clock
divided by the recording's own duration -- and anything at or under 1.0 could
have kept up with the sensors on this machine, with 1/rtf of headroom.

Machine state is recorded next to every result on purpose. This box ships with a
powersave governor pinning the CPU near 1.1 GHz of 4.9 and the GPU under a soft
power cap, and neither the fix nor the throttle survives a reboot. A real-time
factor measured in one state means nothing in the other, so the state travels
with the measurement rather than living in someone's memory.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time
from typing import Any

import psutil

SAMPLE_INTERVAL_S = 0.5
# nvidia-smi is shelled out to rather than adding a dependency on pynvml, and it
# is polled on the same tick as the CPU sampler.
GPU_QUERY = "utilization.gpu,memory.used,clocks.sm,power.draw"


def machine_state() -> dict[str, Any]:
    """Clock and governor state, so a number measured here can be read later."""
    governors = set()
    for path in Path("/sys/devices/system/cpu").glob("cpu*/cpufreq/scaling_governor"):
        governors.add(path.read_text().strip())
    state: dict[str, Any] = {
        "cpu_count": psutil.cpu_count(logical=True),
        "cpu_governor": sorted(governors) or ["unknown"],
        "cpu_mhz_max": round(psutil.cpu_freq().max) if psutil.cpu_freq() else None,
    }
    gpu = _nvidia_smi("name,clocks.max.sm,memory.total")
    if gpu:
        state["gpu"] = gpu[0]
    return state


def _nvidia_smi(query: str) -> list[list[str]]:
    if shutil.which("nvidia-smi") is None:
        return []
    try:
        output = subprocess.run(
            ["nvidia-smi", f"--query-gpu={query}", "--format=csv,noheader,nounits"],
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        ).stdout
    except (subprocess.SubprocessError, OSError):
        return []
    return [[field.strip() for field in line.split(",")] for line in output.splitlines() if line]


def measure(
    command: list[str], cwd: Path | None = None, log_path: Path | None = None
) -> dict[str, Any]:
    """Run ``command`` to completion, sampling its whole process tree."""
    started = time.monotonic()
    process = subprocess.Popen(
        command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    )
    parent = psutil.Process(process.pid)

    cpu_samples: list[float] = []
    rss_samples: list[float] = []
    gpu_util: list[float] = []
    gpu_mem: list[float] = []
    output: list[str] = []
    assert process.stdout is not None

    def sample() -> None:
        try:
            family = [parent, *parent.children(recursive=True)]
            cpu_samples.append(sum(p.cpu_percent(None) for p in family))
            rss_samples.append(sum(p.memory_info().rss for p in family) / 1e9)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return
        reading = _nvidia_smi(GPU_QUERY)
        if reading:
            gpu_util.append(float(reading[0][0]))
            gpu_mem.append(float(reading[0][1]) / 1000.0)

    # Sampling runs on its own thread. Driving it from the stdout loop instead
    # ties the sample rate to how chatty the child is, and a quiet child yields a
    # handful of samples over several minutes -- an average of nothing.
    sample()  # priming call; psutil's first cpu_percent always reads 0
    done = threading.Event()

    def sampler() -> None:
        while not done.wait(SAMPLE_INTERVAL_S):
            sample()

    sampler_thread = threading.Thread(target=sampler, daemon=True)
    sampler_thread.start()
    for line in process.stdout:
        output.append(line)
    process.wait()
    done.set()
    sampler_thread.join(timeout=2.0)
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text("".join(output))
    wall = time.monotonic() - started

    # The priming sample is always zero, so it is dropped rather than averaged in.
    cpu = cpu_samples[1:] or cpu_samples
    return {
        "command": " ".join(command),
        "exit_code": process.returncode,
        "wall_s": round(wall, 1),
        "cpu_mean_pct": round(sum(cpu) / len(cpu), 1) if cpu else None,
        "cpu_peak_pct": round(max(cpu), 1) if cpu else None,
        "rss_mean_gb": round(sum(rss_samples) / len(rss_samples), 2) if rss_samples else None,
        "rss_peak_gb": round(max(rss_samples), 2) if rss_samples else None,
        "gpu_util_mean_pct": round(sum(gpu_util) / len(gpu_util), 1) if gpu_util else None,
        "gpu_mem_peak_gb": round(max(gpu_mem), 2) if gpu_mem else None,
        "samples": len(cpu),
        "output_tail": "".join(output[-15:]),
    }


FEED_LINE = re.compile(r"(\d+)/(\d+) fed, (\d+) poses, (\d+)s")


def feed_rates(log_text: str) -> dict[str, Any]:
    """Split the replay's own feed loop out of the total wall clock.

    The loop is timed from after every image is pulled from sqlite into RAM, so
    its rate excludes the preload -- which is the largest thing in the total that
    a live camera would never pay. Still includes the LCM round trip, so it is an
    upper bound on the tracker's own cost, not the tracker's cost.
    """
    matches = FEED_LINE.findall(log_text)
    if not matches:
        return {}
    fed, total, poses, seconds = (int(v) for v in matches[-1])
    if seconds <= 0:
        return {}
    return {
        "feed_fps": round(fed / seconds, 1),
        "feed_pose_yield": round(poses / fed, 3) if fed else None,
        "pairs_total": total,
    }


def recording_duration_s(db_path: Path) -> float:
    """Span of the recording, for the real-time factor."""
    import sqlite3

    connection = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    spans = []
    for (name,) in connection.execute("SELECT name FROM _streams"):
        row = connection.execute(f'SELECT MIN(ts), MAX(ts) FROM "{name}"').fetchone()
        if row and row[0] is not None:
            spans.append(row[1] - row[0])
    connection.close()
    return round(max(spans), 1) if spans else 0.0


def record(recording_dir: Path, method: str, result: dict[str, Any], duration: float) -> None:
    """Merge one method's result into the recording's single stats file."""
    stats_path = recording_dir / "stats.json"
    stats = json.loads(stats_path.read_text()) if stats_path.exists() else {}
    stats.setdefault("recording", recording_dir.name)
    stats["duration_s"] = duration
    stats["machine"] = machine_state()
    result["real_time_factor"] = round(result["wall_s"] / duration, 2) if duration else None
    result["keeps_up"] = bool(duration and result["wall_s"] <= duration)
    stats.setdefault("methods", {})[method] = result
    stats_path.write_text(json.dumps(stats, indent=2) + "\n")


# The cuVSLAM entry pointed at demo_cuvslam_replay, which drove the tracker through the
# dimos module and froze partway through every recording -- so what it timed was the
# transport, not cuVSLAM. Timing now comes from the standalone harness instead:
#     python -m dimos.mapping.cuvslam_replay_export <db> <dir>
#     bench_cuvslam <dir>            # prints track ms and real-time factor
METHODS: dict[str, Callable[[Path, Path], list[str]]] = {}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording_dir", type=Path)
    parser.add_argument("--method", required=True, choices=sorted(METHODS))
    args = parser.parse_args()

    db = args.recording_dir / f"{args.recording_dir.name}.db"
    if not db.exists():
        print(f"benchmark: no recording at {db}", file=sys.stderr)
        return 1

    duration = recording_duration_s(db)
    work_dir = args.recording_dir / f".{args.method}"
    result = measure(
        METHODS[args.method](db, work_dir),
        cwd=Path.home() / "repos/dimos4",
        log_path=work_dir / "run.log",
    )
    log = work_dir / "run.log"
    if log.exists():
        result.update(feed_rates(log.read_text()))
    record(args.recording_dir, args.method, result, duration)

    trajectory = work_dir / "trajectory.npy"
    if trajectory.exists():
        shutil.copy2(trajectory, args.recording_dir / f"{args.method}_traj.npy")
    corrected = work_dir / "corrected.npy"
    if corrected.exists():
        shutil.copy2(corrected, args.recording_dir / f"{args.method}_corrected_traj.npy")

    print(json.dumps({k: v for k, v in result.items() if k != "output_tail"}, indent=2))
    exit_code: int = result["exit_code"]
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
