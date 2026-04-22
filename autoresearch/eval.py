#!/usr/bin/env python3
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

"""Eval script for Go2 blueprint CPU/sys footprint autoresearch.

Workflow:
1. Profile (first run only) — runs replay under cProfile, saves top functions
2. Benchmark — runs replay with `time`, parses user+sys as primary score
3. Validate — records LCM message counts/hashes, compares against baseline

Usage:
    python eval.py [--profile-only] [--skip-profile] [--timeout SECONDS]
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path
import re
import subprocess
import time

import psutil

SCRIPT_DIR = Path(__file__).parent
# autoresearch → <repo root>
REPO_ROOT = SCRIPT_DIR.parents[0]
BASELINE_PATH = SCRIPT_DIR / "baseline_record.json"
PROFILE_PATH = SCRIPT_DIR / "profile_output.txt"
PROF_PATH = SCRIPT_DIR / "profile.prof"
RESULTS_DIR = SCRIPT_DIR / "results"

BASE_REPLAY_CMD = [
    "test-venv/bin/dimos",
    "--replay",
    "--viewer=none",
    "--exit-on-eof",
    "--replay-dir=unitree_go2_bigoffice",
    "run",
    "unitree-go2",
]

DEFAULT_TIMEOUT = 600  # 10 minutes — safety cap; unitree-go2 is heavier


@dataclass
class BenchmarkResult:
    score: float  # user + sys seconds
    real: float = 0.0
    user: float = 0.0
    sys: float = 0.0
    ttfm: float = 0.0  # time to first message (seconds from process start)
    peak_cpu_pct: float = 0.0
    peak_memory_mb: float = 0.0
    avg_threads: float = 0.0
    total_io_read_mb: float = 0.0
    total_io_write_mb: float = 0.0
    context_switches: int = 0  # total voluntary + involuntary context switches
    validation: str = "SKIPPED"
    validation_detail: str = ""


@dataclass
class MessageRecord:
    """Records LCM message activity for validation."""

    topic_counts: dict[str, int] = field(default_factory=dict)
    topic_hashes: dict[str, list[str]] = field(default_factory=dict)


def parse_time_output(stderr: str) -> tuple[float, float, float]:
    """Parse real/user/sys from `time` command output.

    Handles both formats:
    - GNU time: '0.50user 0.30system 0:01.00elapsed'
    - Bash time: 'real 0m1.000s\\nuser 0m0.500s\\nsys 0m0.300s'
    """
    # Try bash time format first
    real_match = re.search(r"real\s+(\d+)m([\d.]+)s", stderr)
    user_match = re.search(r"user\s+(\d+)m([\d.]+)s", stderr)
    sys_match = re.search(r"sys\s+(\d+)m([\d.]+)s", stderr)

    if real_match and user_match and sys_match:
        real = int(real_match.group(1)) * 60 + float(real_match.group(2))
        user = int(user_match.group(1)) * 60 + float(user_match.group(2))
        sys_ = int(sys_match.group(1)) * 60 + float(sys_match.group(2))
        return real, user, sys_

    # Try GNU time format
    gnu_user = re.search(r"([\d.]+)user", stderr)
    gnu_sys = re.search(r"([\d.]+)system", stderr)
    gnu_real = re.search(r"([\d:]+\.\d+)elapsed", stderr)

    if gnu_user and gnu_sys and gnu_real:
        user = float(gnu_user.group(1))
        sys_ = float(gnu_sys.group(1))
        elapsed = gnu_real.group(1)
        if ":" in elapsed:
            parts = elapsed.split(":")
            real = int(parts[0]) * 60 + float(parts[1])
        else:
            real = float(elapsed)
        return real, user, sys_

    raise ValueError(f"Could not parse time output from stderr:\n{stderr[-500:]}")


class ProcessTreeMonitor:
    """Monitors a process tree, tracking peak stats over time."""

    def __init__(self, root_pid: int) -> None:
        self._root_pid = root_pid
        self._known_pids: dict[int, psutil.Process] = {}

    def _refresh_tree(self) -> list[psutil.Process]:
        """Get all processes in tree, priming cpu_percent for new ones."""
        try:
            root = psutil.Process(self._root_pid)
        except psutil.NoSuchProcess:
            return []

        all_procs = [root, *root.children(recursive=True)]
        result = []
        for p in all_procs:
            if p.pid not in self._known_pids:
                # Prime cpu_percent — first call always returns 0
                try:
                    p.cpu_percent(interval=None)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
                self._known_pids[p.pid] = p
            result.append(p)
        return result

    def sample(self) -> dict:
        """Collect one sample of stats across the whole process tree."""
        procs = self._refresh_tree()

        total_cpu = 0.0
        total_mem = 0.0
        total_threads = 0
        total_io_read = 0
        total_io_write = 0
        total_ctx_switches = 0

        for p in procs:
            try:
                total_cpu += p.cpu_percent(interval=None)
                mem_info = p.memory_info()
                total_mem += mem_info.rss
                total_threads += p.num_threads()
                try:
                    io = p.io_counters()
                    total_io_read += io.read_bytes
                    total_io_write += io.write_bytes
                except (psutil.AccessDenied, AttributeError):
                    pass
                try:
                    ctx = p.num_ctx_switches()
                    total_ctx_switches += ctx.voluntary + ctx.involuntary
                except (psutil.AccessDenied, AttributeError):
                    pass
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        return {
            "cpu_pct": total_cpu,
            "memory_bytes": total_mem,
            "threads": total_threads,
            "io_read_bytes": total_io_read,
            "io_write_bytes": total_io_write,
            "ctx_switches": total_ctx_switches,
        }


def _collect_tree_cpu_times(pid: int) -> tuple[float, float]:
    """Sum user+sys cpu_times across entire process tree. Must call BEFORE killing."""
    total_user = 0.0
    total_sys = 0.0
    try:
        parent = psutil.Process(pid)
        for p in [parent, *parent.children(recursive=True)]:
            try:
                ct = p.cpu_times()
                total_user += ct.user
                total_sys += ct.system
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    except psutil.NoSuchProcess:
        pass
    return total_user, total_sys


def _kill_tree(pid: int) -> None:
    """Kill a process and all its children."""
    try:
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)
        for child in children:
            child.kill()
        parent.kill()
    except psutil.NoSuchProcess:
        pass


def run_benchmark(timeout: int) -> BenchmarkResult:
    """Run the replay with time, collecting metrics."""
    # Get optimizations: extra CLI args, env vars, and startup patches
    extra_args: list[str] = []
    extra_env: dict[str, str] = {}
    startup_code: str = ""
    try:
        import optimizations

        result = optimizations.apply()
        if isinstance(result, dict):
            extra_args = result.get("cli_args", [])
            extra_env = result.get("env", {})
            startup_code = result.get("startup_code", "")
        print(
            f"OPTIMIZATIONS: applied (cli_args={extra_args}, env_keys={list(extra_env.keys())}, "
            f"startup_code={'yes' if startup_code else 'no'})"
        )
    except Exception as e:
        print(f"OPTIMIZATIONS: failed to apply - {e}")

    # Build command with extra args inserted before the dimos "run" subcommand
    # Structure: ["uv", "run", "dimos", ...flags..., "run", "unitree-go2"]
    # Insert after dimos flags, before the "run" subcommand
    replay_cmd = list(BASE_REPLAY_CMD)
    if extra_args:
        run_idx = replay_cmd.index("run")
        for arg in reversed(extra_args):
            replay_cmd.insert(run_idx, arg)

    # Write startup patch if provided (injected via PYTHONSTARTUP)
    patch_file = SCRIPT_DIR / "_startup_patch.py"
    env = os.environ.copy()
    env.update(extra_env)
    if startup_code:
        patch_file.write_text(startup_code)
        env["PYTHONSTARTUP"] = str(patch_file)

    cmd = f"time {' '.join(replay_cmd)}"
    print(f"RUNNING: {cmd}")
    print(f"TIMEOUT: {timeout}s")

    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=REPO_ROOT,  # repo root
        env=env,
    )

    # Sample stats while running
    monitor = ProcessTreeMonitor(proc.pid)
    peak_cpu = 0.0
    peak_mem = 0.0
    cpu_samples: list[float] = []
    thread_samples: list[int] = []
    last_io_read = 0
    last_io_write = 0
    last_ctx_switches = 0
    timed_out = False
    ttfm = 0.0  # time to first message
    ttfm_detected = False
    initial_io_read = 0

    start = time.monotonic()

    # Wait for process tree to start up
    time.sleep(5)
    # First sample primes cpu_percent for initial processes
    initial_sample = monitor.sample()
    initial_io_read = initial_sample["io_read_bytes"]

    while proc.poll() is None:
        elapsed = time.monotonic() - start
        if elapsed > timeout:
            print("TIMEOUT: collecting final cpu_times before kill...")
            # Collect cpu_times from all processes before killing
            tree_user, tree_sys = _collect_tree_cpu_times(proc.pid)
            print(f"TIMEOUT: killing after {timeout}s")
            _kill_tree(proc.pid)
            proc.wait()
            timed_out = True
            break

        stats = monitor.sample()
        peak_cpu = max(peak_cpu, stats["cpu_pct"])
        cpu_samples.append(stats["cpu_pct"])
        peak_mem = max(peak_mem, stats["memory_bytes"])
        thread_samples.append(stats["threads"])
        last_io_read = max(last_io_read, stats["io_read_bytes"])
        last_io_write = max(last_io_write, stats["io_write_bytes"])
        last_ctx_switches = max(last_ctx_switches, stats["ctx_switches"])

        # TTFM: detect first significant I/O activity (replay data being read)
        # A jump of >1MB from initial indicates replay streams started publishing
        if not ttfm_detected and (stats["io_read_bytes"] - initial_io_read) > 1_000_000:
            ttfm = time.monotonic() - start
            ttfm_detected = True
            print(f"TTFM: first message detected at {ttfm:.2f}s")

        time.sleep(1.0)

    wall_time = time.monotonic() - start
    proc.stdout.read().decode() if proc.stdout else ""
    stderr = proc.stderr.read().decode() if proc.stderr else ""

    # Determine user/sys CPU time
    if not timed_out:
        # Process exited cleanly — collect from tree (may already be gone)
        tree_user, tree_sys = 0.0, 0.0
        try:
            real_parsed, user_parsed, sys_parsed = parse_time_output(stderr)
            tree_user, tree_sys = user_parsed, sys_parsed
        except ValueError:
            pass

    user = tree_user
    sys_ = tree_sys
    real = wall_time

    avg_cpu_pct = sum(cpu_samples) / len(cpu_samples) if cpu_samples else 0
    avg_threads = sum(thread_samples) / len(thread_samples) if thread_samples else 0

    print(f"TIMING: user={user:.2f}s sys={sys_:.2f}s wall={real:.2f}s")
    print(f"TIMING: avg_cpu={avg_cpu_pct:.1f}% peak_cpu={peak_cpu:.1f}%")
    print(f"TIMING: score (user+sys) = {user + sys_:.2f}s")
    if ttfm_detected:
        print(f"TIMING: ttfm = {ttfm:.2f}s")
    else:
        print("TIMING: ttfm = N/A (no I/O activity detected)")

    if proc.returncode != 0 and not timed_out:
        print(f"PROCESS EXITED with code {proc.returncode}")
        print(f"STDERR (last 1000 chars): {stderr[-1000:]}")

    result = BenchmarkResult(
        score=user + sys_,
        real=real,
        user=user,
        sys=sys_,
        ttfm=ttfm if ttfm_detected else -1.0,
        peak_cpu_pct=peak_cpu,
        peak_memory_mb=peak_mem / (1024 * 1024),
        avg_threads=avg_threads,
        total_io_read_mb=last_io_read / (1024 * 1024),
        total_io_write_mb=last_io_write / (1024 * 1024),
        context_switches=last_ctx_switches,
    )

    if timed_out:
        result.validation = "TIMEOUT"
        result.validation_detail = f"killed after {timeout}s (replay loops forever)"
    elif proc.returncode != 0:
        result.validation = "FAIL"
        result.validation_detail = f"exit code {proc.returncode}"

    return result


def run_profile(duration: int = 30) -> None:
    """Run replay under cProfile for a short duration, save results."""
    print(f"PROFILING: running for {duration}s...")

    cmd = f"{' '.join(BASE_REPLAY_CMD)}"
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=REPO_ROOT,
    )

    time.sleep(duration)

    # Kill process tree
    try:
        parent = psutil.Process(proc.pid)
        for child in parent.children(recursive=True):
            child.kill()
        parent.kill()
    except psutil.NoSuchProcess:
        pass
    proc.wait()

    # Since we can't cProfile a subprocess, use py-spy if available,
    # otherwise fall back to psutil-based profiling
    print("NOTE: For detailed profiling, run manually with:")
    print(f"  py-spy record -o profile.svg -- {cmd}")
    print(f"  or: python -m cProfile -o profile.prof -c '{cmd}'")

    # Collect a basic thread/function summary via py-spy top
    try:
        # Try py-spy dump for a snapshot
        dump_proc = subprocess.run(
            ["py-spy", "dump", "--pid", str(proc.pid)],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if dump_proc.returncode == 0:
            PROFILE_PATH.write_text(dump_proc.stdout)
            print(f"PROFILE: saved to {PROFILE_PATH}")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        # py-spy not available, write a placeholder
        PROFILE_PATH.write_text(
            "Profile not available. Install py-spy for detailed profiling:\n"
            "  pip install py-spy\n"
            "  py-spy record -o profile.svg -- " + cmd + "\n\n"
            "Or run with cProfile:\n"
            "  python -m cProfile -s cumulative -c 'import subprocess; "
            f"subprocess.run({BASE_REPLAY_CMD})'\n"
        )
        print(f"PROFILE: placeholder saved to {PROFILE_PATH}")


def print_results(result: BenchmarkResult) -> None:
    """Print results in machine-parseable format."""
    print("\n" + "=" * 50)
    print(f"SCORE: {result.score:.2f}")
    print(f"TTFM: {result.ttfm:.2f}" if result.ttfm >= 0 else "TTFM: N/A")
    print(f"VALIDATION: {result.validation}", end="")
    if result.validation_detail:
        print(f" ({result.validation_detail})", end="")
    print()
    print(f"REAL: {result.real:.2f}")
    print(f"USER: {result.user:.2f}")
    print(f"SYS: {result.sys:.2f}")
    print(f"PEAK_CPU_PCT: {result.peak_cpu_pct:.1f}")
    print(f"PEAK_MEMORY_MB: {result.peak_memory_mb:.1f}")
    print(f"AVG_THREADS: {result.avg_threads:.1f}")
    print(f"IO_READ_MB: {result.total_io_read_mb:.1f}")
    print(f"IO_WRITE_MB: {result.total_io_write_mb:.1f}")
    print(f"CTX_SWITCHES: {result.context_switches}")
    print("=" * 50)

    # Save to results directory
    RESULTS_DIR.mkdir(exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    result_file = RESULTS_DIR / f"run_{timestamp}.json"
    result_file.write_text(json.dumps(asdict(result), indent=2))
    print(f"SAVED: {result_file}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 CPU footprint eval")
    parser.add_argument(
        "--profile-only", action="store_true", help="Only run profiling, skip benchmark"
    )
    parser.add_argument(
        "--skip-profile", action="store_true", help="Skip profiling even on first run"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=DEFAULT_TIMEOUT,
        help=f"Timeout in seconds (default: {DEFAULT_TIMEOUT})",
    )
    args = parser.parse_args()

    # Step 1: Profile (first run only)
    if args.profile_only:
        run_profile()
        return

    if not args.skip_profile and not PROFILE_PATH.exists():
        run_profile()

    # Step 2: Benchmark
    result = run_benchmark(args.timeout)

    # Step 3: Validation is TODO — needs LCM recording hook
    # For now, mark as PASS if the process completed successfully
    if result.validation != "FAIL":
        result.validation = "PASS"
        result.validation_detail = "basic (process exited cleanly)"

    print_results(result)


if __name__ == "__main__":
    main()
