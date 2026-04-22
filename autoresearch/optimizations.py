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

"""Optimizations for Go2 basic blueprint CPU usage.

This is the PRIMARY file the autoresearch agent edits. `eval.py` imports
`apply()` and splices its return value into the replay subprocess.

Design:
- Each knob lives as a top-level constant with a default matching DimOS
  upstream (i.e. the baseline when the knob is "off"). Flip a value or
  toggle an `ENABLE_*` flag to activate the change.
- `apply()` composes the active knobs into `{cli_args, env, startup_code}`.
- `startup_code` is written to `_patches/sitecustomize.py` and injected via
  `PYTHONPATH`, so it runs before `dimos` imports — that's how we
  monkey-patch module-level constants like `_LCM_LOOP_TIMEOUT` without
  touching the DimOS source tree.
- Knobs 7-10 control DimOS source-level optimizations that are gated behind
  env vars / global_config flags so the agent can toggle them without editing
  DimOS source each time.

Baseline: all `ENABLE_*` flags False → `apply()` returns empty dict-ish
(no CLI args, no env, no patches). This matches "Re-baseline with a clean
`optimizations.py` (empty apply() returning {})" from program.md.
"""

from __future__ import annotations

import os
from pathlib import Path

PATCH_DIR = Path(__file__).parent / "_patches"


# ------------------------------------------------------------------
# KNOB 1: LCM polling timeout
# ------------------------------------------------------------------
# Source: dimos/protocol/service/lcmservice.py:56 — `_LCM_LOOP_TIMEOUT = 50` (ms)
# Each LCM service runs a dedicated thread that blocks on `lcm.handle_timeout(T)`.
# Lower T = more context switches. With ~10 LCM services and T=50ms, that's
# ~200 wakeups/sec of pure overhead.
# Safe range: 20 - 500 ms. Above ~500ms, RPC latency becomes visible.
ENABLE_LCM_TIMEOUT = False
LCM_LOOP_TIMEOUT_MS = 50  # upstream default = 50


# ------------------------------------------------------------------
# KNOB 2: RPC call thread pool size
# ------------------------------------------------------------------
# Source: dimos/protocol/rpc/pubsubrpc.py:80 — `_call_thread_pool_max_workers = 50`
# Each module instance lazily creates a 50-worker pool for RPC call handlers.
# During replay, RPC traffic is ~zero (no skill invocations). 50 is overkill.
# Safe range: 2 - 16. Going to 1 risks deadlocking recursive RPCs.
ENABLE_RPC_POOL = False
RPC_POOL_MAX_WORKERS = 50  # upstream default


# ------------------------------------------------------------------
# KNOB 3: camera_info 1 Hz daemon thread
# ------------------------------------------------------------------
# Source: dimos/robot/unitree/go2/connection.py:355-358 — `while True: publish; sleep(1.0)`
# During replay, camera intrinsics are static and don't need to be re-published
# 1x/sec. Options:
#   "default": 1 Hz (upstream behavior)
#   "slow":    0.1 Hz (every 10s)
#   "once":    publish once on start, then exit thread
CAMERA_INFO_MODE = "once"  # "default" | "slow" | "once" | "skip"


# ------------------------------------------------------------------
# KNOB 4: n_workers (DimOS worker process count)
# ------------------------------------------------------------------
# Basic blueprint has 3-4 modules; 2 workers is usually enough. Each extra
# worker is a full Python process = ~150MB RSS + forkserver overhead.
# None = leave at blueprint default (unitree-go2-basic sets 4 via
# global_config(n_workers=4)).
N_WORKERS: int | None = None  # use blueprint default (7 for unitree-go2)


# ------------------------------------------------------------------
# KNOB 5: BLAS / OpenMP thread counts
# ------------------------------------------------------------------
# numpy / torch / lcm codec libs spawn parallel threads by default (one per
# core). For single-threaded operations these do nothing but burn CPU on
# thread-pool management. Pinning to 1 often helps on replay.
ENABLE_BLAS_PINNING = True
OMP_NUM_THREADS = 1
MKL_NUM_THREADS = 1
OPENBLAS_NUM_THREADS = 1


# ------------------------------------------------------------------
# KNOB 6: Logging verbosity
# ------------------------------------------------------------------
# Source: dimos/utils/logging_config.py:252 reads DIMOS_LOG_LEVEL env var.
# Default is INFO; every transport/module logs a few lines per second.
# WARNING or ERROR cuts stdout volume (and formatter CPU) substantially.
ENABLE_LOG_REDUCTION = True
LOG_LEVEL = "WARNING"  # "DEBUG" | "INFO" | "WARNING" | "ERROR" | "CRITICAL"


# ------------------------------------------------------------------
# KNOB 7: Skip WebsocketVisModule during replay
# ------------------------------------------------------------------
# The WebsocketVisModule spawns a Uvicorn server, SocketIO broadcast loop,
# and its own asyncio event loop — all serving zero clients during benchmark.
# When enabled, the blueprint skips this module entirely.
# Controlled via env var DIMOS_SKIP_WEBSOCKET_VIS=1 → checked in blueprint.
ENABLE_SKIP_WEBSOCKET_VIS = True


# ------------------------------------------------------------------
# KNOB 8: Skip ClockSyncConfigurator during replay
# ------------------------------------------------------------------
# ClockSyncConfigurator runs NTP-like sync at coordinator build time.
# During replay there's no real robot clock to sync with — pure waste.
# Controlled via env var DIMOS_SKIP_CLOCK_SYNC=1 → checked in blueprint.
ENABLE_SKIP_CLOCK_SYNC = True


# ------------------------------------------------------------------
# KNOB 9: Lazy asyncio event loop per module
# ------------------------------------------------------------------
# By default, every Module.__init__() eagerly creates a dedicated asyncio
# event loop + daemon thread. Most modules never use asyncio during replay.
# When enabled, loops are created lazily on first access.
# Controlled via env var DIMOS_LAZY_ASYNCIO=1 → checked in module.py.
ENABLE_LAZY_ASYNCIO = True


# ------------------------------------------------------------------
# KNOB 10: Replay I/O prefetch buffer
# ------------------------------------------------------------------
# LegacyPickleStore reads one pickle file at a time, blocking on disk I/O
# each frame. When enabled, a background thread prefetches the next N items.
# Controlled via env var DIMOS_REPLAY_PREFETCH=<N> (0=off, default).
ENABLE_REPLAY_PREFETCH = False
REPLAY_PREFETCH_SIZE = 5  # number of items to prefetch ahead


# ------------------------------------------------------------------
# KNOB 11: Skip TF publishing during replay
# ------------------------------------------------------------------
# Every odom message triggers 3 TF transforms published via LCM.
# During --viewer=none replay, nobody subscribes to TF. Pure waste.
# Controlled via env var DIMOS_SKIP_TF=1 → checked in GO2Connection._publish_tf.
ENABLE_SKIP_TF = False  # navigation planners need TF


# ------------------------------------------------------------------
# KNOB 12: Skip sensor stream LCM publishing
# ------------------------------------------------------------------
# GO2Connection publishes lidar/color_image via LCM. During --viewer=none
# bench, nobody subscribes. Encoding + broadcast is pure overhead.
# Keeps stream subscription (so exit-on-eof works) but no-ops the publish.
# Controlled via env var DIMOS_SKIP_SENSOR_PUBLISH=1.
ENABLE_SKIP_SENSOR_PUBLISH = False  # navigation needs lidar/odom


# ------------------------------------------------------------------
# KNOB 13: Skip robot motion init/shutdown during replay
# ------------------------------------------------------------------
# GO2Connection.start() calls standup/balance_stand + sleep(3).
# During replay these are no-ops but sleep(3) delays TTFM.
# Controlled via env var DIMOS_SKIP_ROBOT_INIT=1.
ENABLE_SKIP_ROBOT_INIT = True
ENABLE_SKIP_CMDVEL_SUB = False  # navigation/patrol needs cmd_vel
ENABLE_DISABLE_GC = True
ENABLE_REPLAY_ONLY = True


# ------------------------------------------------------------------
# KNOB 14: Replay speed multiplier
# ------------------------------------------------------------------
# Replay is time-paced (emits at recorded timestamps). Higher speed = shorter
# wall time = less idle polling overhead. Fixed work still processed fully.
# Controlled via env var DIMOS_REPLAY_SPEED=<float>.
ENABLE_REPLAY_SPEED = False  # navigation planners need real-time pacing
REPLAY_SPEED = 1.0


# ------------------------------------------------------------------
# KNOB 15: Skip video autocast
# ------------------------------------------------------------------
# Video stream runs _autocast_video per frame (VideoFrame→ndarray→Image).
# Expensive per-frame numpy conversion. Nobody consumes it during bench.
ENABLE_SKIP_VIDEO_AUTOCAST = False


def _build_startup_code() -> str:
    """Assemble the monkey-patch string injected via sitecustomize.py."""
    lines: list[str] = []

    if ENABLE_LCM_TIMEOUT:
        lines.append(
            "import dimos.protocol.service.lcmservice as _lcm_mod\n"
            f"_lcm_mod._LCM_LOOP_TIMEOUT = {LCM_LOOP_TIMEOUT_MS}\n"
        )

    if ENABLE_RPC_POOL:
        lines.append(
            "import dimos.protocol.rpc.pubsubrpc as _rpc_mod\n"
            f"_rpc_mod.PubSubRPCBase._call_thread_pool_max_workers = {RPC_POOL_MAX_WORKERS}\n"
        )

    return "".join(lines)


def apply() -> dict:
    """Return optimization config for the eval harness."""

    cli_args: list[str] = []
    env: dict[str, str] = {}

    if N_WORKERS is not None:
        cli_args.append(f"--n-workers={N_WORKERS}")

    if ENABLE_LOG_REDUCTION:
        env["DIMOS_LOG_LEVEL"] = LOG_LEVEL

    if ENABLE_BLAS_PINNING:
        env["OMP_NUM_THREADS"] = str(OMP_NUM_THREADS)
        env["MKL_NUM_THREADS"] = str(MKL_NUM_THREADS)
        env["OPENBLAS_NUM_THREADS"] = str(OPENBLAS_NUM_THREADS)

    if ENABLE_SKIP_WEBSOCKET_VIS:
        env["DIMOS_SKIP_WEBSOCKET_VIS"] = "1"

    if ENABLE_SKIP_CLOCK_SYNC:
        env["DIMOS_SKIP_CLOCK_SYNC"] = "1"

    if ENABLE_LAZY_ASYNCIO:
        env["DIMOS_LAZY_ASYNCIO"] = "1"

    if ENABLE_REPLAY_PREFETCH:
        env["DIMOS_REPLAY_PREFETCH"] = str(REPLAY_PREFETCH_SIZE)

    if CAMERA_INFO_MODE != "default":
        env["DIMOS_CAMERA_INFO_MODE"] = CAMERA_INFO_MODE

    if ENABLE_SKIP_TF:
        env["DIMOS_SKIP_TF"] = "1"

    if ENABLE_SKIP_SENSOR_PUBLISH:
        env["DIMOS_SKIP_SENSOR_PUBLISH"] = "1"

    if ENABLE_SKIP_ROBOT_INIT:
        env["DIMOS_SKIP_ROBOT_INIT"] = "1"

    if ENABLE_SKIP_CMDVEL_SUB:
        env["DIMOS_SKIP_CMDVEL_SUB"] = "1"

    if ENABLE_DISABLE_GC:
        env["DIMOS_DISABLE_GC"] = "1"

    if ENABLE_REPLAY_ONLY:
        env["DIMOS_REPLAY_ONLY"] = "1"

    env["MALLOC_ARENA_MAX"] = "1"

    if ENABLE_REPLAY_SPEED:
        env["DIMOS_REPLAY_SPEED"] = str(REPLAY_SPEED)

    if ENABLE_SKIP_VIDEO_AUTOCAST:
        env["DIMOS_SKIP_VIDEO_AUTOCAST"] = "1"

    startup_code = _build_startup_code()
    if startup_code:
        PATCH_DIR.mkdir(exist_ok=True)
        (PATCH_DIR / "sitecustomize.py").write_text(startup_code)
        existing = os.environ.get("PYTHONPATH", "")
        env["PYTHONPATH"] = f"{PATCH_DIR}:{existing}" if existing else str(PATCH_DIR)

    return {
        "cli_args": cli_args,
        "env": env,
        "startup_code": "",  # handled via sitecustomize
    }
