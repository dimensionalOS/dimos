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

"""Every tuning number of the gateway: timeouts, intervals, cache lifetimes, size and length limits."""

import sys

# scan: one blueprint's imports in the scan child, past this it reads as hung (the default of the scan routes' ?timeout=)
SCAN_ITEM_TIMEOUT_S = 120.0
# scan: longest line the scan child may print, since one record carries a whole blueprint's modules
SCAN_STDOUT_LIMIT_BYTES = 64 * 1024 * 1024
# scan: stderr lines kept to explain a child that died, enough for a traceback's end
SCAN_STDERR_TAIL_LINES = 20
# scan: a checkout's git-status fingerprint is reused this long, so a burst of requests shares one git status
FINGERPRINT_CACHE_S = 2.0

# jobs: a failed job's last non-blank output lines shown as its failure, enough for the error without the whole log
JOB_FAILURE_TAIL_LINES = 15

# introspection: a cold import of a heavy blueprint's dependencies (torch, open3d) can take minutes
INTROSPECT_TIMEOUT_S = 180.0
# introspection: an answer is reused ten minutes; a blueprint added or removed clears them all
INTROSPECT_CACHE_S = 600.0

# watchers: a run that crashed leaves its registry file behind, so no file event says it ended; this finds it
STALE_RUN_CHECK_S = 5.0
# watchers: file changes are grouped into one batch for at most this long (awatch's default)
FILE_EVENT_DEBOUNCE_MS = 1_600

# launches: a relaunch waits this long for a killed, still-starting launch to exit before starting its replacement
KILLED_LAUNCH_EXIT_WAIT_S = 5.0
# launches: how often that wait checks whether the killed launch is gone
KILLED_LAUNCH_POLL_S = 0.05
# launches: the last ~200 KB of a launch's output, enough for its startup error without reading a huge log
LAUNCH_OUTPUT_TAIL_BYTES = 200_000

# logs: at most the last 4 MiB of a run's log per read, so a huge log answers quickly
LOG_MAX_READ_BYTES = 4 * 1024 * 1024
# logs: bytes read at a log's start or end to find its first or last timestamp, a few records' worth
LOG_TIMESTAMP_PROBE_BYTES = 8192
# logs: records returned when a log request gives no ?limit=
DEFAULT_LOG_LIMIT = 1000

# process stats: the API's "a minute of cpu history" per process
CPU_HISTORY_S = 60.0
# process stats: stats read as stale once this many samples are missed
STALE_AFTER_SAMPLES = 3

# server: POST /server/stop exits this long after answering, so its response gets out first
SERVER_STOP_DELAY_S = 0.2

# server: the default TCP host is loopback because the gateway has no auth
DEFAULT_HOST = "127.0.0.1"
# server: longest unix socket path the kernel takes (sun_path is 104 bytes on macOS/BSD, 108 on Linux)
UNIX_SOCKET_PATH_LIMIT = 104 if sys.platform == "darwin" else 108

# cloud uploads: the shown upload speed is a moving average over about this long, so it doesn't jump per chunk
UPLOAD_RATE_SMOOTHING_S = 6.0
# cloud uploads: no ETA until this long after an upload starts, while the measured rate is still noise
UPLOAD_ETA_WARMUP_S = 1.0
# cloud uploads: progress events go out at most this often within a phase, so a fast upload doesn't flood the bus
UPLOAD_PROGRESS_EMIT_S = 0.4
# cloud account: account checks are cached this long; ?fresh= forces one
ACCOUNT_CACHE_S = 20.0
