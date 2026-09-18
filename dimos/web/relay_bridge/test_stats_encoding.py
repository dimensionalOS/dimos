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

"""stats.json.v1: the resource monitor's /resource_stats dict -> the Stats page's JSON."""

from dataclasses import asdict
import json

import pytest

from dimos.core.resource_monitor.stats import ChildProcessStats, ProcessStats, WorkerStats
from dimos.web.codecs import resolve_encoder
from dimos.web.relay_bridge.builtin_codecs import encode_stats

# The input is what LCMResourceLogger.log_stats publishes: asdict() of the
# producer's own dataclasses. A renamed field changes this dict and fails the
# golden below, so producer drift is loud here rather than silent in the page.
COORDINATOR = ProcessStats(
    pid=1234,
    alive=True,
    cpu_percent=12.3,
    cpu_time_user=1.2,
    cpu_time_system=0.3,
    pss=47_400_000,
    num_threads=4,
    num_fds=32,
    io_read_bytes=12_582_912,
    io_write_bytes=4_194_304,
)
WORKERS = [
    WorkerStats(
        pid=1235,
        alive=True,
        cpu_percent=34.0,
        cpu_time_user=5.1,
        cpu_time_system=1.0,
        cpu_time_iowait=0.2,
        pss=125_829_120,
        num_threads=8,
        num_children=2,
        num_fds=64,
        io_read_bytes=47_185_920,
        io_write_bytes=12_582_912,
        worker_id=0,
        modules=["Navigation", "Lidar"],
        children=[ChildProcessStats(pid=1300, name="ffmpeg", cpu_percent=5.5)],
    ),
    # A dead worker: pid 0 and every metric at its default, identity kept.
    WorkerStats(pid=0, alive=False, worker_id=1, modules=["Vision"], dedicated=True),
]
MESSAGE = {"coordinator": asdict(COORDINATOR), "workers": [asdict(w) for w in WORKERS]}

# Every key the Stats page (web/cockpit/src/panels/stats.ts) reads, in the
# producer's declaration order.
GOLDEN = {
    "coordinator": {
        "pid": 1234,
        "alive": True,
        "cpu_percent": 12.3,
        "cpu_time_user": 1.2,
        "cpu_time_system": 0.3,
        "cpu_time_iowait": 0.0,
        "pss": 47400000,
        "num_threads": 4,
        "num_children": 0,
        "num_fds": 32,
        "io_read_bytes": 12582912,
        "io_write_bytes": 4194304,
    },
    "workers": [
        {
            "pid": 1235,
            "alive": True,
            "cpu_percent": 34.0,
            "cpu_time_user": 5.1,
            "cpu_time_system": 1.0,
            "cpu_time_iowait": 0.2,
            "pss": 125829120,
            "num_threads": 8,
            "num_children": 2,
            "num_fds": 64,
            "io_read_bytes": 47185920,
            "io_write_bytes": 12582912,
            "worker_id": 0,
            "modules": ["Navigation", "Lidar"],
            "dedicated": False,
            "children": [{"pid": 1300, "name": "ffmpeg", "cpu_percent": 5.5}],
        },
        {
            "pid": 0,
            "alive": False,
            "cpu_percent": 0.0,
            "cpu_time_user": 0.0,
            "cpu_time_system": 0.0,
            "cpu_time_iowait": 0.0,
            "pss": 0,
            "num_threads": 0,
            "num_children": 0,
            "num_fds": 0,
            "io_read_bytes": 0,
            "io_write_bytes": 0,
            "worker_id": 1,
            "modules": ["Vision"],
            "dedicated": True,
            "children": [],
        },
    ],
}


def test_golden_frame() -> None:
    assert encode_stats(MESSAGE) == json.dumps(GOLDEN, separators=(",", ":")).encode()
    assert resolve_encoder("stats.json.v1", dict).encode is encode_stats


def test_only_the_pinned_keys_cross() -> None:
    coordinator = {**MESSAGE["coordinator"], "rss": 1, "cmdline": "dimos"}
    payload = encode_stats({**MESSAGE, "coordinator": coordinator})
    assert json.loads(payload)["coordinator"] == GOLDEN["coordinator"]


def test_missing_key_is_loud() -> None:
    coordinator = dict(MESSAGE["coordinator"])
    del coordinator["pss"]
    with pytest.raises(KeyError, match="pss"):
        encode_stats({**MESSAGE, "coordinator": coordinator})
