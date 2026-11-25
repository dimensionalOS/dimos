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

import json

from dimos.diagnostics.ring import TraceRing


def test_full_rate_ninety_second_history_fits_and_preserves_every_event():
    ring = TraceRing(8 * 1024 * 1024)
    for n in range(135_000):
        row = (
            json.dumps(
                {
                    "event": "gateway.command_received",
                    "mono_ns": n * 666667,
                    "context": {
                        "run_id": "qualification",
                        "publisher": "operator",
                        "stream": "pose",
                        "sequence": n,
                        "clock_domain": "operator/clock",
                        "parents": [],
                        "lineage": "complete",
                    },
                }
            ).encode()
            + b"\n"
        )
        ring.append(100 + n / 1500, row)
        ring.trim(100)
    assert ring.size <= ring.limit
    assert ring.capacity_lost_until == 0
    rows = (json.loads(row) for chunk in ring.rows() for row in chunk.splitlines())
    assert [row["context"]["sequence"] for row in rows] == list(range(135_000))


def test_capacity_loss_is_distinct_from_normal_history_expiry():
    ring = TraceRing(300)
    ring.append(100, b'{"event":"first"}\n')
    ring.trim(100)
    ring.append(101, b'{"event":"second"}\n')
    ring.trim(100)
    assert ring.capacity_lost_until >= 100
    roomy = TraceRing(65536)
    roomy.append(100, b'{"event":"first"}\n')
    roomy.append(101, b'{"event":"second"}\n')
    roomy.trim(100.5)
    assert roomy.capacity_lost_until == 0
    assert b"first" not in b"".join(roomy.rows())
