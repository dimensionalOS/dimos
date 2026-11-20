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

"""Count samples per key under a prefix, no decoding in the callback (so the counter itself cannot be the bottleneck).

usage: count_topics.py <prefix> <seconds> <router>
"""

import collections
import sys
import threading
import time

import zenoh

prefix, seconds, router = sys.argv[1], float(sys.argv[2]), sys.argv[3]
counts: collections.Counter = collections.Counter()
lock = threading.Lock()


def on_sample(sample) -> None:
    key = str(sample.key_expr)
    with lock:
        counts[key] += 1


config = zenoh.Config()
config.insert_json5("mode", '"client"')
config.insert_json5("connect/endpoints", f'["{router}"]')
config.insert_json5("scouting/multicast/enabled", "false")
session = zenoh.open(config)
subscriber = session.declare_subscriber(f"{prefix}/**", on_sample)
time.sleep(seconds)
subscriber.undeclare()
with lock:
    for key, n in sorted(counts.items()):
        print(f"{n / seconds:7.1f} Hz  {n:6d}  {key}")
session.close()
