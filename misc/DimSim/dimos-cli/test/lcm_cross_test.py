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

"""Quick test: can Python receive LCM messages from Deno?"""

import sys
import time

sys.path.insert(0, "/Users/viswajitnair/Desktop/4Wall.nosync/Dimensional/dimos")
import lcm

received = {"count": 0}


def handler(channel, data):
    received["count"] += 1
    print(f"[py] Got message #{received['count']} on {channel} ({len(data)} bytes)")


lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
lc.subscribe(".*lcm_cross_test.*", handler)

print("[py] Listening for LCM messages on /lcm_cross_test...")
print("[py] Waiting 15s for Deno publisher...\n")

deadline = time.time() + 15
while time.time() < deadline:
    lc.handle_timeout(500)
    if received["count"] >= 3:
        print(f"\n[py] SUCCESS: received {received['count']} messages from Deno!")
        sys.exit(0)

print(f"\n[py] FAIL: only received {received['count']} messages")
sys.exit(1)
