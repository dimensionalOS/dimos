#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Forward M20 zenoh topics onto the local LCM bus.

The M20's perception (``slam_aligned_points``, ``slam_odom``, ...) only exists on
zenoh (robot -> aos router). LCM-based native modules -- e.g. ``RayTracingVoxelMap``
in the ``m20-nav`` blueprint -- subscribe on LCM. dimos serializes both transports
identically (``lcm_encode``), so we forward the raw payload bytes untouched; only
the channel string differs:

    zenoh  "dimos/slam_aligned_points/sensor_msgs.PointCloud2"
    LCM    "dimos/slam_aligned_points#sensor_msgs.PointCloud2"

i.e. replace the last ``/`` (before the ``module.Class`` type segment) with ``#``;
the ``dimos/`` prefix is kept. Verified against the running native module's CLI:
``--lidar dimos/slam_aligned_points#sensor_msgs.PointCloud2``.

Usage:
    python -m dimos.robot.deeprobotics.m20.zenoh_lcm_bridge [zenoh_key ...]
    DIMOS_ZENOH_CONNECT=tcp/192.168.0.119:7447 python -m ...zenoh_lcm_bridge

Default keys are the ray tracer's inputs.
"""

import os
import sys
import time

import lcm
import zenoh

DEFAULT_ROUTER = "tcp/192.168.0.119:7447"
# Subscribe to literally everything on zenoh and forward each to its LCM
# channel. Pass explicit keys as argv to narrow the set.
DEFAULT_KEYS = ["**"]


def zkey_to_lcm(key: str) -> str:
    """zenoh key-expr -> LCM channel ("dimos/topic#module.Class").

    Just replaces the last "/" (the type separator) with "#"; the "dimos/"
    prefix is kept, matching what the native module subscribes to.
    """
    topic, sep, typ = key.rpartition("/")
    return f"/{topic}#{typ}" if sep else key


def main() -> None:
    router = os.environ.get("DIMOS_ZENOH_CONNECT", DEFAULT_ROUTER)
    keys = sys.argv[1:] or DEFAULT_KEYS

    lc = lcm.LCM()  # standard default URL -- matches the native modules

    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{router}"]')
    conf.insert_json5("scouting/multicast/enabled", "false")
    session = zenoh.open(conf)

    counts: dict[str, int] = {}

    def cb(sample: "zenoh.Sample") -> None:
        chan = zkey_to_lcm(str(sample.key_expr))
        lc.publish(chan, bytes(sample.payload))
        counts[chan] = counts.get(chan, 0) + 1

    for k in keys:
        session.declare_subscriber(k, cb)

    print(f"bridging zenoh {router} -> LCM (standard url)", flush=True)
    print(f"keys: {keys}", flush=True)
    try:
        while True:
            time.sleep(5)
            print("forwarded:", {k: counts[k] for k in sorted(counts)}, flush=True)
    except KeyboardInterrupt:
        session.close()


if __name__ == "__main__":
    main()
