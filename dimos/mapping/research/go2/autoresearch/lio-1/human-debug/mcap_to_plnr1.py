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

"""MCAP (go2-station) -> PLNR1 binary for point_lio_noros.

Unlike the old extract_to_bin.py (SQLite + SYNTHESIZED ring/time), our MCAP
records the raw rt/utlidar/cloud whose data blob IS the 32-byte
unilidar_ros::Point layout (x@0,y@4,z@8,intensity@16,ring@20,time@24) with the
REAL per-point ring + sweep time — so we write it verbatim. Lidar frame ts +
imu ts = the onboard device stamp (MCAP publish_time). Records are time-sorted.

PLNR1 (see extract_to_bin.py):
  header 16B: b"PLNR1\\0"*pad
  lidar:  <B7xd  (type=0, ts_sec)  +  <II (n_pts, 0)  +  n_pts*32B points
  imu:    <B7xd6d (type=1, ts_sec, gx,gy,gz, ax,ay,az)

Usage: uv run --with mcap --with numpy python mcap_to_plnr1.py <in.mcap> <out.bin>
"""

import struct
import sys

sys.path.insert(0, "/Users/p2o5/Desktop/2026.5/dim/go2-station/scripts")
import go2_cdr as cdr
from mcap.reader import make_reader

MAGIC = b"PLNR1\0\0\0\0\0\0\0\0\0\0\0"  # 16 B
inp, out = sys.argv[1], sys.argv[2]

recs = []  # (ts_ns, payload_bytes)
nl = ni = 0
with open(inp, "rb") as f:
    for _sch, ch, msg in make_reader(f).iter_messages(
        topics=["rt/utlidar/cloud", "rt/utlidar/imu"]
    ):
        ts = msg.publish_time  # onboard stamp (ns)
        if ch.topic == "rt/utlidar/imu":
            m = cdr.decode_imu(msg.data)
            g, a = m["ang_vel"], m["lin_acc"]
            payload = struct.pack("<B7xd6d", 1, ts / 1e9, g[0], g[1], g[2], a[0], a[1], a[2])
            recs.append((ts, payload))
            ni += 1
        else:
            pc = cdr.decode_pointcloud2(msg.data)
            arr = pc["arr"]
            if len(arr) == 0:
                continue
            blob = arr.tobytes()  # n*32 bytes, exact L1 point layout (real ring+time)
            payload = struct.pack("<B7xd", 0, ts / 1e9) + struct.pack("<II", len(arr), 0) + blob
            recs.append((ts, payload))
            nl += 1

recs.sort(key=lambda r: r[0])
with open(out, "wb") as fp:
    fp.write(MAGIC)
    for _, p in recs:
        fp.write(p)

span = (recs[-1][0] - recs[0][0]) / 1e9 if recs else 0
print(f"wrote {nl} lidar + {ni} imu, span {span:.1f}s -> {out}")
