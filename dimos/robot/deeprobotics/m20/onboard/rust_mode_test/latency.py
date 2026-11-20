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

"""Rate and latency (receive time - message stamp) per stream and frame, decoding after capture so the callback stays cheap.

usage: latency.py <router> <seconds> <key>=<type> ...   types: cloud, imu, odom
"""

import collections
import sys
import time

from dimos_lcm.sensor_msgs.PointCloud2 import PointCloud2 as RawCloud
import numpy as np
import zenoh

from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def last_offset_s(payload: bytes) -> float:
    """The cloud's largest offset_time (s after its stamp), 0 without the field."""
    raw = RawCloud.lcm_decode(payload)
    field = next((f for f in raw.fields if f.name == "offset_time"), None)
    if field is None or raw.width * raw.height == 0:
        return 0.0
    data = np.frombuffer(bytes(raw.data), dtype=np.uint8).reshape(-1, raw.point_step)
    offsets = data[:, field.offset : field.offset + 4].copy().view("<u4")
    return float(offsets.max()) * 1e-9


router, seconds = sys.argv[1], float(sys.argv[2])
kinds = {
    "cloud": PointCloud2,
    "imu": Imu,
    "odom": Odometry,
    "joint": JointState,
    "battery": BatteryState,
    "video": CompressedVideo,
}
streams = dict(arg.split("=") for arg in sys.argv[3:])
got: dict[str, list] = collections.defaultdict(list)
batteries: dict[str, list] = collections.defaultdict(list)
cfg = zenoh.Config()
cfg.insert_json5("mode", '"client"')
cfg.insert_json5("connect/endpoints", f'["{router}"]')
cfg.insert_json5("scouting/multicast/enabled", "false")
s = zenoh.open(cfg)
subs = [
    s.declare_subscriber(
        key, (lambda k: lambda x: got[k].append((time.time(), x.payload.to_bytes())))(key)
    )
    for key in streams
]
time.sleep(seconds)
for sub in subs:
    sub.undeclare()
for key, kind in streams.items():
    per_frame = collections.defaultdict(list)
    for t, payload in got[key]:
        msg = kinds[kind].lcm_decode(payload)
        # a cloud's last point: its stamp is the scan's first point, offset_time (ns) after it per point
        end = msg.ts
        if kind == "cloud":
            end += last_offset_s(payload)
        extra = len(msg) if kind == "cloud" else 0
        if kind == "battery":
            # current (A, + = charging) and charge (%), shown in the pts column
            extra = (getattr(msg, "current", 0.0) or 0.0, getattr(msg, "percentage", 0.0) or 0.0)
        per_frame[getattr(msg, "frame_id", "")].append(
            (t, msg.ts, 0 if kind == "battery" else extra, end)
        )
        if kind == "battery":
            batteries[key].append(extra)
    for frame, rows in per_frame.items():
        a = np.array(rows)
        lag = (a[:, 0] - a[:, 1]) * 1e3
        end_lag = (a[:, 0] - a[:, 3]) * 1e3
        print(
            f"{key} [{frame}]: {len(a) / seconds:.1f} Hz, stamp-to-arrival median {np.median(lag):.0f} ms p95 {np.percentile(lag, 95):.0f} ms"
            + (
                f", last-point-to-arrival median {np.median(end_lag):.0f} ms p95 {np.percentile(end_lag, 95):.0f} ms"
                if kind == "cloud"
                else ""
            )
            + (f", {a[:, 2].mean():.0f} pts" if kind == "cloud" else "")
        )
for key, rows in batteries.items():
    a = np.array(rows)
    print(
        f"{key}: current median {np.median(a[:, 0]):+.2f} A (min {a[:, 0].min():+.2f}, max {a[:, 0].max():+.2f}), charge {np.median(a[:, 1]):.2f}"
    )
s.close()
