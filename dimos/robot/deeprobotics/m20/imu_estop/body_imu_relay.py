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

"""The dimos ``imu`` stream (sensor_msgs Imu, base_link axes) as one text line per sample on stdout, for imu_estop.

imu_estop is stdlib-only Python 3.8, so it runs this with a dimos venv's Python (zenoh + the LCM types) as a child and
reads its stdout. Every sample of any run's ``imu`` on the board's zenoh router is relayed, whoever publishes it: the
body IMU sniffer (vendor mode) or the Yesense serial module (rust mode). Line: ``ax ay az gx gy gz`` (m/s^2, rad/s).

    python body_imu_relay.py [router]      # default tcp/127.0.0.1:7447, the board's dimos Host
"""

from __future__ import annotations

import os
import sys
import threading

# a hosted run's streams (dimos-domains/<run>/imu/...) and a run on the board's own zenoh (dimos/imu/...)
KEYS = ("dimos-domains/*/imu/sensor_msgs.Imu", "dimos/imu/sensor_msgs.Imu")


def line(payload: bytes) -> str:
    from dimos_lcm.sensor_msgs import Imu

    msg = Imu.lcm_decode(payload)
    a, g = msg.linear_acceleration, msg.angular_velocity
    return f"{a.x:.6f} {a.y:.6f} {a.z:.6f} {g.x:.6f} {g.y:.6f} {g.z:.6f}\n"


def main(router: str = "tcp/127.0.0.1:7447") -> None:
    import zenoh

    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{router}"]')
    config.insert_json5("scouting/multicast/enabled", "false")
    lock = threading.Lock()
    reader_gone = threading.Event()

    def on_sample(sample: zenoh.Sample) -> None:
        text = line(sample.payload.to_bytes())
        with lock:
            try:
                sys.stdout.write(text)
                sys.stdout.flush()
            except (BrokenPipeError, ValueError):
                # imu_estop closed the pipe: an exception here would stay in zenoh's thread, so wake main to exit
                reader_gone.set()

    session = zenoh.open(config)
    subscribers = [session.declare_subscriber(key, on_sample) for key in KEYS]
    # runs until imu_estop closes the pipe (the next write fails) or kills it
    reader_gone.wait()
    del subscribers
    session.close()
    # stdout is already broken: skip the interpreter's final flush of it
    os._exit(0)


if __name__ == "__main__":
    main(*sys.argv[1:2])
