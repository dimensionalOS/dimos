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

import argparse
import time

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Twist import Twist


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a Twist command at a fixed rate.")
    parser.add_argument("--topic", default="/nav_cmd_vel", help="Topic to publish to.")
    parser.add_argument("--linear-x", type=float, default=0.0)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.8)
    parser.add_argument("--rate", type=float, default=50.0, help="Publish rate in Hz.")
    args = parser.parse_args()

    pub = make_transport(args.topic, Twist)
    msg = Twist(
        linear=(args.linear_x, args.linear_y, 0.0),
        angular=(0.0, 0.0, args.angular_z),
    )
    dt = 1.0 / args.rate

    print(f"Publishing {msg!r} to {args.topic} at {args.rate} Hz. Press Ctrl-C to stop.")
    try:
        while True:
            pub.broadcast(None, msg)
            time.sleep(dt)
    except KeyboardInterrupt:
        pub.broadcast(None, Twist())
        print("stopped")


if __name__ == "__main__":
    main()
