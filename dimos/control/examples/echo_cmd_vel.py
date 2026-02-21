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

"""Echo Twist messages on /cmd_vel.

Usage:
    python -m dimos.control.examples.echo_cmd_vel
    python -m dimos.control.examples.echo_cmd_vel --topic /my_cmd_vel
"""

from __future__ import annotations

import argparse
import time

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Twist


def main() -> None:
    parser = argparse.ArgumentParser(description="Echo Twist on an LCM topic")
    parser.add_argument("--topic", default="/cmd_vel", help="LCM topic (default: /cmd_vel)")
    args = parser.parse_args()

    transport = LCMTransport(args.topic, Twist)
    print(f"Listening on {args.topic} ...")

    def on_twist(twist: Twist) -> None:
        print(
            f"  linear=({twist.linear.x:+.3f}, {twist.linear.y:+.3f}, {twist.linear.z:+.3f})"
            f"  angular=({twist.angular.x:+.3f}, {twist.angular.y:+.3f}, {twist.angular.z:+.3f})"
        )

    transport.subscribe(on_twist)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
