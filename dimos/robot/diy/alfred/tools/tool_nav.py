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

"""Stop navigating.

    python -m dimos.robot.diy.alfred.tools.tool_nav stop

Wired to the Controller's action bar as "Nav: stop". Goals come from the Controller's "Go here"
(a right-click in the 3D view publishes ``clicked_point``); this publishes the cancel the nav
stack already knows, MovementManager's NaN click: the MLS planner clears its path and sends an
empty one, and the follower goes idle. Teleop's Space only outranks the follower while held.
"""

from __future__ import annotations

import sys
import time

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped

PLANNING_FRAME = "odom"
# A plain publish: a few copies a short way apart make sure one lands after zenoh has
# matched the planner's subscription.
REPEATS = 3
REPEAT_S = 0.2


def cancel_navigation() -> None:
    click = make_transport("/clicked_point", PointStamped)
    nan = float("nan")
    for _ in range(REPEATS):
        click.publish(PointStamped(nan, nan, nan, frame_id=PLANNING_FRAME))
        time.sleep(REPEAT_S)


def main(argv: list[str]) -> None:
    if (argv[1] if len(argv) > 1 else "") != "stop":
        sys.exit(__doc__)
    cancel_navigation()
    print("navigation cancelled: the planner clears its path and the follower idles")


if __name__ == "__main__":
    main(sys.argv)
