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

"""The Franka Hand as a program of its own, for FrankaConnection.

pylibfranka's Gripper calls hold Python's GIL while the fingers move, so the
connection runs the Hand here, in a separate interpreter: widths come in on
stdin and readings go out on stdout, one JSON value per line. Closing stdin
stops it. A subprocess rather than multiprocessing, because DimOS modules run
in daemon processes, which may not have multiprocessing children.

    python -m dimos.robot.manipulators.franka.hand_process <ip> <grasp_force>
"""

from __future__ import annotations

import json
import queue
import sys
import threading
from typing import Any


class _Lines:
    """``put`` writes one JSON line to stdout."""

    def __init__(self) -> None:
        self._lock = threading.Lock()

    def put(self, item: Any) -> None:
        with self._lock:
            sys.stdout.write(json.dumps(item) + "\n")
            sys.stdout.flush()


def main() -> None:
    from dimos.robot.manipulators.franka.connection import _hand_loop, _import_libfranka

    ip, grasp_force = sys.argv[1], float(sys.argv[2])
    goals: queue.Queue[float] = queue.Queue()
    stop = threading.Event()

    def read_goals() -> None:
        for line in sys.stdin:
            goals.put(float(json.loads(line)))
        stop.set()  # the connection closed our stdin

    threading.Thread(target=read_goals, daemon=True).start()
    libfranka = _import_libfranka()
    _hand_loop(lambda: libfranka.Gripper(ip), grasp_force, goals, _Lines(), stop)


if __name__ == "__main__":
    main()
