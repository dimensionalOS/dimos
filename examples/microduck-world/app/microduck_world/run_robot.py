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

"""Run one robot blueprint against the persistent world."""

import argparse
import faulthandler
import signal
import threading

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from microduck_world.robot_blueprint import robot_blueprint
from microduck_world.robot_io import ROBOT_IDS


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=ROBOT_IDS, required=True)
    parser.add_argument("--generation", required=True)
    args = parser.parse_args()
    stopped = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    # A missing RPC discovery reply can stall a startup. Save the blocked stack
    # and exit so the supervisor can replace this process and its worker group.
    # Hosted assets are already installed; a normal build takes a few seconds.
    faulthandler.dump_traceback_later(60, exit=True)
    try:
        coordinator = ModuleCoordinator.build(robot_blueprint(args.robot, args.generation))
    finally:
        faulthandler.cancel_dump_traceback_later()
    try:
        # The world owns the operator coordinator endpoint. Robot modules keep
        # their own namespaced RPC endpoints and private MCP server.
        stopped.wait()
    finally:
        coordinator.stop()


if __name__ == "__main__":
    main()
