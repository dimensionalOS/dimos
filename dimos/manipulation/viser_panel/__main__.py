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

from __future__ import annotations

import argparse
import signal
import sys
import time

from dimos.manipulation.viser_panel.module import ViserManipulationPanelModule


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8095)
    parser.add_argument("--robot", default=None)
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument("--allow-execute", action="store_true")
    args = parser.parse_args()

    module = ViserManipulationPanelModule(
        host=args.host,
        port=args.port,
        default_robot=args.robot,
        open_browser=args.open_browser,
        allow_plan_execute=args.allow_execute,
    )
    stopped = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True
        module.stop()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        module.start()
    except ModuleNotFoundError as e:
        print(str(e), file=sys.stderr)
        raise SystemExit(1) from None
    while not stopped:
        time.sleep(0.2)


if __name__ == "__main__":
    main()
