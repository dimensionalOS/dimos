#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""
use lcm_foxglove_bridge as a module from dimos_lcm
"""

import asyncio
import os
import threading

import dimos_lcm  # type: ignore[import-untyped]
from dimos_lcm.foxglove_bridge import FoxgloveBridge  # type: ignore[import-untyped]

dimos_lcm_path = os.path.dirname(os.path.abspath(dimos_lcm.__file__))
print(f"Using dimos_lcm from: {dimos_lcm_path}")


def run_bridge_example() -> None:
    """Example of running the bridge in a separate thread"""

    def bridge_thread() -> None:
        """Thread function to run the bridge"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            bridge_instance = FoxgloveBridge(host="0.0.0.0", port=8765, debug=True, num_threads=4)

            loop.run_until_complete(bridge_instance.run())
        except Exception as e:
            print(f"Bridge error: {e}")
        finally:
            loop.close()

    thread = threading.Thread(target=bridge_thread, daemon=True)
    thread.start()

    print("Bridge started in background thread")
    print("Open Foxglove Studio and connect to ws://localhost:8765")
    print("Press Ctrl+C to exit")

    try:
        while True:
            threading.Event().wait(1)
    except KeyboardInterrupt:
        print("Shutting down...")


def main() -> None:
    run_bridge_example()


if __name__ == "__main__":
    main()
