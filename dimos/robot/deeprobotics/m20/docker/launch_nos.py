#!/usr/bin/env python3
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

"""Host-side launcher for M20 ROSNav on the NOS.

Builds and runs the m20_rosnav blueprint (M20Connection + VoxelGridMapper +
CostMapper + ROSNav). Handles SIGINT/SIGTERM for graceful shutdown.
"""

import signal
import sys

from dimos.robot.deeprobotics.m20.blueprints.rosnav.m20_rosnav import m20_rosnav


def main() -> None:
    coordinator = m20_rosnav.build()

    def _shutdown(signum: int, _frame: object) -> None:
        coordinator.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    coordinator.loop()


if __name__ == "__main__":
    main()
