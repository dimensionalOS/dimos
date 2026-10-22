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

"""Optional native viewer process. It reads snapshots and never steps physics."""

import json
from pathlib import Path
import sys
import time

import mujoco.viewer

from dimos.sim2.sensors.reader import WorldReader


def main() -> None:
    reader = WorldReader(json.loads(Path(sys.argv[1]).read_text()))
    try:
        reader.update()
        with mujoco.viewer.launch_passive(reader.model, reader.data) as viewer:
            viewer.opt.geomgroup[:] = 1
            while viewer.is_running() and reader.channel.lifecycle == "ready":
                with viewer.lock():
                    reader.update()
                viewer.sync()
                time.sleep(1.0 / 30.0)
    finally:
        reader.close()


if __name__ == "__main__":
    main()
