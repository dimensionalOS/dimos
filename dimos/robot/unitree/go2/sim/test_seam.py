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

"""The seam stays engine-free. Runs everywhere — no mujoco, no menagerie."""

from __future__ import annotations

import subprocess
import sys


def test_importing_the_seam_never_imports_an_engine():
    """backend.py is the seam, not the simulator: with two backends installed,
    a seam that imports its engine makes using either import both. A clean
    interpreter proves it — in-process, some other test has already imported
    mujoco and the check would be vacuous."""
    code = (
        "import sys\n"
        "import dimos.robot.unitree.go2.sim.backend\n"
        "import dimos.robot.unitree.go2.sim.sysid.regimes\n"
        "import dimos.robot.unitree.go2.sim.sysid.replay\n"
        "import dimos.robot.unitree.go2.sim.sysid.score\n"
        "import dimos.robot.unitree.go2.sim.sysid.rollouts\n"
        "import dimos.robot.unitree.go2.sim.sysid.fit\n"
        "banned = [m for m in sys.modules if m == 'mujoco' or m.startswith('mujoco.')]\n"
        "assert not banned, f'the seam dragged in the engine: {banned}'\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
