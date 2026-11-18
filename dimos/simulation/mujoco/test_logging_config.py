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

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize("destination", ["inherited-run", "local-run", "standalone"])
def test_simulation_warnings_use_dimos_logs(tmp_path: Path, destination: str) -> None:
    # Isolate both the process-wide MuJoCo callback and DimOS logging configuration.
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
from pathlib import Path
import sys

import mujoco

from dimos.utils import logging_config

root = Path.cwd()
logging_config.LOG_DIR = root / "logs"

from dimos.simulation.engines.mujoco_engine import MujocoEngine

xml = root / "robot.xml"
xml.write_text('''<mujoco><worldbody><body>
  <freejoint/><geom type="sphere" size="0.1" mass="1"/>
</body></worldbody></mujoco>''')
engine = MujocoEngine(config_path=xml, headless=True)
try:
    if sys.argv[1] == "local-run":
        logging_config.set_run_log_dir(root / "run")
    engine.data.qfrc_applied[3] = float("inf")
    mujoco.mj_step(engine.model, engine.data)
finally:
    engine.disconnect()
""",
            destination,
        ],
        cwd=tmp_path,
        env=os.environ
        | {
            "DIMOS_RUN_LOG_DIR": str(tmp_path / "run") if destination == "inherited-run" else "",
            "DIMOS_LOG_LEVEL": "INFO",
        },
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert not (tmp_path / "MUJOCO_LOG.TXT").exists()
    log_dir = tmp_path / ("logs" if destination == "standalone" else "run")
    (log_file,) = log_dir.glob("*.jsonl")
    records = [json.loads(line) for line in log_file.read_text().splitlines()]
    (warning,) = [record for record in records if record["level"] == "warning"]
    assert warning["warning"] == (
        "Nan, Inf or huge value in QACC at DOF 3. The simulation is unstable. Time = 0.0000."
    )
    assert warning["warning"] in result.stdout
