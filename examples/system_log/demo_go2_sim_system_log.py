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

"""The simulated Go2 with ``system_log`` alongside it, both recorded into one memory db.

Click in the rerun viewer to drive the dog. The machine's health, alerts, dimos' own logs and (with ``--dtop``)
per-module usage land on ``system_log`` next to the robot's odometry, in ``recordings/<time>/mem2.db`` (the run's
logs go in that folder too)::

    python examples/system_log/demo_go2_sim_system_log.py

Without the recorder, the same pairing is just ``dimos --dtop run go2-sim system-log``.
"""

from __future__ import annotations

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.stream import In
from dimos.diagnostics.system_log.module import SystemLog
from dimos.memory.module import Recorder, default_recording_dir, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.std_msgs.String import String
from dimos.robot.unitree.go2.blueprints.basic.go2_sim import go2_sim
from dimos.utils.logging_config import set_run_log_dir

recording_dir = default_recording_dir()


class SystemLogRecorder(Recorder):
    system_log: In[String]
    odometry: In[Odometry]

    @pose_setter_for("odometry")
    async def _odometry_pose(self, msg: Odometry) -> Pose:
        return msg.pose.pose


demo_go2_sim_system_log = autoconnect(
    go2_sim,
    SystemLog.blueprint(),
    SystemLogRecorder.blueprint(
        db_path=str(recording_dir / "mem2.db"), poseless_streams=["system_log"]
    ),
)


if __name__ == "__main__":
    # system_log tails the run's logs from here, as it does under `dimos run`
    set_run_log_dir(recording_dir)
    parsed_config = BlueprintConfigParser(demo_go2_sim_system_log).parse()
    ModuleCoordinator.build(demo_go2_sim_system_log, parsed_config).loop()
