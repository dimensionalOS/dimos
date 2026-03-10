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


from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import (  # type: ignore[attr-defined]
    JointCommand,
    JointState,
    RobotState,
)
from dimos.msgs.trajectory_msgs import JointTrajectory
from dimos.simulation.manipulators.sim_module import simulation
from dimos.utils.data import LfsPath

xarm7_trajectory_sim = simulation(
    engine="mujoco",
    config_path=LfsPath("xarm7/scene.xml"),
    headless=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/xarm/joint_position_command", JointCommand
        ),
        ("trajectory", JointTrajectory): LCMTransport("/trajectory", JointTrajectory),
    }
)


__all__ = [
    "simulation",
    "xarm7_trajectory_sim",
]

if __name__ == "__main__":
    xarm7_trajectory_sim.build().loop()
