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

"""A sandbox case on a mock wheeled base: drive it, then report what it says about itself.

dimos evals sandbox dimos.evals.sandbox.example
"""

import json

from dimos.evals.sandbox.case import DimosRobot, FunctionRef, SandboxCase, SandboxOutcome

PROMPT = """\
You are connected to a mobile robot. Its only interface is a Zenoh router at tcp/robot:7447
(connect to it in client mode; `pip install eclipse-zenoh` works).

  robot/arm/state/json   {"t", "joint_names", "positions", "velocities"}: the base's state
  robot/cmd_vel/json     publish {"vx": m/s, "vy": m/s, "wz": rad/s, "t": seconds}; the robot
                         holds that velocity for t seconds (max 2), then stops.

Drive the robot forward at 0.2 m/s for about 2 seconds. Leave the script you used at
/root/work/drive.py. Then call done with the joint names the robot reports.
"""


def evaluate(o: SandboxOutcome) -> float:
    assert o.recording is not None
    commands = [json.loads(s.payload) for s in o.recording.samples("robot/cmd_vel/json")]
    drove = any(abs(c.get("vx", 0) - 0.2) < 0.05 for c in commands)
    reported = bool(o.done["joint_names"] == ["base/vx", "base/vy", "base/wz"])
    kept_script = o.exec("test -f /root/work/drive.py").returncode == 0
    return (drove + reported + kept_script) / 3


SUITE = [
    SandboxCase(
        id="drive_forward",
        prompt=PROMPT,
        robot=DimosRobot(
            args=("run", "coordinator-mock-twist-base", "raw-robot-bridge"),
            forward=("tcp/127.0.0.1:7448",),  # raw-robot-bridge only listens
        ),
        ready_key="robot/arm/state/json",
        done_schema={
            "type": "object",
            "properties": {"joint_names": {"type": "array", "items": {"type": "string"}}},
            "required": ["joint_names"],
            "additionalProperties": False,
        },
        evaluate=FunctionRef(function=f"{__name__}:evaluate"),
        timeout_s=600.0,
    ),
]
