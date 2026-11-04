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

"""The vendor's M20 URDF, and its standing pose for robots that do not report joint angles."""

from pathlib import Path
import threading
import time
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel

# The vendor's URDF and meshes (BSD-3-Clause, assets/LICENSE); M20Connection's camera relay owns the camera frames.
M20_MODEL = RobotModel.from_file(
    Path(__file__).parent / "assets" / "M20.urdf"
).without_joint_subtrees("front_camera_joint", "rear_camera_joint")

# Hip pitch and knee of the vendor SDK's standing pose; front knees fold back, hind knees forward.
STAND_JOINT_POSITIONS = {
    **{f"{leg}_hipx_joint": 0.0 for leg in ("fl", "fr", "hl", "hr")},
    **{f"{leg}_hipy_joint": -0.3 for leg in ("fl", "fr")},
    **{f"{leg}_knee_joint": 0.6 for leg in ("fl", "fr")},
    **{f"{leg}_hipy_joint": 0.3 for leg in ("hl", "hr")},
    **{f"{leg}_knee_joint": -0.6 for leg in ("hl", "hr")},
}


class M20StandingJointState(Module):
    """Publishes the standing pose as the joint state; the M20 bridge does not expose measured joint angles yet."""

    coordinator_joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._publish, name="m20-standing-joint-state", daemon=True
        )
        self._thread.start()

    def _publish(self) -> None:
        while not self._stop_event.is_set():
            self.coordinator_joint_state.publish(
                JointState(
                    ts=time.time(),
                    name=list(STAND_JOINT_POSITIONS),
                    position=list(STAND_JOINT_POSITIONS.values()),
                )
            )
            self._stop_event.wait(1.0)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()
