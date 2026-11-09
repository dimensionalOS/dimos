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

"""The M20's model, from the vendor's MJCF, and its standing pose for robots that do not report joint angles."""

import threading
from typing import Any

import mujoco
import numpy as np

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.message_utils import time as message_time
from dimos.messages.sensor_msgs.msg import JointState
from dimos.robot.assets.mjcf import mjcf_to_urdf
from dimos.robot.deeprobotics.m20.assets.description import SparseDescriptionSource

_SDK = "src/M20_sdk_deploy"
_MJCF_DIR = f"{_SDK}/M20_description/m20_mjcf"
# The vendor's deploy repo: its MJCF (what the walking policy runs with), its meshes and the policy itself.
M20_SDK_SOURCE = SparseDescriptionSource(
    url="https://github.com/DeepRoboticsLab/sdk_deploy",
    ref="2367375f922f2a739f7b578849ae2affd4d8660d",
    sparse_paths=(
        f"{_MJCF_DIR}/mjcf/M20.xml",
        f"{_MJCF_DIR}/meshes/*.STL",
        f"{_SDK}/policy/policy.onnx",
    ),
)
M20_MJCF = M20_SDK_SOURCE / _MJCF_DIR / "mjcf" / "M20.xml"
M20_POLICY = M20_SDK_SOURCE / _SDK / "policy" / "policy.onnx"
# The URDF the real robot and the sim show, derived from the same MJCF the sim steps.
M20_MODEL = mjcf_to_urdf(mujoco.MjSpec.from_file(str(M20_MJCF)))

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
                    header=message_time.header_now(),
                    name=list(STAND_JOINT_POSITIONS),
                    position=np.asarray(list(STAND_JOINT_POSITIONS.values()), dtype=np.float64),
                    velocity=np.asarray([], dtype=np.float64),
                    effort=np.asarray([], dtype=np.float64),
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
