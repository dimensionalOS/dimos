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

from dimos_generated.geometry_msgs.msg import Pose
import mujoco

from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport


class PersonPositionController:
    """Controls the person position in MuJoCo by subscribing to LCM pose updates."""

    def __init__(self, model: mujoco.MjModel) -> None:
        person_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "person")
        self._person_mocap_id = model.body_mocapid[person_body_id]
        self._latest_pose: Pose | None = None
        self._transport: PubSubTransport[Pose] = make_transport("/person_pose", Pose)
        self._transport.subscribe(self._on_pose)

    def _on_pose(self, pose: Pose) -> None:
        self._latest_pose = pose

    def tick(self, data: mujoco.MjData) -> None:
        if self._latest_pose is None:
            return

        pose = self._latest_pose
        data.mocap_pos[self._person_mocap_id][0] = pose.position.x
        data.mocap_pos[self._person_mocap_id][1] = pose.position.y
        data.mocap_pos[self._person_mocap_id][2] = pose.position.z
        data.mocap_quat[self._person_mocap_id] = [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]

    def stop(self) -> None:
        self._transport.stop()
