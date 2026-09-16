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

"""Go2 connection manifest: the implementation each ``unitree_connection_type`` names.

The dependency catalog reads this table statically and ``make_connection``
resolves it at run time, so planning and execution agree on what a
configuration value selects. Keep it free of imports.
"""

CONNECTION_FACTORIES = {
    "webrtc": "dimos.robot.unitree.connection:UnitreeWebRTCConnection",
    "replay": "dimos.robot.unitree.go2.connection:ReplayConnection",
    "mujoco": "dimos.robot.unitree.mujoco_connection:MujocoConnection",
    "dimsim": "dimos.robot.unitree.dimsim_connection:DimSimConnection",
}
