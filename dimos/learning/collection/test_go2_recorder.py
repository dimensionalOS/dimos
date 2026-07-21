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

from dimos.learning.collection.go2_recorder import Go2CollectionRecorder
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic_record import (
    unitree_go2_agentic_record,
)


def test_cmd_vel_action_stream_has_producer_and_recorder() -> None:
    cmd_vel_refs = [
        (atom.module, stream.direction)
        for atom in unitree_go2_agentic_record.active_blueprints
        for stream in atom.streams
        if stream.name == "cmd_vel" and stream.type is Twist
    ]

    assert (MovementManager, "out") in cmd_vel_refs
    assert (Go2CollectionRecorder, "in") in cmd_vel_refs
