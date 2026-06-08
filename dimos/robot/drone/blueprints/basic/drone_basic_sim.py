#!/usr/bin/env python3

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

"""Basic drone sim: visualization and mujoco connection."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.drone.blueprints.primitive.drone_primitive_no_nav import drone_primitive_no_nav
from dimos.robot.drone.mujoco_sim import DroneSimConnection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

drone_basic_sim = autoconnect(
    drone_primitive_no_nav,
    DroneSimConnection.blueprint(),
    KeyboardTeleop.blueprint(publish_only_when_active=True),
)

__all__ = ["drone_basic_sim"]
