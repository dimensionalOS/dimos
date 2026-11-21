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

from dimos.core.coordination.blueprints import autoconnect
from dimos.teleop.esp32.esp32_gesture_teleop import Esp32GestureTeleop

# ESP32 gesture sensor -> Joy on `joystick`. Input only: nothing here moves a robot.
teleop_esp32 = autoconnect(
    Esp32GestureTeleop.blueprint(),
)
