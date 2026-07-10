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

"""Example ArduinoModule demonstrating bidirectional DimOS<->Arduino comms."""

from __future__ import annotations

from dimos.core.stream import In, Out
from dimos.experimental.arduino.arduino_module import ArduinoModule, ArduinoModuleConfig
from dimos.msgs.geometry_msgs.Twist import Twist


class ArduinoTwistEchoConfig(ArduinoModuleConfig):
    sketch_path: str = "sketch/sketch.ino"
    board_fqbn: str = "arduino:avr:uno"
    baudrate: int = 115200

    # embedded as #define DIMOS_ECHO_DELAY_MS 50
    echo_delay_ms: int = 50


class ArduinoTwistEcho(ArduinoModule):
    config: ArduinoTwistEchoConfig

    twist_in: In[Twist]
    twist_echo_out: Out[Twist]
