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

"""Multi-type echo ArduinoModule for hardware round-trip testing.

Echoes Bool, Int32, Vector3, and Quaternion messages to validate
serialization correctness and float64→float32 precision on AVR.
"""

from __future__ import annotations

from dimos.core.arduino_module import ArduinoModule, ArduinoModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32


class MultiEchoConfig(ArduinoModuleConfig):
    sketch_path: str = "sketch/sketch.ino"
    board_fqbn: str = "arduino:avr:uno"
    baudrate: int = 115200


class MultiEcho(ArduinoModule):
    """Arduino that echoes multiple message types back."""

    config: MultiEchoConfig

    bool_in: In[Bool]
    bool_out: Out[Bool]

    int32_in: In[Int32]
    int32_out: Out[Int32]

    vec3_in: In[Vector3]
    vec3_out: Out[Vector3]

    quat_in: In[Quaternion]
    quat_out: Out[Quaternion]
