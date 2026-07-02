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

"""Demo: blink an Arduino's built-in LED from Python.

    uv run python examples/arduino_led_echo/demo_led_blink.py

Requires an Arduino Uno plugged in and ``nix`` on PATH.
"""

from __future__ import annotations

import time

from dimos.core.arduino_module import ArduinoModule, ArduinoModuleConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.Bool import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Arduino module


class LedEchoConfig(ArduinoModuleConfig):
    # basically all of the arduino module lives inside sketch.ino
    sketch_path: str = "sketch/sketch.ino"
    board_fqbn: str = "arduino:avr:uno"
    baudrate: int = 115200


class LedEcho(ArduinoModule):
    config: LedEchoConfig
    led_cmd: In[Bool]
    led_state: Out[Bool]


# Host-side blinker


class Blinker(Module):
    config: ModuleConfig

    led_cmd: Out[Bool]
    led_state: In[Bool]

    @rpc
    def start(self) -> None:
        super().start()

        # output from the arduino
        self.led_state.subscribe(
            lambda msg: logger.info(f"LED STATUS: {'ON' if msg.data else 'OFF'}")
        )

        # input to the arduino
        led_is_on = True
        count = 20
        while count > 0:
            count -= 1
            self.led_cmd.publish(Bool(data=led_is_on))
            led_is_on = not led_is_on
            time.sleep(0.5)


# Blueprint & run

blueprint = autoconnect(
    Blinker.blueprint(),
    LedEcho.blueprint(),
).global_config(n_workers=2)

if __name__ == "__main__":
    ModuleCoordinator.build(blueprint).loop()
