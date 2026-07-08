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

"""Blueprint: virtual ArduinoTwistEcho wired to a test publisher."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.experimental.arduino.examples.arduino_twist_echo.module import ArduinoTwistEcho
from dimos.experimental.arduino.examples.arduino_twist_echo.test_publisher import (
    TestPublisher,
)

# Two modules wired by autoconnect via stream name+type matching:
#   TestPublisher.cmd_out      (Out[Twist])  ──┐
#   ArduinoTwistEcho.twist_in         (In[Twist])  ◀──┘  via remapping
#
#   ArduinoTwistEcho.twist_echo_out   (Out[Twist])  ──┐
#   TestPublisher.echo_in      (In[Twist])   ◀─┘  via remapping
arduino_msg_example = (
    autoconnect(
        TestPublisher.blueprint(publish_period_s=0.5),
        ArduinoTwistEcho.blueprint(virtual=True),
    )
    .remappings(
        [
            # TestPublisher.cmd_out → ArduinoTwistEcho.twist_in
            (TestPublisher, "cmd_out", "twist_command"),
            (ArduinoTwistEcho, "twist_in", "twist_command"),
            # ArduinoTwistEcho.twist_echo_out → TestPublisher.echo_in
            (ArduinoTwistEcho, "twist_echo_out", "twist_echo"),
            (TestPublisher, "echo_in", "twist_echo"),
        ]
    )
    .global_config(n_workers=2)
)
