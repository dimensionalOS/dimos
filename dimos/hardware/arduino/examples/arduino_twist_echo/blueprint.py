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

"""Blueprint: virtual Arduino TwistEcho + a test publisher.

Run with:
    dimos run arduino-twist-echo-virtual

Or, since this lives outside the auto-discovery path, run by importing
this module's variable from a script.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.arduino.examples.arduino_twist_echo.module import TwistEcho
from dimos.hardware.arduino.examples.arduino_twist_echo.test_publisher import (
    TestPublisher,
)

# Two modules wired by autoconnect via stream name+type matching:
#   TestPublisher.cmd_out      (Out[Twist])  ──┐
#   TwistEcho.twist_in         (In[Twist])  ◀──┘  via remapping
#
#   TwistEcho.twist_echo_out   (Out[Twist])  ──┐
#   TestPublisher.echo_in      (In[Twist])   ◀─┘  via remapping
arduino_twist_echo_virtual = (
    autoconnect(
        TestPublisher.blueprint(publish_period_s=0.5),
        TwistEcho.blueprint(virtual=True),
    )
    .remappings(
        [
            # TestPublisher.cmd_out → TwistEcho.twist_in
            (TestPublisher, "cmd_out", "twist_command"),
            (TwistEcho, "twist_in", "twist_command"),
            # TwistEcho.twist_echo_out → TestPublisher.echo_in
            (TwistEcho, "twist_echo_out", "twist_echo"),
            (TestPublisher, "echo_in", "twist_echo"),
        ]
    )
    .global_config(n_workers=2)
)
