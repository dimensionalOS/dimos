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

"""go2/ControlEvent — app-level JSON control-log events (topic ``control_log``).

A tagged event stream keyed by ``type`` (e.g. ``velocity_input`` carries
``lx/ly/az``; ``brightness`` carries ``level``). Fields beyond ``type`` are
optional; :class:`~dimos.robot.unitree.go2dds.codec.JsonCodec` drops keys it
doesn't recognise, so new event shapes won't break decoding.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.robot.unitree.go2dds.msgs.base import PrettyMsg


@dataclass(repr=False)
class ControlEvent(PrettyMsg):
    type: str
    lx: float | None = None  # velocity_input
    ly: float | None = None
    az: float | None = None
    level: int | None = None  # brightness
