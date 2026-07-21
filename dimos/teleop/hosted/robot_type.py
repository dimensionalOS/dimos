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

"""Robot kinds for hosted teleop - the cockpits the operator UI can render.

Command modules declare their kind and push it to the broker, which sends it in
the session POST. The broker re-validates the wire string against its own copy.
"""

from enum import StrEnum


class RobotType(StrEnum):
    """Operator cockpit kind; value is the wire string sent to the broker."""

    GO2 = "go2"
    XARM = "xarm"
