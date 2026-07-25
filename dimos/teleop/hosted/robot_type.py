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

"""Robot kinds for hosted teleop - the operator views the UI can render.

Pinned on a broker transport spec in the hosted blueprint (robot_type=...); the
value is the wire string sent to the broker in the session-create POST.
"""

from enum import Enum


class RobotType(str, Enum):
    """Operator view kind; value is the wire string sent to the broker.

    ``str`` mixin (not ``StrEnum``, which is 3.11+) so the member serializes as
    its wire value and the package still imports on the 3.10 floor.
    """

    GO2 = "go2"
    ARM = "arm"
