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

"""The logistics room with three pushable props: two crates and a ball, down the corridor along +x.

The room itself is `logistics.usda` by reference, so there is one converted copy of it. Each prop
is a rigid body with an authored mass, which is what makes it a thing a robot can walk into and
move rather than another wall.
"""

from pathlib import Path

LOGISTICS_PROPS = Path(__file__).parent / "logistics_props.usda"
