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
"""Go2 driven by TypeSafe: `go to the <object>` on /human_input; objects on `detections_3d`."""

from dimos.agents.typesafe.agent import typesafe_api_key
from dimos.agents.typesafe.navigation import TypeSafeNavigationAgent
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

unitree_go2_typesafe = autoconnect(
    unitree_go2_basic, TypeSafeNavigationAgent.blueprint()
).requirements(typesafe_api_key)
