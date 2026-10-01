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
"""`unitree-go2-typesafe` plus a fixed demo object so `go to the chair` has a target.

dimos --simulation dimsim --no-dimsim-headless run unitree-go2-typesafe-demo
"""

from dimos.agents.typesafe.demo_objects import DemoObjects
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_typesafe import unitree_go2_typesafe

unitree_go2_typesafe_demo = autoconnect(
    unitree_go2_typesafe,
    DemoObjects.blueprint(),
)
