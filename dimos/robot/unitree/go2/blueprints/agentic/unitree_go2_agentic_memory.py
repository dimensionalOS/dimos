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

"""Go2 agent stack with a live recording available through Python."""

from dimos.agents.skills.run_python import RunPythonSkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import unitree_go2_agentic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import Go2Memory

unitree_go2_agentic_memory = autoconnect(
    unitree_go2_agentic,
    Go2Memory.blueprint(),
    RunPythonSkill.blueprint(),
).global_config(n_workers=12, robot_model="unitree_go2")
