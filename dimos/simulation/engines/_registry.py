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

"""Simulation module manifest: the module blueprint each ``--simulation`` value selects.

Read statically by the dependency catalog and at run time by the manipulator
blueprints, so both agree on what a value installs and starts. Keep it free
of imports.
"""

SIM_MODULE_FACTORIES = {
    "mujoco": "dimos.simulation.engines.mujoco_sim_module:MujocoSimModule",
}
