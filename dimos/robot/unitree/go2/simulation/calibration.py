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

"""Existing Go2 compiled-model fit from real rollouts; not policy parameters."""

FITTED_PHYSICS = {
    "armature": 0.00712,
    "damping": 0.2850,
    "frictionloss": 0.3650,
    "trunk_mass_scale": 0.9412,
    "trunk_inertia_scale": 0.8601,
    "foot_friction": 0.7860,
    "foot_friction_torsional": 0.006134,
    "trunk_com_x": -0.006845,
    "leg_mass_scale": 1.616,
}
FITTED_ACTUATOR_TAU = 0.01510
