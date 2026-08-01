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

"""Versioned DimSim benchmark-generation policy."""

from typing import Final

SCHEMA_VERSION: Final = "1.0"
SEMANTIC_SCHEMA_VERSION: Final = "1.0"
GENERATOR_REVISION: Final = "dimsim-smoke-generator-v2"
FRAME_POLICY_VERSION: Final = "threejs-y-up-xz-v1"
CLEARANCE_POLICY_VERSION: Final = "rapier-grid-clearance-v1"
PREDICATE_POLICY_VERSION: Final = "dimsim-smoke-predicates-v2"
TEMPLATE_VERSION: Final = "dimsim-smoke-en-v1"
DESTINATION_THRESHOLD_M: Final = 1.0
DESTINATION_LINEAR_SPEED_TOLERANCE_M_S: Final = 0.05
DESTINATION_ANGULAR_SPEED_TOLERANCE_RAD_S: Final = 0.1
DESTINATION_STATIONARY_DWELL_S: Final = 1.0
EMBODIMENT_CLEARANCE_M: Final = 0.05
NAVIGATION_GRID_RESOLUTION_M: Final = 0.1
NAVIGATION_GROUND_TOLERANCE_M: Final = 0.08
COMPARISON_STABILITY_MARGIN_M: Final = 0.25
POWER_VOCABULARY: Final = ("ON", "OFF")

CATEGORY_ORDER: Final = (
    "destination",
    "targeted-qa",
    "broad-exploration-qa",
    "multi-hop-qa",
)

PUBLIC_TEMPLATES: Final = {
    "destination": "Go to the bathtub and stop within 1 meter of its outer edge.",
    "targeted-qa": "Is the television ON or OFF?",
    "broad-exploration-qa": "How many dining chairs are in the apartment?",
    "multi-hop-qa": "Which is closer to the sofa: the bathtub or the television?",
}
