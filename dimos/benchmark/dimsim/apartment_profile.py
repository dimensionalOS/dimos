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

"""Versioned semantic bindings for the pinned DimSim apartment scene."""

from dataclasses import dataclass
from typing import Final

from dimos.simulation.dimsim.revision import DIMSIM_REPO_COMMIT

APARTMENT_PROFILE_REVISION: Final = "dimsim-apartment-profile-v1"
APARTMENT_SCENE_ID: Final = "dimsim-apartment"
APARTMENT_SCENE_REVISION: Final = f"apt@{DIMSIM_REPO_COMMIT}"
APARTMENT_REGION_ID: Final = "apartment-profile-region"
APARTMENT_NAVIGATION_POLYGON: Final = (
    (-5.0, -5.5),
    (5.5, -5.5),
    (5.5, 5.5),
    (-5.0, 5.5),
)
APARTMENT_CANONICAL_SPAWN: Final = (2.0, 3.0)
APARTMENT_EMBODIMENT_RADIUS_M: Final = 0.12
APARTMENT_EMBODIMENT_HALF_HEIGHT_M: Final = 0.25
APARTMENT_ASSET_COUNT: Final = 123
APARTMENT_COLLISION_SOURCE_COUNT: Final = 167


@dataclass(frozen=True)
class ApartmentEntityProfile:
    entity_id: str
    semantic_class: str
    aliases: tuple[str, ...]
    state_property: str | None = None
    state_values: tuple[tuple[str, str], ...] = ()


APARTMENT_ENTITY_PROFILES: Final = (
    ApartmentEntityProfile(
        entity_id="6eaff768b565c-19c7335c107",
        semantic_class="bathtub",
        aliases=("bathtub",),
    ),
    ApartmentEntityProfile(
        entity_id="c7899f757219b-19c731bc6ff",
        semantic_class="television",
        aliases=("television", "TV"),
        state_property="power",
        state_values=(
            ("state-mlr4bgf0-u7gv", "OFF"),
            ("state-mlr4dvjl-fyw3", "ON"),
        ),
    ),
    ApartmentEntityProfile(
        entity_id="59a525468c75d8-19c73105016",
        semantic_class="sofa",
        aliases=("sofa",),
    ),
    ApartmentEntityProfile(
        entity_id="80c4a613fb7268-19c73c9ac7a",
        semantic_class="dining-chair",
        aliases=("dining chair",),
    ),
    ApartmentEntityProfile(
        entity_id="b521e3d18de31-19c73cc4ab7",
        semantic_class="dining-chair",
        aliases=("dining chair",),
    ),
    ApartmentEntityProfile(
        entity_id="c4896d1ee8f58-19c73cca1c3",
        semantic_class="dining-chair",
        aliases=("dining chair",),
    ),
    ApartmentEntityProfile(
        entity_id="5870c87d73e6e-19c73cd1041",
        semantic_class="dining-chair",
        aliases=("dining chair",),
    ),
    ApartmentEntityProfile(
        entity_id="5028f5326c4f1-19c73300d7f",
        semantic_class="work-chair",
        aliases=("work chair",),
    ),
)

APARTMENT_ENTITY_IDS: Final = tuple(profile.entity_id for profile in APARTMENT_ENTITY_PROFILES)
