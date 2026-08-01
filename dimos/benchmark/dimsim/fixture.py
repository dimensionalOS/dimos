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

"""Minimal typed apartment fixture for hermetic compiler contract tests."""

from dimos.benchmark.dimsim.config import FRAME_POLICY_VERSION
from dimos.benchmark.dimsim.models import (
    Embodiment,
    Entity,
    FrameContract,
    NavigationGeometry,
    Pose2,
    ProvenanceGroup,
    Region,
    SceneOracleView,
    SemanticProperty,
)


def _box(x: float, z: float, width: float, depth: float) -> tuple[tuple[float, float], ...]:
    return (
        (x - width / 2, z - depth / 2),
        (x + width / 2, z - depth / 2),
        (x + width / 2, z + depth / 2),
        (x - width / 2, z + depth / 2),
    )


def apartment_oracle_fixture(*, television_power: str = "OFF") -> SceneOracleView:
    """Return a fixture shaped like the private DimSim v1 oracle contract.

    This fixture validates compiler behavior. It is not production scene truth.
    """

    entities = (
        Entity(
            entity_id="apt-bathtub-01",
            semantic_class="bathtub",
            aliases=("bathtub",),
            display_title="Apartment bath",
            position=(2.0, 2.0),
            yaw_rad=0.0,
            footprint=_box(2.0, 2.0, 1.8, 0.8),
            region_ids=("apt-main",),
        ),
        Entity(
            entity_id="apt-television-01",
            semantic_class="television",
            aliases=("television", "TV"),
            display_title="Living room display",
            position=(8.0, 7.0),
            yaw_rad=0.0,
            footprint=_box(8.0, 7.0, 1.2, 0.3),
            properties=(
                SemanticProperty(
                    name="power",
                    value=television_power,
                    authoritative=True,
                ),
            ),
            region_ids=("apt-main",),
        ),
        Entity(
            entity_id="apt-sofa-01",
            semantic_class="sofa",
            aliases=("sofa",),
            display_title="Living room sofa",
            position=(4.0, 2.0),
            yaw_rad=0.0,
            footprint=_box(4.0, 2.0, 2.0, 0.9),
            region_ids=("apt-main",),
        ),
        *tuple(
            Entity(
                entity_id=f"apt-dining-chair-{index:02d}",
                semantic_class="dining-chair",
                aliases=("dining chair",),
                display_title=f"Seat {index}",
                position=(5.0 + index * 0.5, 5.0),
                yaw_rad=0.0,
                footprint=_box(5.0 + index * 0.5, 5.0, 0.4, 0.4),
                region_ids=("apt-main",),
            )
            for index in range(1, 5)
        ),
        Entity(
            entity_id="apt-work-chair-01",
            semantic_class="work-chair",
            aliases=("work chair",),
            display_title="Dining-looking office chair",
            position=(9.0, 2.0),
            yaw_rad=0.0,
            footprint=_box(9.0, 2.0, 0.5, 0.5),
            region_ids=("apt-main",),
        ),
    )
    return SceneOracleView(
        scene_id="dimsim-apartment",
        scene_revision="apartment-contract-fixture-v1",
        reset_revision=f"canonical-reset-power-{television_power.lower()}-v1",
        upstream_revision="fixture-mirrors-dimsim-oracle-schema-v1",
        profile_revision="dimsim-apartment-profile-v1",
        frame=FrameContract(
            frame_id="dimsim-world",
            handedness="right",
            length_unit="metre",
            gravity_axis="-Y",
            horizontal_axes="XZ",
            transform_convention="world-from-local",
            policy_version=FRAME_POLICY_VERSION,
        ),
        embodiment=Embodiment(
            embodiment_id="unitree-go2",
            footprint_radius_m=0.12,
            canonical_spawn=Pose2(x_m=1.0, z_m=1.0, yaw_rad=0.0),
        ),
        navigation=NavigationGeometry(
            navigable=(_box(5.0, 5.0, 10.0, 10.0),),
        ),
        entities=entities,
        regions=(
            Region(
                region_id="apt-main",
                semantic_class="apartment",
                footprint=_box(5.0, 5.0, 10.0, 10.0),
            ),
        ),
        provenance=(
            ProvenanceGroup(
                source_kind="authored",
                source_revision="apartment-contract-fixture-v1",
                field_paths=(
                    "entities.*.semantic_class",
                    "entities.*.aliases",
                    "entities.*.properties",
                    "entities.*.footprint",
                    "entities.*.region_ids",
                    "regions.*",
                    "navigation.*",
                    "embodiment.canonical_spawn",
                ),
            ),
        ),
    )
