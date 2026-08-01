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

"""Private providers for coherent DimSim oracle views."""

from collections.abc import Callable
import hashlib
import json
import time
from typing import Protocol

from dimos.benchmark.dimsim.apartment_profile import (
    APARTMENT_ASSET_COUNT,
    APARTMENT_CANONICAL_SPAWN,
    APARTMENT_COLLISION_SOURCE_COUNT,
    APARTMENT_EMBODIMENT_HALF_HEIGHT_M,
    APARTMENT_EMBODIMENT_RADIUS_M,
    APARTMENT_ENTITY_IDS,
    APARTMENT_ENTITY_PROFILES,
    APARTMENT_NAVIGATION_POLYGON,
    APARTMENT_PROFILE_REVISION,
    APARTMENT_REGION_ID,
    APARTMENT_SCENE_ID,
    APARTMENT_SCENE_REVISION,
    ApartmentEntityProfile,
)
from dimos.benchmark.dimsim.config import (
    EMBODIMENT_CLEARANCE_M,
    FRAME_POLICY_VERSION,
    NAVIGATION_GRID_RESOLUTION_M,
    NAVIGATION_GROUND_TOLERANCE_M,
    SEMANTIC_SCHEMA_VERSION,
)
from dimos.benchmark.dimsim.models import (
    Embodiment,
    Entity,
    FrameContract,
    NavigationGeometry,
    Pose2,
    ProvenanceGroup,
    Region,
    RuntimeEntitySnapshot,
    RuntimeNavigationGrid,
    RuntimeSceneSnapshot,
    SceneOracleView,
    SemanticProperty,
)
from dimos.benchmark.spatial.utilities import canonical_json
from dimos.simulation.dimsim.revision import DIMSIM_REPO_COMMIT
from dimos.simulation.dimsim.scene_client import SceneClient

_BOUNDS_POLICY_VERSION = "threejs-world-aabb-xz-v1"
_NAVIGATION_POLICY_VERSION = "rapier-reachable-grid-v1"
_SPAWN_TOLERANCE_M = 0.05
_EMBODIMENT_TOLERANCE_M = 1e-6


class OracleCompatibilityError(ValueError):
    """Raised when a live scene does not match the versioned profile."""


class SceneOracleProvider(Protocol):
    def get_scene_oracle_view(self) -> SceneOracleView:
        """Return one validated, coherent scene-reset view."""


class InMemorySceneOracleProvider:
    def __init__(self, view: SceneOracleView) -> None:
        self._view = view

    def get_scene_oracle_view(self) -> SceneOracleView:
        return self._view


class SceneClientOracleProvider:
    def __init__(self, client: SceneClient) -> None:
        self._client = client

    def get_scene_oracle_view(self) -> SceneOracleView:
        xs = [point[0] for point in APARTMENT_NAVIGATION_POLYGON]
        zs = [point[1] for point in APARTMENT_NAVIGATION_POLYGON]
        payload = self._client.get_scene_oracle_snapshot(
            APARTMENT_ENTITY_IDS,
            snapshot_schema_version=SEMANTIC_SCHEMA_VERSION,
            navigation_bounds=(min(xs), min(zs), max(xs), max(zs)),
            navigation_resolution_m=NAVIGATION_GRID_RESOLUTION_M,
            embodiment_clearance_m=EMBODIMENT_CLEARANCE_M,
            ground_tolerance_m=NAVIGATION_GROUND_TOLERANCE_M,
        )
        try:
            snapshot = RuntimeSceneSnapshot.model_validate_json(
                json.dumps(payload, allow_nan=False, separators=(",", ":"), sort_keys=True)
            )
        except (TypeError, ValueError) as error:
            raise OracleCompatibilityError(
                "DimSim returned an unsupported or malformed scene snapshot"
            ) from error
        return apartment_oracle_view_from_snapshot(snapshot)


def get_stable_scene_oracle_view(
    provider: SceneOracleProvider,
    *,
    stability_delay_s: float = 1.0,
    pause: Callable[[float], None] = time.sleep,
) -> SceneOracleView:
    """Require two identical live views before freezing benchmark identity."""
    if stability_delay_s < 0:
        raise ValueError("stability delay must be non-negative")
    first = provider.get_scene_oracle_view()
    pause(stability_delay_s)
    second = provider.get_scene_oracle_view()
    if second != first:
        raise OracleCompatibilityError(
            "DimSim scene changed between oracle snapshots; wait for physics to settle "
            "and generate again"
        )
    return second


def apartment_oracle_view_from_snapshot(snapshot: RuntimeSceneSnapshot) -> SceneOracleView:
    """Join one validated runtime snapshot to the pinned apartment profile."""

    if snapshot.missing_entity_ids:
        raise OracleCompatibilityError(
            f"pinned apartment is missing profiled entity IDs: {list(snapshot.missing_entity_ids)}"
        )
    runtime_by_id = {entity.entity_id: entity for entity in snapshot.entities}
    if set(runtime_by_id) != set(APARTMENT_ENTITY_IDS):
        raise OracleCompatibilityError(
            "runtime snapshot entity IDs do not exactly match the apartment profile"
        )
    if snapshot.asset_count != APARTMENT_ASSET_COUNT:
        raise OracleCompatibilityError(
            "DimSim apartment asset inventory is incomplete or incompatible: "
            f"expected {APARTMENT_ASSET_COUNT}, got {snapshot.asset_count}"
        )
    if snapshot.navigation_grid.collision_source_count != APARTMENT_COLLISION_SOURCE_COUNT:
        raise OracleCompatibilityError(
            "DimSim apartment collision world is incomplete or incompatible: "
            f"expected {APARTMENT_COLLISION_SOURCE_COUNT} sources, got "
            f"{snapshot.navigation_grid.collision_source_count}"
        )
    spawn_delta = (
        (snapshot.agent_position.x - APARTMENT_CANONICAL_SPAWN[0]) ** 2
        + (snapshot.agent_position.z - APARTMENT_CANONICAL_SPAWN[1]) ** 2
    ) ** 0.5
    if spawn_delta > _SPAWN_TOLERANCE_M:
        raise OracleCompatibilityError(
            "DimSim apartment is not at the canonical reset spawn: "
            f"expected {APARTMENT_CANONICAL_SPAWN}, got "
            f"({snapshot.agent_position.x}, {snapshot.agent_position.z})"
        )
    if (
        abs(snapshot.agent_radius_m - APARTMENT_EMBODIMENT_RADIUS_M) > _EMBODIMENT_TOLERANCE_M
        or abs(snapshot.agent_half_height_m - APARTMENT_EMBODIMENT_HALF_HEIGHT_M)
        > _EMBODIMENT_TOLERANCE_M
    ):
        raise OracleCompatibilityError(
            "DimSim apartment uses an unsupported embodiment: "
            f"radius={snapshot.agent_radius_m}, "
            f"half_height={snapshot.agent_half_height_m}"
        )
    required_clearance = APARTMENT_EMBODIMENT_RADIUS_M + EMBODIMENT_CLEARANCE_M
    if snapshot.navigation_grid.clearance_radius_m + _EMBODIMENT_TOLERANCE_M < required_clearance:
        raise OracleCompatibilityError(
            "collision-world navigation was generated with insufficient clearance: "
            f"required {required_clearance}, got "
            f"{snapshot.navigation_grid.clearance_radius_m}"
        )

    entities = tuple(
        _entity_from_snapshot(profile, runtime_by_id[profile.entity_id])
        for profile in APARTMENT_ENTITY_PROFILES
    )
    navigation = _navigation_from_grid(snapshot.navigation_grid)
    reset_digest = hashlib.sha256(
        canonical_json(
            {
                "profile_revision": APARTMENT_PROFILE_REVISION,
                "navigation_grid": snapshot.navigation_grid.model_dump(mode="json"),
                "entities": [
                    {
                        "entity_id": entity.entity_id,
                        "current_state_id": entity.current_state_id,
                        "position": entity.position.model_dump(mode="json"),
                        "yaw_rad": entity.yaw_rad,
                        "bounds": entity.bounds.model_dump(mode="json"),
                    }
                    for entity in snapshot.entities
                ],
            }
        )
    ).hexdigest()
    return SceneOracleView(
        scene_id=APARTMENT_SCENE_ID,
        scene_revision=APARTMENT_SCENE_REVISION,
        reset_revision=f"runtime-snapshot-{reset_digest}",
        upstream_revision=DIMSIM_REPO_COMMIT,
        profile_revision=APARTMENT_PROFILE_REVISION,
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
            embodiment_id="unitree-go2-capsule",
            footprint_radius_m=APARTMENT_EMBODIMENT_RADIUS_M,
            canonical_spawn=Pose2(
                x_m=APARTMENT_CANONICAL_SPAWN[0],
                z_m=APARTMENT_CANONICAL_SPAWN[1],
                yaw_rad=0.0,
            ),
        ),
        navigation=navigation,
        entities=entities,
        regions=(
            Region(
                region_id=APARTMENT_REGION_ID,
                semantic_class="apartment",
                footprint=APARTMENT_NAVIGATION_POLYGON,
            ),
        ),
        provenance=(
            ProvenanceGroup(
                source_kind="authored",
                source_revision=APARTMENT_PROFILE_REVISION,
                field_paths=(
                    "entities.*.semantic_class",
                    "entities.*.aliases",
                    "entities.*.properties",
                    "entities.*.region_ids",
                    "regions.*",
                    "embodiment.canonical_spawn",
                ),
            ),
            ProvenanceGroup(
                source_kind="policy-derived",
                source_revision=DIMSIM_REPO_COMMIT,
                policy_version=_BOUNDS_POLICY_VERSION,
                field_paths=(
                    "entities.*.display_title",
                    "entities.*.position",
                    "entities.*.yaw_rad",
                    "entities.*.footprint",
                ),
            ),
            ProvenanceGroup(
                source_kind="policy-derived",
                source_revision=DIMSIM_REPO_COMMIT,
                policy_version=_NAVIGATION_POLICY_VERSION,
                field_paths=("navigation.*",),
            ),
        ),
    )


def _navigation_from_grid(grid: RuntimeNavigationGrid) -> NavigationGeometry:
    cell = grid.cell_size_m
    polygons = tuple(
        (
            (grid.min_x + run.start_col * cell, grid.min_z + run.row * cell),
            (grid.min_x + (run.end_col + 1) * cell, grid.min_z + run.row * cell),
            (
                grid.min_x + (run.end_col + 1) * cell,
                grid.min_z + (run.row + 1) * cell,
            ),
            (grid.min_x + run.start_col * cell, grid.min_z + (run.row + 1) * cell),
        )
        for run in grid.reachable_runs
    )
    spawn_col = int((APARTMENT_CANONICAL_SPAWN[0] - grid.min_x) // cell)
    spawn_row = int((APARTMENT_CANONICAL_SPAWN[1] - grid.min_z) // cell)
    if not any(
        run.row == spawn_row and run.start_col <= spawn_col <= run.end_col
        for run in grid.reachable_runs
    ):
        raise OracleCompatibilityError(
            "collision-world navigation does not contain the canonical spawn"
        )
    return NavigationGeometry(
        navigable=polygons,
        clearance_radius_m=grid.clearance_radius_m,
        collision_source_count=grid.collision_source_count,
    )


def _entity_from_snapshot(
    profile: ApartmentEntityProfile,
    runtime: RuntimeEntitySnapshot,
) -> Entity:
    properties: tuple[SemanticProperty, ...] = ()
    if profile.state_property is not None:
        state_values = dict(profile.state_values)
        undeclared = set(runtime.state_ids).difference(state_values)
        if undeclared:
            raise OracleCompatibilityError(
                f"entity {runtime.entity_id} exposes unsupported states: {sorted(undeclared)}"
            )
        try:
            value = state_values[runtime.current_state_id]
        except KeyError as error:
            raise OracleCompatibilityError(
                f"entity {runtime.entity_id} has unsupported current state "
                f"{runtime.current_state_id!r}"
            ) from error
        properties = (
            SemanticProperty(
                name=profile.state_property,
                value=value,
                authoritative=True,
            ),
        )
    bounds = runtime.bounds
    footprint = (
        (bounds.min_x, bounds.min_z),
        (bounds.max_x, bounds.min_z),
        (bounds.max_x, bounds.max_z),
        (bounds.min_x, bounds.max_z),
    )
    return Entity(
        entity_id=runtime.entity_id,
        semantic_class=profile.semantic_class,
        aliases=profile.aliases,
        display_title=runtime.display_title,
        position=(runtime.position.x, runtime.position.z),
        yaw_rad=runtime.yaw_rad,
        footprint=footprint,
        properties=properties,
        region_ids=(APARTMENT_REGION_ID,),
    )
