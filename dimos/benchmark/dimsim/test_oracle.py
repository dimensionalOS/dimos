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

import pytest

from dimos.benchmark.dimsim.apartment_profile import (
    APARTMENT_ENTITY_IDS,
    APARTMENT_ENTITY_PROFILES,
    APARTMENT_PROFILE_REVISION,
)
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture
from dimos.benchmark.dimsim.models import RuntimeSceneSnapshot
from dimos.benchmark.dimsim.oracle import (
    InMemorySceneOracleProvider,
    OracleCompatibilityError,
    SceneClientOracleProvider,
    apartment_oracle_view_from_snapshot,
    get_stable_scene_oracle_view,
)
from dimos.simulation.dimsim.revision import DIMSIM_REPO_COMMIT
from dimos.simulation.dimsim.scene_client import SceneClient


def _runtime_snapshot(
    *,
    missing_entity_ids: tuple[str, ...] = (),
    television_state: str = "state-mlr4bgf0-u7gv",
    spawn: tuple[float, float] = (2.0, 3.0),
) -> RuntimeSceneSnapshot:
    positions = {
        "bathtub": (4.52, -0.90, 2.0, 1.0),
        "television": (4.50, 4.88, 1.4, 0.3),
        "sofa": (4.38, 1.06, 2.8, 1.4),
        "dining-chair": (-0.5, 3.8, 0.5, 0.5),
        "work-chair": (-1.67, -1.54, 0.6, 0.6),
    }
    class_indexes: dict[str, int] = {}
    entities = []
    for profile in APARTMENT_ENTITY_PROFILES:
        index = class_indexes.get(profile.semantic_class, 0)
        class_indexes[profile.semantic_class] = index + 1
        x, z, width, depth = positions[profile.semantic_class]
        x += index * 0.7
        state_ids = tuple(state_id for state_id, _ in profile.state_values) or ("state-default",)
        current_state_id = (
            television_state if profile.semantic_class == "television" else state_ids[0]
        )
        entities.append(
            {
                "entity_id": profile.entity_id,
                "display_title": profile.aliases[0],
                "current_state_id": current_state_id,
                "state_ids": state_ids,
                "position": {"x": x, "z": z},
                "yaw_rad": 0.0,
                "bounds": {
                    "min_x": x - width / 2,
                    "max_x": x + width / 2,
                    "min_z": z - depth / 2,
                    "max_z": z + depth / 2,
                },
            }
        )
    return RuntimeSceneSnapshot.model_validate(
        {
            "snapshot_schema_version": "1.0",
            "asset_count": 123,
            "missing_entity_ids": missing_entity_ids,
            "agent_position": {"x": spawn[0], "z": spawn[1]},
            "agent_radius_m": 0.12,
            "agent_half_height_m": 0.25,
            "navigation_grid": {
                "min_x": -5.0,
                "min_z": -5.5,
                "cell_size_m": 0.5,
                "width": 21,
                "height": 22,
                "clearance_radius_m": 0.17,
                "collision_source_count": 167,
                "reachable_runs": tuple(
                    {"row": row, "start_col": 0, "end_col": 20} for row in range(22)
                ),
            },
            "entities": tuple(entities),
        }
    )


def test_in_memory_provider_returns_validated_view() -> None:
    view = apartment_oracle_fixture()

    assert InMemorySceneOracleProvider(view).get_scene_oracle_view() is view


def test_scene_client_provider_joins_snapshot_to_profile(mocker) -> None:
    client = mocker.create_autospec(SceneClient, instance=True)
    snapshot = _runtime_snapshot()
    client.get_scene_oracle_snapshot.return_value = snapshot.model_dump(mode="json")

    result = SceneClientOracleProvider(client).get_scene_oracle_view()

    television = next(entity for entity in result.entities if entity.semantic_class == "television")
    power = television.property("power")
    assert power is not None
    assert power.value == "OFF"
    assert result.upstream_revision == DIMSIM_REPO_COMMIT
    assert result.provenance[0].source_revision == APARTMENT_PROFILE_REVISION
    assert result.navigation.collision_source_count == 167
    assert len(result.navigation.navigable) == 22
    assert result.navigation.blocked == ()
    client.get_scene_oracle_snapshot.assert_called_once()
    assert client.get_scene_oracle_snapshot.call_args.args == (APARTMENT_ENTITY_IDS,)


def test_stable_oracle_view_requires_two_identical_snapshots(mocker) -> None:
    view = apartment_oracle_fixture()
    provider = mocker.Mock()
    provider.get_scene_oracle_view.side_effect = [view, view]
    pause = mocker.Mock()

    result = get_stable_scene_oracle_view(
        provider,
        stability_delay_s=0.25,
        pause=pause,
    )

    assert result is view
    assert provider.get_scene_oracle_view.call_count == 2
    pause.assert_called_once_with(0.25)


def test_stable_oracle_view_rejects_physics_drift(mocker) -> None:
    first = apartment_oracle_fixture()
    second = first.model_copy(update={"reset_revision": "runtime-snapshot-settled"})
    provider = mocker.Mock()
    provider.get_scene_oracle_view.side_effect = [first, second]

    with pytest.raises(OracleCompatibilityError, match="changed between oracle snapshots"):
        get_stable_scene_oracle_view(provider, stability_delay_s=0, pause=mocker.Mock())


def test_scene_client_private_command_requests_exact_ids(mocker) -> None:
    client = SceneClient()
    execute = mocker.patch.object(
        client,
        "exec",
        return_value={"snapshot_schema_version": "1.0"},
    )

    result = client.get_scene_oracle_snapshot(
        ("asset-a", "asset-b"),
        snapshot_schema_version="1.0",
        navigation_bounds=(-5.0, -5.5, 5.5, 5.5),
        navigation_resolution_m=0.1,
        embodiment_clearance_m=0.05,
        ground_tolerance_m=0.08,
    )

    assert result == {"snapshot_schema_version": "1.0"}
    code = execute.call_args.args[0]
    assert '["asset-a", "asset-b"]' in code
    assert 'requestedSnapshotSchemaVersion = "1.0"' in code
    assert "assets.find" in code
    assert "rapierWorld.intersectionsWithShape" in code
    assert "reachableRuns" in code
    assert "getSceneOracleView" not in code


def test_profile_rejects_missing_runtime_entity() -> None:
    snapshot = _runtime_snapshot(missing_entity_ids=(APARTMENT_ENTITY_IDS[0],))

    with pytest.raises(OracleCompatibilityError, match="missing profiled entity IDs"):
        apartment_oracle_view_from_snapshot(snapshot)


def test_scene_client_provider_wraps_unsupported_snapshot_schema(mocker) -> None:
    client = mocker.create_autospec(SceneClient, instance=True)
    payload = _runtime_snapshot().model_dump(mode="json")
    payload["snapshot_schema_version"] = "2.0"
    client.get_scene_oracle_snapshot.return_value = payload

    with pytest.raises(OracleCompatibilityError, match="unsupported or malformed"):
        SceneClientOracleProvider(client).get_scene_oracle_view()


def test_profile_rejects_unsupported_television_state() -> None:
    snapshot = _runtime_snapshot(television_state="state-unknown")

    with pytest.raises(OracleCompatibilityError, match="unsupported current state"):
        apartment_oracle_view_from_snapshot(snapshot)


def test_profile_rejects_noncanonical_spawn() -> None:
    snapshot = _runtime_snapshot(spawn=(0.0, 0.0))

    with pytest.raises(OracleCompatibilityError, match="not at the canonical reset spawn"):
        apartment_oracle_view_from_snapshot(snapshot)


def test_runtime_snapshot_rejects_zero_area_bounds() -> None:
    payload = _runtime_snapshot().model_dump(mode="python")
    payload["entities"][0]["bounds"]["max_x"] = payload["entities"][0]["bounds"]["min_x"]

    with pytest.raises(ValueError, match="positive XZ area"):
        RuntimeSceneSnapshot.model_validate(payload)


def test_runtime_snapshot_rejects_navigation_run_outside_grid() -> None:
    payload = _runtime_snapshot().model_dump(mode="python")
    payload["navigation_grid"]["reachable_runs"][0]["end_col"] = 21

    with pytest.raises(ValueError, match="outside declared dimensions"):
        RuntimeSceneSnapshot.model_validate(payload)


def test_profile_rejects_navigation_without_canonical_spawn() -> None:
    payload = _runtime_snapshot().model_dump(mode="python")
    payload["navigation_grid"]["reachable_runs"] = ({"row": 0, "start_col": 0, "end_col": 0},)
    snapshot = RuntimeSceneSnapshot.model_validate(payload)

    with pytest.raises(OracleCompatibilityError, match="canonical spawn"):
        apartment_oracle_view_from_snapshot(snapshot)


def test_profile_rejects_navigation_with_insufficient_clearance() -> None:
    payload = _runtime_snapshot().model_dump(mode="python")
    payload["navigation_grid"]["clearance_radius_m"] = 0.12
    snapshot = RuntimeSceneSnapshot.model_validate(payload)

    with pytest.raises(OracleCompatibilityError, match="insufficient clearance"):
        apartment_oracle_view_from_snapshot(snapshot)


def test_profile_rejects_incomplete_collision_world() -> None:
    payload = _runtime_snapshot().model_dump(mode="python")
    payload["navigation_grid"]["collision_source_count"] = 166
    snapshot = RuntimeSceneSnapshot.model_validate(payload)

    with pytest.raises(OracleCompatibilityError, match="collision world is incomplete"):
        apartment_oracle_view_from_snapshot(snapshot)
