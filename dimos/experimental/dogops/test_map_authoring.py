from __future__ import annotations

import json

import pytest

from dimos.experimental.dogops.dashboard_static import build_map_data
from dimos.experimental.dogops.map_authoring import (
    EditableMapEntity,
    EditableMapPoint,
    EditableNoGoShape,
    EditableRoute,
    EditableRouteWaypoint,
    EditableTagBinding,
    MapAuthoringState,
    export_authoring_yaml,
    load_map_authoring,
    publish_no_go_constraints,
    replace_entity,
    save_map_authoring,
)
from dimos.experimental.dogops.mission_engine import run_offline_simulation


def test_map_authoring_round_trips_empty_state(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    authoring = MapAuthoringState(site_id="dogops_demo_site")

    path = save_map_authoring(run_dir, authoring)
    loaded = load_map_authoring(run_dir)

    assert path.name == "map_authoring.json"
    assert loaded.site_id == "dogops_demo_site"
    assert loaded.entities == []
    assert loaded.no_go_shapes == []


def test_map_authoring_rejects_duplicate_ids() -> None:
    point = EditableMapPoint(x=1.0, y=2.0)
    entity = EditableMapEntity(id="A", kind="checkpoint", label="A", pose=point)

    with pytest.raises(ValueError, match="duplicate entity id"):
        MapAuthoringState(entities=[entity, entity])


def test_map_authoring_rejects_duplicate_tag_bindings() -> None:
    binding = EditableTagBinding(
        tag_id=42,
        entity_id="CHECKPOINT_A",
        label="Checkpoint A",
        binding_kind="checkpoint",
    )

    with pytest.raises(ValueError, match="duplicate tag id"):
        MapAuthoringState(tag_bindings=[binding, binding])


def test_build_map_data_applies_authoring_overrides(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    state = run_offline_simulation(out=run_dir)
    report = json.loads((run_dir / "report.json").read_text(encoding="utf-8"))
    authoring = MapAuthoringState(
        site_id="dogops_demo_site",
        home=EditableMapPoint(x=-1.5, y=2.5),
        entities=[
            EditableMapEntity(
                id="COOLING_1",
                kind="asset",
                label="Edited Cooling",
                pose=EditableMapPoint(x=9.0, y=8.0),
                tag_id=141,
            ),
            EditableMapEntity(
                id="CHECKPOINT_X",
                kind="checkpoint",
                label="Checkpoint X",
                pose=EditableMapPoint(x=4.0, y=5.0),
            ),
        ],
        no_go_shapes=[
            EditableNoGoShape(
                id="NO_GO_EDIT",
                label="Edited No-Go",
                shape="rectangle",
                points=[
                    EditableMapPoint(x=7.0, y=7.0),
                    EditableMapPoint(x=8.0, y=8.0),
                ],
            )
        ],
        routes=[
            EditableRoute(
                id="ROUTE_EDIT",
                label="Edited Route",
                waypoints=[
                    EditableRouteWaypoint(
                        id="WP1",
                        label="Waypoint 1",
                        pose=EditableMapPoint(x=4.0, y=5.0),
                        target_id="CHECKPOINT_X",
                    )
                ],
            )
        ],
    )

    map_data = build_map_data(
        state.model_dump(mode="json"),
        report,
        authoring=authoring.model_dump(mode="json"),
    )

    home = next(zone for zone in map_data["zones"] if zone["id"] == "HOME")
    cooling = next(asset for asset in map_data["assets"] if asset["id"] == "COOLING_1")
    checkpoint = next(zone for zone in map_data["zones"] if zone["id"] == "CHECKPOINT_X")

    assert home["x"] == -1.5
    assert home["y"] == 2.5
    assert cooling["display_name"] == "Edited Cooling"
    assert cooling["tag_id"] == 141
    assert cooling["x"] == 9.0
    assert checkpoint["source"] == "dashboard_edit"
    assert map_data["route"][0]["target_id"] == "CHECKPOINT_X"
    assert map_data["authoring"]["selected_route_id"] is None
    assert map_data["no_go_shapes"][0]["dimos_constraint_status"] == "not_supported"
    assert map_data["authoring"]["entities"] == 2


def test_incident_location_uses_authored_pose(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    state = run_offline_simulation(out=run_dir)
    report = json.loads((run_dir / "report.json").read_text(encoding="utf-8"))
    authoring = {
        "incident_locations": [
            {
                "incident_id": "INC-001",
                "pose": {"x": 6.0, "y": 6.5, "source": "dashboard_edit"},
                "evidence_observation_ids": ["OBS-003"],
            }
        ]
    }

    map_data = build_map_data(state.model_dump(mode="json"), report, authoring=authoring)
    incident = next(item for item in map_data["incidents"] if item["id"] == "INC-001")

    assert incident["x"] == 6.0
    assert incident["y"] == 6.5
    assert incident["source"] == "dashboard_edit"


def test_authoring_export_writes_run_local_yaml(tmp_path) -> None:
    authoring = replace_entity(
        MapAuthoringState(site_id="dogops_demo_site"),
        EditableMapEntity(
            id="CHECKPOINT_A",
            kind="checkpoint",
            label="Checkpoint A",
            pose=EditableMapPoint(x=1.0, y=2.0),
        ),
    )

    exports = export_authoring_yaml(tmp_path, authoring)

    assert exports["site"].endswith("exports/site_authoring.yaml")
    assert exports["mission"].endswith("exports/mission_authoring.yaml")
    assert "CHECKPOINT_A" in (tmp_path / "exports" / "site_authoring.yaml").read_text(
        encoding="utf-8"
    )


def test_authoring_export_includes_selected_route_mission_steps(tmp_path) -> None:
    authoring = MapAuthoringState(
        site_id="dogops_demo_site",
        selected_route_id="ROUTE_A",
        routes=[
            EditableRoute(
                id="ROUTE_A",
                label="Route A",
                waypoints=[
                    EditableRouteWaypoint(
                        id="WP_A",
                        label="Waypoint A",
                        target_id="CHECKPOINT_A",
                        pose=EditableMapPoint(x=1.0, y=2.0),
                    )
                ],
            )
        ],
    )

    exports = export_authoring_yaml(tmp_path, authoring)
    mission_yaml = (tmp_path / "exports" / "mission_authoring.yaml").read_text(
        encoding="utf-8"
    )

    assert exports["mission"].endswith("mission_authoring.yaml")
    assert "selected_route_id: ROUTE_A" in mission_yaml
    assert "target_id: CHECKPOINT_A" in mission_yaml


def test_no_go_publish_marks_enabled_shapes_published_and_excludes_disabled(
    monkeypatch,
) -> None:
    calls: list[dict[str, object]] = []

    def fake_run(*_: object, **kwargs: object) -> object:
        calls.append(json.loads(str(kwargs["input"])))

        class Result:
            returncode = 0

        return Result()

    monkeypatch.setattr("subprocess.run", fake_run)
    authoring = MapAuthoringState(
        site_id="dogops_demo_site",
        no_go_shapes=[
            EditableNoGoShape(
                id="NO_GO_ENABLED",
                label="Enabled",
                points=[EditableMapPoint(x=0.0, y=0.0), EditableMapPoint(x=1.0, y=1.0)],
                enabled=True,
            ),
            EditableNoGoShape(
                id="NO_GO_DISABLED",
                label="Disabled",
                points=[EditableMapPoint(x=2.0, y=2.0), EditableMapPoint(x=3.0, y=3.0)],
                enabled=False,
            ),
        ],
    )

    published = publish_no_go_constraints(authoring, command="dogops-publisher")

    assert calls[0]["site_id"] == "dogops_demo_site"
    assert [shape["id"] for shape in calls[0]["no_go_shapes"]] == ["NO_GO_ENABLED"]  # type: ignore[index]
    assert published.no_go_shapes[0].dimos_constraint_status == "published"
    assert published.no_go_shapes[1].dimos_constraint_status == "not_supported"


def test_no_go_publish_marks_enabled_shapes_failed_on_command_failure() -> None:
    authoring = MapAuthoringState(
        no_go_shapes=[
            EditableNoGoShape(
                id="NO_GO_FAIL",
                label="Fail",
                points=[EditableMapPoint(x=0.0, y=0.0), EditableMapPoint(x=1.0, y=1.0)],
            )
        ]
    )

    failed = publish_no_go_constraints(authoring, command="false")

    assert failed.no_go_shapes[0].dimos_constraint_status == "failed"
