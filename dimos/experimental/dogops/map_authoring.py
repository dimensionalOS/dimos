from __future__ import annotations

import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import Field, ValidationError, field_validator, model_validator

from dimos.experimental.dogops.models import DogOpsModel
from dimos.experimental.dogops.route_actions import EditableRouteAction


AUTHORING_FILENAME = "map_authoring.json"
AUTHORING_SCHEMA_VERSION = 1


class EditableMapPoint(DogOpsModel):
    x: float
    y: float
    theta_deg: float | None = None
    source: Literal[
        "site_config",
        "dashboard_edit",
        "observation",
        "live_topic",
        "qr_cargo_event",
    ] = "dashboard_edit"

    @field_validator("x", "y", "theta_deg")
    @classmethod
    def finite_coordinate(cls, value: float | None) -> float | None:
        if value is None:
            return None
        result = float(value)
        if not math.isfinite(result):
            raise ValueError("coordinate must be finite")
        return result


class EditableMapEntity(DogOpsModel):
    id: str
    kind: Literal["zone", "asset", "package", "checkpoint"]
    label: str
    pose: EditableMapPoint
    tag_id: int | None = None
    zone_id: str | None = None
    source_id: str | None = None

    @field_validator("id", "label")
    @classmethod
    def required_text(cls, value: str) -> str:
        result = str(value).strip()
        if not result:
            raise ValueError("value must not be empty")
        return result


class EditableNoGoShape(DogOpsModel):
    id: str
    label: str
    shape: Literal["rectangle", "polygon"] = "rectangle"
    points: list[EditableMapPoint]
    enabled: bool = True
    dimos_constraint_status: Literal["not_supported", "pending", "published", "failed"] = (
        "not_supported"
    )

    @field_validator("id", "label")
    @classmethod
    def required_text(cls, value: str) -> str:
        result = str(value).strip()
        if not result:
            raise ValueError("value must not be empty")
        return result

    @model_validator(mode="after")
    def validate_points(self) -> "EditableNoGoShape":
        minimum = 2 if self.shape == "rectangle" else 3
        if len(self.points) < minimum:
            raise ValueError(f"{self.shape} requires at least {minimum} points")
        return self


class EditableRouteWaypoint(DogOpsModel):
    id: str
    label: str
    pose: EditableMapPoint
    target_id: str | None = None
    required: bool = True
    actions: list[EditableRouteAction] = Field(default_factory=list)

    @field_validator("id", "label")
    @classmethod
    def required_text(cls, value: str) -> str:
        result = str(value).strip()
        if not result:
            raise ValueError("value must not be empty")
        return result


class EditableRoute(DogOpsModel):
    id: str
    label: str
    waypoints: list[EditableRouteWaypoint] = Field(default_factory=list)
    mission_id: str | None = None

    @field_validator("id", "label")
    @classmethod
    def required_text(cls, value: str) -> str:
        result = str(value).strip()
        if not result:
            raise ValueError("value must not be empty")
        return result

    @model_validator(mode="after")
    def validate_waypoint_ids(self) -> "EditableRoute":
        _require_unique([waypoint.id for waypoint in self.waypoints], "route waypoint id")
        return self


class EditableIncidentLocation(DogOpsModel):
    incident_id: str
    entity_id: str | None = None
    pose: EditableMapPoint
    evidence_observation_ids: list[str] = Field(default_factory=list)

    @field_validator("incident_id")
    @classmethod
    def required_text(cls, value: str) -> str:
        result = str(value).strip()
        if not result:
            raise ValueError("incident_id must not be empty")
        return result


class EditableTagBinding(DogOpsModel):
    tag_id: int
    entity_id: str
    label: str
    binding_kind: Literal["zone", "asset", "package", "checkpoint"]

    @field_validator("entity_id", "label")
    @classmethod
    def required_text(cls, value: str) -> str:
        result = str(value).strip()
        if not result:
            raise ValueError("value must not be empty")
        return result


class MapAuthoringState(DogOpsModel):
    schema_version: int = AUTHORING_SCHEMA_VERSION
    site_id: str = ""
    frame: str = "world"
    updated_at: float = Field(default_factory=time.time)
    home: EditableMapPoint | None = None
    selected_route_id: str | None = None
    entities: list[EditableMapEntity] = Field(default_factory=list)
    no_go_shapes: list[EditableNoGoShape] = Field(default_factory=list)
    routes: list[EditableRoute] = Field(default_factory=list)
    incident_locations: list[EditableIncidentLocation] = Field(default_factory=list)
    tag_bindings: list[EditableTagBinding] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_unique_keys(self) -> "MapAuthoringState":
        _require_unique([entity.id for entity in self.entities], "entity id")
        _require_unique([shape.id for shape in self.no_go_shapes], "no-go shape id")
        _require_unique([route.id for route in self.routes], "route id")
        _require_unique(
            [location.incident_id for location in self.incident_locations],
            "incident id",
        )
        _require_unique([binding.tag_id for binding in self.tag_bindings], "tag id")
        if self.selected_route_id and self.selected_route_id not in {
            route.id for route in self.routes
        }:
            raise ValueError(f"unknown selected route id: {self.selected_route_id}")
        return self

    def touch(self) -> "MapAuthoringState":
        self.updated_at = time.time()
        return self


def authoring_path(run_dir: str | Path) -> Path:
    return Path(run_dir) / AUTHORING_FILENAME


def default_authoring(site_id: str = "") -> MapAuthoringState:
    return MapAuthoringState(site_id=site_id)


def load_map_authoring(run_dir: str | Path, *, site_id: str = "") -> MapAuthoringState:
    path = authoring_path(run_dir)
    if not path.exists():
        return default_authoring(site_id=site_id)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid map authoring JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"map authoring file must contain an object: {path}")
    if site_id and not payload.get("site_id"):
        payload["site_id"] = site_id
    return MapAuthoringState.model_validate(payload)


def save_map_authoring(run_dir: str | Path, authoring: MapAuthoringState) -> Path:
    path = authoring_path(run_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    authoring.touch()
    raw = json.dumps(authoring.model_dump(mode="json"), indent=2, sort_keys=True)
    tmp_path = path.with_name(
        f"{path.name}.{os.getpid()}.{threading.get_ident()}.{time.time_ns()}.tmp"
    )
    tmp_path.write_text(raw + "\n", encoding="utf-8")
    tmp_path.replace(path)
    return path


def replace_entity(
    authoring: MapAuthoringState,
    entity: EditableMapEntity,
) -> MapAuthoringState:
    authoring.entities = [
        existing for existing in authoring.entities if existing.id != entity.id
    ]
    authoring.entities.append(entity)
    return _validated(authoring)


def delete_entity(authoring: MapAuthoringState, entity_id: str) -> MapAuthoringState:
    authoring.entities = [entity for entity in authoring.entities if entity.id != entity_id]
    return _validated(authoring)


def replace_no_go_shape(
    authoring: MapAuthoringState,
    shape: EditableNoGoShape,
) -> MapAuthoringState:
    authoring.no_go_shapes = [
        existing for existing in authoring.no_go_shapes if existing.id != shape.id
    ]
    authoring.no_go_shapes.append(shape)
    return _validated(authoring)


def delete_no_go_shape(authoring: MapAuthoringState, shape_id: str) -> MapAuthoringState:
    authoring.no_go_shapes = [
        shape for shape in authoring.no_go_shapes if shape.id != shape_id
    ]
    return _validated(authoring)


def replace_route(authoring: MapAuthoringState, route: EditableRoute) -> MapAuthoringState:
    authoring.routes = [existing for existing in authoring.routes if existing.id != route.id]
    authoring.routes.append(route)
    if authoring.selected_route_id is None:
        authoring.selected_route_id = route.id
    return _validated(authoring)


def delete_route(authoring: MapAuthoringState, route_id: str) -> MapAuthoringState:
    authoring.routes = [route for route in authoring.routes if route.id != route_id]
    if authoring.selected_route_id == route_id:
        authoring.selected_route_id = authoring.routes[0].id if authoring.routes else None
    return _validated(authoring)


def select_route(authoring: MapAuthoringState, route_id: str | None) -> MapAuthoringState:
    authoring.selected_route_id = route_id
    return _validated(authoring)


def replace_incident_location(
    authoring: MapAuthoringState,
    location: EditableIncidentLocation,
) -> MapAuthoringState:
    authoring.incident_locations = [
        existing
        for existing in authoring.incident_locations
        if existing.incident_id != location.incident_id
    ]
    authoring.incident_locations.append(location)
    return _validated(authoring)


def replace_tag_binding(
    authoring: MapAuthoringState,
    binding: EditableTagBinding,
) -> MapAuthoringState:
    authoring.tag_bindings = [
        existing for existing in authoring.tag_bindings if existing.tag_id != binding.tag_id
    ]
    authoring.tag_bindings.append(binding)
    return _validated(authoring)


def delete_tag_binding(authoring: MapAuthoringState, tag_id: int) -> MapAuthoringState:
    authoring.tag_bindings = [
        binding for binding in authoring.tag_bindings if binding.tag_id != tag_id
    ]
    return _validated(authoring)


def export_authoring_yaml(
    run_dir: str | Path,
    authoring: MapAuthoringState,
) -> dict[str, str]:
    export_dir = Path(run_dir) / "exports"
    export_dir.mkdir(parents=True, exist_ok=True)
    site_path = export_dir / "site_authoring.yaml"
    mission_path = export_dir / "mission_authoring.yaml"
    payload = authoring.model_dump(mode="json")
    site_payload = {
        "site_id": authoring.site_id,
        "frame": authoring.frame,
        "home": payload["home"],
        "entities": payload["entities"],
        "no_go_shapes": payload["no_go_shapes"],
        "tag_bindings": payload["tag_bindings"],
    }
    mission_payload = {
        "site_id": authoring.site_id,
        "selected_route_id": authoring.selected_route_id,
        "routes": payload["routes"],
        "mission_steps": _mission_steps_for_selected_route(authoring),
        "incident_locations": payload["incident_locations"],
    }
    site_path.write_text(yaml.safe_dump(site_payload, sort_keys=False), encoding="utf-8")
    mission_path.write_text(
        yaml.safe_dump(mission_payload, sort_keys=False),
        encoding="utf-8",
    )
    return {"site": str(site_path), "mission": str(mission_path)}


def publish_no_go_constraints(
    authoring: MapAuthoringState,
    *,
    command: str | None = None,
    timeout_s: float = 5.0,
) -> MapAuthoringState:
    publisher = command or os.environ.get("DOGOPS_NO_GO_PUBLISH_COMMAND")
    enabled_shapes = [shape for shape in authoring.no_go_shapes if shape.enabled]
    if not enabled_shapes:
        return authoring
    if not publisher:
        for shape in enabled_shapes:
            shape.dimos_constraint_status = "not_supported"
        return _validated(authoring)

    payload = json.dumps(
        {
            "site_id": authoring.site_id,
            "frame": authoring.frame,
            "no_go_shapes": [
                shape.model_dump(mode="json") for shape in enabled_shapes
            ],
        },
        separators=(",", ":"),
    )
    try:
        result = subprocess.run(
            publisher,
            input=payload,
            capture_output=True,
            check=False,
            shell=True,
            text=True,
            timeout=timeout_s,
        )
    except Exception:
        for shape in enabled_shapes:
            shape.dimos_constraint_status = "failed"
        return _validated(authoring)

    status = "published" if result.returncode == 0 else "failed"
    for shape in enabled_shapes:
        shape.dimos_constraint_status = status
    return _validated(authoring)


def validation_error_message(exc: ValidationError | ValueError) -> str:
    if isinstance(exc, ValidationError):
        errors = exc.errors()
        if errors:
            return str(errors[0].get("msg") or errors[0])
    return str(exc)


def _mission_steps_for_selected_route(authoring: MapAuthoringState) -> list[dict[str, Any]]:
    route = next(
        (item for item in authoring.routes if item.id == authoring.selected_route_id),
        authoring.routes[0] if authoring.routes else None,
    )
    if route is None:
        return []
    steps: list[dict[str, Any]] = []
    for index, waypoint in enumerate(route.waypoints, 1):
        steps.append(
            {
                "id": waypoint.id,
                "action": "goto",
                "target_id": waypoint.target_id or waypoint.id,
                "required": waypoint.required,
                "pose_hint": waypoint.pose.model_dump(mode="json"),
                "sequence": index,
            }
        )
    return steps


def _validated(authoring: MapAuthoringState) -> MapAuthoringState:
    return MapAuthoringState.model_validate(authoring.model_dump(mode="json"))


def _require_unique(values: list[Any], label: str) -> None:
    seen: set[Any] = set()
    for value in values:
        if value in seen:
            raise ValueError(f"duplicate {label}: {value}")
        seen.add(value)
