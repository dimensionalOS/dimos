from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


class DogOpsModel(BaseModel):
    model_config = ConfigDict(extra="ignore", use_enum_values=True)


class EntityKind(str, Enum):
    zone = "zone"
    asset = "asset"
    package = "package"
    dock = "dock"
    portal = "portal"


class ZoneKind(str, Enum):
    home = "home"
    inbound_dock = "inbound_dock"
    qa_hold = "qa_hold"
    rack_row = "rack_row"
    aisle = "aisle"
    no_go = "no_go"
    dock = "dock"
    portal = "portal"


class AssetKind(str, Enum):
    cooling_clearance = "cooling_clearance"
    rack_status = "rack_status"
    aisle_clearance = "aisle_clearance"
    safety_station = "safety_station"
    temperature_station = "temperature_station"


class PackageState(str, Enum):
    expected = "expected"
    found_ok = "found_ok"
    wrong_zone = "wrong_zone"
    missing = "missing"
    damaged = "damaged"
    blocking_asset = "blocking_asset"
    unknown = "unknown"


class IncidentType(str, Enum):
    blocked_cooling = "blocked_cooling"
    wrong_zone = "wrong_zone"
    missing_package = "missing_package"
    damaged_package = "damaged_package"
    blocked_aisle = "blocked_aisle"
    no_go_breach = "no_go_breach"
    high_temperature = "high_temperature"
    unknown = "unknown"


class Severity(str, Enum):
    P1 = "P1"
    P2 = "P2"
    P3 = "P3"
    INFO = "INFO"


class IncidentState(str, Enum):
    open = "open"
    acked = "acked"
    ready_to_verify = "ready_to_verify"
    resolved = "resolved"
    unresolved = "unresolved"
    false_positive = "false_positive"


class WorkOrderState(str, Enum):
    open = "open"
    assigned = "assigned"
    ready_to_verify = "ready_to_verify"
    verified_closed = "verified_closed"
    blocked = "blocked"
    cancelled = "cancelled"


class MissionState(str, Enum):
    init = "init"
    running = "running"
    waiting_for_human = "waiting_for_human"
    verifying = "verifying"
    done = "done"
    failed = "failed"
    stopped = "stopped"


class NavAction(str, Enum):
    goto = "goto"
    scan = "scan"
    search_tag = "search_tag"
    rotate = "rotate"
    step_back = "step_back"
    guided = "guided"
    dock_align = "dock_align"
    portal_entry = "portal_entry"


class Pose2D(DogOpsModel):
    x: float | None = None
    y: float | None = None
    theta_deg: float | None = None
    frame: str = "world"
    source: str = "unknown"


class SiteEntity(DogOpsModel):
    id: str
    kind: EntityKind
    tag_id: int | None = None
    display_name: str
    zone_id: str | None = None
    expected_state: dict[str, Any] = Field(default_factory=dict)
    severity_if_failed: Severity = Severity.P3
    notes: str = ""


class Zone(SiteEntity):
    kind: Literal[EntityKind.zone] = EntityKind.zone
    zone_kind: ZoneKind
    pose_hint: Pose2D | None = None
    radius_m: float = 0.8
    no_go: bool = False


class Asset(SiteEntity):
    kind: Literal[EntityKind.asset] = EntityKind.asset
    asset_kind: AssetKind
    expected_clear: bool | None = None
    expected_status: str | None = None
    blocking_package_ids: list[str] = Field(default_factory=list)


class Package(SiteEntity):
    kind: Literal[EntityKind.package] = EntityKind.package
    expected_zone_id: str
    expected_condition: str = "ok"


class SpecialEntity(SiteEntity):
    kind: EntityKind


class SiteConfig(DogOpsModel):
    site_id: str
    site_name: str = ""
    tag_family: str = "tag36h11"
    marker_length_m: float
    zones: list[Zone] = Field(default_factory=list)
    assets: list[Asset] = Field(default_factory=list)
    packages: list[Package] = Field(default_factory=list)
    special_entities: dict[str, SpecialEntity] = Field(default_factory=dict)

    def package_by_id(self) -> dict[str, Package]:
        return {pkg.id: pkg for pkg in self.packages}

    def asset_by_id(self) -> dict[str, Asset]:
        return {asset.id: asset for asset in self.assets}

    def zone_by_id(self) -> dict[str, Zone]:
        return {zone.id: zone for zone in self.zones}

    def entity_for_tag(self) -> dict[int, SiteEntity]:
        entities: list[SiteEntity] = [*self.zones, *self.assets, *self.packages]
        entities.extend(self.special_entities.values())
        return {entity.tag_id: entity for entity in entities if entity.tag_id is not None}


class ManifestItem(DogOpsModel):
    package_id: str
    expected_zone_id: str
    expected_condition: str = "ok"


class Manifest(DogOpsModel):
    manifest_id: str
    items: list[ManifestItem]

    def item_by_package_id(self) -> dict[str, ManifestItem]:
        return {item.package_id: item for item in self.items}


class PolicyRule(DogOpsModel):
    id: str
    severity: Severity = Severity.P3
    incident_type: IncidentType = IncidentType.unknown
    description: str
    condition: dict[str, Any] = Field(default_factory=dict)
    recommended_action: str = ""


class PolicyConfig(DogOpsModel):
    policy_id: str
    rules: list[PolicyRule] = Field(default_factory=list)

    def rule_for_type(self, incident_type: IncidentType | str) -> PolicyRule | None:
        type_value = incident_type.value if isinstance(incident_type, IncidentType) else incident_type
        for rule in self.rules:
            if rule.incident_type == type_value:
                return rule
        return None


class MissionStep(DogOpsModel):
    id: str
    action: str
    target_id: str
    timeout_s: float = 30.0
    required: bool = True


class SimulationObservation(DogOpsModel):
    zone_id: str
    visible_tag_ids: list[int] = Field(default_factory=list)
    facts: dict[str, bool | str | int | float] = Field(default_factory=dict)


class NavSimulationEvent(DogOpsModel):
    target_id: str
    action: NavAction = NavAction.goto
    success: bool = True
    elapsed_s: float = 0.0
    retries: int = 0
    guided: bool = False
    error_m: float | None = None
    note: str = ""


class NavSimulation(DogOpsModel):
    guided: bool = False
    events: list[NavSimulationEvent] = Field(default_factory=list)


class MissionConfig(DogOpsModel):
    mission_id: str
    display_name: str
    steps: list[MissionStep]
    verify_after_human: bool = True
    simulation_observations: dict[str, SimulationObservation] = Field(default_factory=dict)
    nav_simulation: NavSimulation = Field(default_factory=NavSimulation)


class Observation(DogOpsModel):
    id: str
    ts: float
    run_id: str
    entity_id: str | None = None
    tag_id: int | None = None
    zone_id: str | None = None
    pose: Pose2D | None = None
    image_path: str | None = None
    facts: dict[str, bool | str | int | float] = Field(default_factory=dict)
    confidence: float = 1.0
    source: str = "simulation"


class Incident(DogOpsModel):
    id: str
    run_id: str
    ts_open: float
    ts_closed: float | None = None
    severity: Severity
    type: IncidentType
    entity_id: str
    related_package_id: str | None = None
    state: IncidentState
    title: str
    evidence_observation_ids: list[str] = Field(default_factory=list)
    recommended_action: str = ""


class WorkOrder(DogOpsModel):
    id: str
    incident_id: str
    requested_action: str
    assignee: str = "human_operator"
    state: WorkOrderState
    verification_observation_ids: list[str] = Field(default_factory=list)


class NavEvent(DogOpsModel):
    id: str
    run_id: str
    ts: float
    action: NavAction
    target_id: str | None = None
    success: bool = True
    elapsed_s: float = 0.0
    retries: int = 0
    guided: bool = False
    error_m: float | None = None
    note: str = ""


class NavSummary(DogOpsModel):
    run_id: str
    waypoints_total: int
    waypoints_reached: int
    waypoints_failed: int
    success_rate: float
    route_targets: int = 0
    unique_targets_reached: int = 0
    route_coverage: float = 0.0
    retries_total: int
    guided_interventions: int
    guided_fallback_used: bool = False
    tag_reacquisition_attempts: int
    tag_reacquisition_successes: int
    tag_reacquisition_rate: float = 0.0
    mean_elapsed_s: float
    worst_target_id: str | None = None
    safety_stops: int = 0
    notes: list[str] = Field(default_factory=list)


class MissionRun(DogOpsModel):
    id: str
    mission_id: str
    started_at: float
    ended_at: float | None = None
    state: MissionState
    current_step_id: str | None = None
    summary: str = ""


class PackageStatus(DogOpsModel):
    package_id: str
    expected_zone_id: str
    observed_zone_id: str | None = None
    state: PackageState = PackageState.expected
    blocks_asset_id: str | None = None


class DogOpsState(DogOpsModel):
    run: MissionRun
    site: SiteConfig
    manifest: Manifest
    policy: PolicyConfig
    mission: MissionConfig
    package_statuses: dict[str, PackageStatus] = Field(default_factory=dict)
    observations: list[Observation] = Field(default_factory=list)
    incidents: list[Incident] = Field(default_factory=list)
    work_orders: list[WorkOrder] = Field(default_factory=list)
    nav_events: list[NavEvent] = Field(default_factory=list)
    nav_summary: NavSummary | None = None
    what_changed: list[str] = Field(default_factory=list)


class DogOpsConfig(DogOpsModel):
    site: SiteConfig
    manifest: Manifest
    policy: PolicyConfig
    mission: MissionConfig
    paths: dict[str, Path]
