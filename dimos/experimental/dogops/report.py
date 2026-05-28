from __future__ import annotations

from dimos.experimental.dogops.models import (
    DogOpsState,
    IncidentState,
    IncidentType,
    NavAction,
    Observation,
    PackageState,
    SiteEntity,
    WorkOrderState,
)


def _value(value: object) -> object:
    return getattr(value, "value", value)


def build_checkpoint_verifications(state: DogOpsState) -> list[dict[str, object]]:
    entities_by_id: dict[str, SiteEntity] = {
        **state.site.zone_by_id(),
        **state.site.asset_by_id(),
        **state.site.package_by_id(),
    }
    entities_by_id.update(state.site.special_entities)
    checkpoints: list[dict[str, object]] = []
    seen_targets: set[str] = set()

    for event in state.nav_events:
        if event.action != NavAction.goto or not event.target_id:
            continue
        target_id = event.target_id
        if target_id in seen_targets:
            continue
        seen_targets.add(target_id)
        entity = entities_by_id.get(target_id)
        expected_tag_id = entity.tag_id if entity is not None else None
        matching_observation = _first_observation_with_tag(state.observations, expected_tag_id)
        checkpoints.append(
            {
                "target_id": target_id,
                "expected_tag_id": expected_tag_id,
                "verified": matching_observation is not None,
                "observation_id": matching_observation.id if matching_observation else None,
            }
        )

    return checkpoints


def build_report_data(state: DogOpsState) -> dict[str, object]:
    package_statuses = list(state.package_statuses.values())
    current_exception_package_ids = {
        status.package_id
        for status in package_statuses
        if status.state
        in {PackageState.missing, PackageState.wrong_zone, PackageState.blocking_asset}
    }
    incident_exception_package_ids = {
        incident.related_package_id
        for incident in state.incidents
        if incident.related_package_id is not None
        and incident.type
        in {
            IncidentType.blocked_cooling,
            IncidentType.wrong_zone,
            IncidentType.missing_package,
            IncidentType.damaged_package,
        }
    }
    manifest_exception_package_ids = current_exception_package_ids | incident_exception_package_ids
    verified_work_orders = [
        work_order
        for work_order in state.work_orders
        if work_order.state == WorkOrderState.verified_closed
    ]
    open_incidents = [
        incident for incident in state.incidents if incident.state != IncidentState.resolved
    ]
    resolved_incidents = [
        incident for incident in state.incidents if incident.state == IncidentState.resolved
    ]
    nav = state.nav_summary
    checkpoints = build_checkpoint_verifications(state)
    return {
        "run_id": state.run.id,
        "mission_id": state.run.mission_id,
        "mission_state": _value(state.run.state),
        "summary": state.run.summary,
        "packages_expected": len(package_statuses),
        "packages_observed": len(
            [status for status in package_statuses if status.observed_zone_id is not None]
        ),
        "manifest_exceptions": len(manifest_exception_package_ids),
        "incidents_opened": len(state.incidents),
        "incidents_resolved": len(resolved_incidents),
        "work_orders_verified_closed": len(verified_work_orders),
        "open_issues": [
            {
                "incident_id": incident.id,
                "type": _value(incident.type),
                "entity_id": incident.entity_id,
                "related_package_id": incident.related_package_id,
                "state": _value(incident.state),
            }
            for incident in open_incidents
        ],
        "what_changed": state.what_changed,
        "nav_summary": nav.model_dump(mode="json") if nav else None,
        "checkpoints_total": len(checkpoints),
        "checkpoints_verified": len([checkpoint for checkpoint in checkpoints if checkpoint["verified"]]),
        "checkpoint_verifications": checkpoints,
        "packages": [status.model_dump(mode="json") for status in package_statuses],
        "incidents": [incident.model_dump(mode="json") for incident in state.incidents],
        "work_orders": [
            work_order.model_dump(mode="json") for work_order in state.work_orders
        ],
    }


def render_report_markdown(state: DogOpsState) -> str:
    data = build_report_data(state)
    nav = state.nav_summary
    lines = [
        "DOGOPS RUN REPORT",
        f"Mission: {state.run.mission_id}",
        (
            "Packages scanned: "
            f"{data['packages_observed']} observed / {data['packages_expected']} expected"
        ),
        f"Manifest exceptions: {data['manifest_exceptions']}",
        f"Incidents opened: {data['incidents_opened']}",
        f"Work orders verified closed: {data['work_orders_verified_closed']}",
    ]

    open_issues = data["open_issues"]
    if open_issues:
        issue_text = ", ".join(
            f"{issue['related_package_id'] or issue['entity_id']} {issue['type']}"
            for issue in open_issues  # type: ignore[union-attr]
        )
        lines.append(f"Open issue: {issue_text}")
    else:
        lines.append("Open issue: none")

    if nav:
        lines.append(
            "Nav: "
            f"{nav.waypoints_reached}/{nav.waypoints_total} waypoints reached, "
            f"{nav.tag_reacquisition_successes} tag-search recovery, "
            f"{nav.safety_stops} safety stops"
        )
        for note in nav.notes:
            lines.append(f"Nav evidence: {note}")
    lines.append(
        "Checkpoints: "
        f"{data['checkpoints_verified']}/{data['checkpoints_total']} tag sign-ins verified"
    )

    for changed in state.what_changed:
        lines.append(f"What changed: {changed}")

    lines.extend(["", "Incidents:"])
    for incident in state.incidents:
        lines.append(
            "- "
            f"{incident.id} {_value(incident.severity)} {_value(incident.type)}: "
            f"{incident.title} [{_value(incident.state)}]"
        )

    lines.extend(["", "Packages:"])
    for status in state.package_statuses.values():
        location = status.observed_zone_id or "not observed"
        blocker = f", blocks {status.blocks_asset_id}" if status.blocks_asset_id else ""
        lines.append(
            f"- {status.package_id}: {_value(status.state)}, expected {status.expected_zone_id}, "
            f"observed {location}{blocker}"
        )

    return "\n".join(lines) + "\n"


def _first_observation_with_tag(
    observations: list[Observation],
    tag_id: int | None,
) -> Observation | None:
    if tag_id is None:
        return None
    for observation in observations:
        if tag_id in _observation_tag_ids(observation):
            return observation
    return None


def _observation_tag_ids(observation: Observation) -> set[int]:
    tag_ids = set()
    if observation.tag_id is not None:
        tag_ids.add(observation.tag_id)
    raw_visible = observation.facts.get("visible_tag_ids")
    if isinstance(raw_visible, str):
        for item in raw_visible.split(","):
            item = item.strip()
            if not item:
                continue
            try:
                tag_ids.add(int(item))
            except ValueError:
                continue
    elif isinstance(raw_visible, int):
        tag_ids.add(raw_visible)
    return tag_ids


def assert_report_has_closed_loop(state: DogOpsState) -> None:
    incidents_by_id = {incident.id: incident for incident in state.incidents}
    work_orders_by_id = {work_order.id: work_order for work_order in state.work_orders}
    pkg_103 = state.package_statuses.get("PKG-103")
    pkg_104 = state.package_statuses.get("PKG-104")
    if incidents_by_id.get("INC-001") is None:
        raise AssertionError("INC-001 was not opened")
    if incidents_by_id["INC-001"].state != IncidentState.resolved:
        raise AssertionError("INC-001 was not verified closed")
    if work_orders_by_id.get("WO-001") is None:
        raise AssertionError("WO-001 was not opened")
    if work_orders_by_id["WO-001"].state != WorkOrderState.verified_closed:
        raise AssertionError("WO-001 was not verified closed")
    if not pkg_103 or pkg_103.state != PackageState.missing:
        raise AssertionError("PKG-103 must remain missing/open")
    if not pkg_104 or pkg_104.observed_zone_id != "QA_HOLD":
        raise AssertionError("PKG-104 must be moved to QA_HOLD")
    if not state.nav_summary:
        raise AssertionError("nav summary is missing")
    if not any(incident.type == IncidentType.missing_package for incident in state.incidents):
        raise AssertionError("missing package incident is missing")
