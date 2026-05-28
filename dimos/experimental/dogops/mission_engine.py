from __future__ import annotations

from pathlib import Path
import time

from dimos.experimental.dogops.config_loader import load_dogops_config
from dimos.experimental.dogops.models import (
    DogOpsConfig,
    DogOpsState,
    Incident,
    IncidentState,
    IncidentType,
    MissionState,
    NavEvent,
    Observation,
    PackageState,
    Severity,
    SimulationObservation,
    WorkOrder,
    WorkOrderState,
)
from dimos.experimental.dogops.nav_eval import summarize_nav_events
from dimos.experimental.dogops.report import assert_report_has_closed_loop
from dimos.experimental.dogops.store import DogOpsStore


class OfflineMissionEngine:
    def __init__(self, config: DogOpsConfig, out_dir: str | Path) -> None:
        self.config = config
        self.store = DogOpsStore(
            out_dir,
            site=config.site,
            manifest=config.manifest,
            policy=config.policy,
            mission=config.mission,
        )
        self._obs_count = 0
        self._nav_count = 0

    def run(self, *, verify_closed_loop: bool = True) -> DogOpsState:
        started_at = time.time()
        run = self.store.create_run(self.config.mission.mission_id, started_at=started_at)
        state = self.store.state
        assert state is not None

        self._record_nav_events(run.id)
        self._localize_home(run.id)
        self._scan("scan_inbound", run.id, state)
        self._scan("inspect_cooling", run.id, state)
        self._open_blocked_cooling_if_needed(run.id, state)
        self._open_missing_package_incidents(run.id, state)
        self._mark_work_order_ready("WO-001", state)
        self._scan("verify_cooling_after_fix", run.id, state, source="simulated_human_fix")
        self._verify_work_order("WO-001", state)
        self._scan("scan_qa_hold", run.id, state)

        state.nav_summary = summarize_nav_events(run.id, state.nav_events)
        summary = "Closed INC-001 after simulated human remediation; PKG-103 remains missing."
        self.store.finish_run(run.id, MissionState.done, summary, ended_at=time.time())
        self.store.write_state(run.id)
        self.store.write_report(run.id)
        if verify_closed_loop:
            assert_report_has_closed_loop(state)
        return state

    def _record_nav_events(self, run_id: str) -> None:
        for sim_event in self.config.mission.nav_simulation.events:
            self._nav_count += 1
            event = NavEvent(
                id=f"NAV-{self._nav_count:03d}",
                run_id=run_id,
                ts=time.time(),
                action=sim_event.action,
                target_id=sim_event.target_id,
                success=sim_event.success,
                elapsed_s=sim_event.elapsed_s,
                retries=sim_event.retries,
                guided=sim_event.guided or self.config.mission.nav_simulation.guided,
                error_m=sim_event.error_m,
                note=sim_event.note,
            )
            self.store.append_nav_event(event)

    def _localize_home(self, run_id: str) -> None:
        self._obs_count += 1
        obs = Observation(
            id=f"OBS-{self._obs_count:03d}",
            ts=time.time(),
            run_id=run_id,
            entity_id="HOME",
            tag_id=10,
            zone_id="HOME",
            facts={"localized": True},
            confidence=1.0,
            source="simulation",
        )
        self.store.append_observation(obs)

    def _scan(
        self,
        key: str,
        run_id: str,
        state: DogOpsState,
        *,
        source: str = "simulation",
    ) -> Observation:
        sim_obs = self.config.mission.simulation_observations.get(key)
        if sim_obs is None:
            raise KeyError(f"missing simulation observation: {key}")
        obs = self._observation_from_simulation(key, run_id, sim_obs, source=source)
        self.store.append_observation(obs)
        self._apply_observation_facts(obs, state)
        return obs

    def _observation_from_simulation(
        self, key: str, run_id: str, sim_obs: SimulationObservation, *, source: str
    ) -> Observation:
        self._obs_count += 1
        tag_to_entity = self.config.site.entity_for_tag()
        primary_entity_id = None
        primary_tag_id = sim_obs.visible_tag_ids[0] if sim_obs.visible_tag_ids else None
        if primary_tag_id is not None and primary_tag_id in tag_to_entity:
            primary_entity_id = tag_to_entity[primary_tag_id].id
        facts: dict[str, bool | str | int | float] = {
            "scan_key": key,
            "visible_tag_ids": ",".join(str(tag_id) for tag_id in sim_obs.visible_tag_ids),
        }
        facts.update(sim_obs.facts)
        return Observation(
            id=f"OBS-{self._obs_count:03d}",
            ts=time.time(),
            run_id=run_id,
            entity_id=primary_entity_id,
            tag_id=primary_tag_id,
            zone_id=sim_obs.zone_id,
            facts=facts,
            confidence=1.0,
            source=source,
        )

    def _apply_observation_facts(self, obs: Observation, state: DogOpsState) -> None:
        for key, value in obs.facts.items():
            if key.endswith(".zone_id"):
                package_id = key.removesuffix(".zone_id")
                status = state.package_statuses.get(package_id)
                if status is None or not isinstance(value, str):
                    continue
                previous_zone = status.observed_zone_id
                status.observed_zone_id = value
                if value == status.expected_zone_id:
                    status.state = PackageState.found_ok
                    status.blocks_asset_id = None
                else:
                    status.state = PackageState.wrong_zone
                if package_id == "PKG-104" and previous_zone in {"RACK_ROW_A", "COOLING_1"}:
                    if value == "QA_HOLD":
                        state.what_changed.append(
                            "PKG-104 moved from COOLING_1/RACK_ROW_A to QA_HOLD; INC-001 resolved."
                        )
            elif key.endswith(".blocks_asset_id"):
                package_id = key.removesuffix(".blocks_asset_id")
                status = state.package_statuses.get(package_id)
                if status is None or not isinstance(value, str):
                    continue
                status.blocks_asset_id = value

    def _open_blocked_cooling_if_needed(self, run_id: str, state: DogOpsState) -> None:
        pkg_104 = state.package_statuses.get("PKG-104")
        if pkg_104 is None or pkg_104.blocks_asset_id != "COOLING_1":
            return
        if any(incident.id == "INC-001" for incident in state.incidents):
            return
        rule = self.config.policy.rule_for_type(IncidentType.blocked_cooling)
        severity = Severity(rule.severity) if rule else Severity.P1
        action = (
            rule.recommended_action
            if rule
            else "Move blocking package to QA_HOLD and verify COOLING_1 is clear."
        )
        incident = Incident(
            id="INC-001",
            run_id=run_id,
            ts_open=time.time(),
            severity=severity,
            type=IncidentType.blocked_cooling,
            entity_id="COOLING_1",
            related_package_id="PKG-104",
            state=IncidentState.open,
            title="PKG-104 wrong zone and blocking COOLING_1",
            evidence_observation_ids=[obs.id for obs in state.observations if obs.zone_id == "RACK_ROW_A"],
            recommended_action=action,
        )
        work_order = WorkOrder(
            id="WO-001",
            incident_id=incident.id,
            requested_action=action,
            state=WorkOrderState.assigned,
        )
        self.store.append_incident(incident)
        self.store.append_work_order(work_order)

    def _open_missing_package_incidents(self, run_id: str, state: DogOpsState) -> None:
        for package_id, status in state.package_statuses.items():
            if status.observed_zone_id is not None:
                continue
            if any(incident.related_package_id == package_id for incident in state.incidents):
                continue
            status.state = PackageState.missing
            rule = self.config.policy.rule_for_type(IncidentType.missing_package)
            incident = Incident(
                id="INC-002",
                run_id=run_id,
                ts_open=time.time(),
                severity=Severity(rule.severity) if rule else Severity.P2,
                type=IncidentType.missing_package,
                entity_id=package_id,
                related_package_id=package_id,
                state=IncidentState.open,
                title=f"{package_id} missing from inbound scan",
                evidence_observation_ids=[obs.id for obs in state.observations],
                recommended_action=(
                    rule.recommended_action if rule else "Search inbound dock and QA_HOLD."
                ),
            )
            self.store.append_incident(incident)

    def _mark_work_order_ready(self, work_order_id: str, state: DogOpsState) -> None:
        work_order = self._get_work_order(work_order_id, state)
        work_order.state = WorkOrderState.ready_to_verify
        self.store.update_work_order(work_order)
        incident = self._get_incident(work_order.incident_id, state)
        incident.state = IncidentState.ready_to_verify
        self.store.update_incident(incident)

    def _verify_work_order(self, work_order_id: str, state: DogOpsState) -> None:
        work_order = self._get_work_order(work_order_id, state)
        incident = self._get_incident(work_order.incident_id, state)
        pkg_104 = state.package_statuses["PKG-104"]
        cooling_clear = any(
            obs.facts.get("COOLING_1.clearance_clear") is True for obs in state.observations
        )
        if pkg_104.observed_zone_id == "QA_HOLD" and cooling_clear:
            incident.state = IncidentState.resolved
            incident.ts_closed = time.time()
            work_order.state = WorkOrderState.verified_closed
            work_order.verification_observation_ids = [
                obs.id for obs in state.observations if obs.source == "simulated_human_fix"
            ]
            self.store.update_incident(incident)
            self.store.update_work_order(work_order)

    @staticmethod
    def _get_incident(incident_id: str, state: DogOpsState) -> Incident:
        for incident in state.incidents:
            if incident.id == incident_id:
                return incident
        raise KeyError(incident_id)

    @staticmethod
    def _get_work_order(work_order_id: str, state: DogOpsState) -> WorkOrder:
        for work_order in state.work_orders:
            if work_order.id == work_order_id:
                return work_order
        raise KeyError(work_order_id)


def run_offline_simulation(
    *,
    site: str | Path = "examples/dogops/site_demo.yaml",
    manifest: str | Path = "examples/dogops/manifest_demo.yaml",
    mission: str | Path = "examples/dogops/mission_demo.yaml",
    policy: str | Path = "examples/dogops/policy_demo.yaml",
    out: str | Path = ".dogops/runs/latest",
    verify_closed_loop: bool = True,
) -> DogOpsState:
    config = load_dogops_config(site, manifest, mission, policy)
    engine = OfflineMissionEngine(config, out)
    return engine.run(verify_closed_loop=verify_closed_loop)
