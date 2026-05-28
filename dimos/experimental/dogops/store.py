from __future__ import annotations

import json
from pathlib import Path

from dimos.experimental.dogops.models import (
    DogOpsState,
    Incident,
    Manifest,
    MissionConfig,
    MissionRun,
    MissionState,
    NavEvent,
    Observation,
    PackageStatus,
    PolicyConfig,
    SiteConfig,
    WorkOrder,
)
from dimos.experimental.dogops.report import build_report_data, render_report_markdown


class DogOpsStore:
    """JSON/JSONL run store for deterministic offline and demo runs."""

    def __init__(
        self,
        root: str | Path,
        *,
        site: SiteConfig,
        manifest: Manifest,
        policy: PolicyConfig,
        mission: MissionConfig,
    ) -> None:
        self.root = Path(root)
        self.evidence_dir = self.root / "evidence"
        self.site = site
        self.manifest = manifest
        self.policy = policy
        self.mission = mission
        self.state: DogOpsState | None = None

    def create_run(self, mission_id: str, started_at: float) -> MissionRun:
        self.root.mkdir(parents=True, exist_ok=True)
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        run = MissionRun(
            id=self.root.name,
            mission_id=mission_id,
            started_at=started_at,
            state=MissionState.running,
            summary="DogOps offline simulation running",
        )
        package_statuses = {
            item.package_id: PackageStatus(
                package_id=item.package_id,
                expected_zone_id=item.expected_zone_id,
            )
            for item in self.manifest.items
        }
        self.state = DogOpsState(
            run=run,
            site=self.site,
            manifest=self.manifest,
            policy=self.policy,
            mission=self.mission,
            package_statuses=package_statuses,
        )
        self._write_json(self.root / "run.json", run.model_dump(mode="json"))
        self.write_state(run.id)
        return run

    def finish_run(self, run_id: str, state: MissionState, summary: str, ended_at: float) -> MissionRun:
        dogops_state = self._require_state(run_id)
        dogops_state.run.state = state
        dogops_state.run.summary = summary
        dogops_state.run.ended_at = ended_at
        dogops_state.run.current_step_id = None
        self._write_json(self.root / "run.json", dogops_state.run.model_dump(mode="json"))
        self.write_state(run_id)
        return dogops_state.run

    def append_observation(self, obs: Observation) -> None:
        state = self._require_state(obs.run_id)
        state.observations.append(obs)
        self._append_jsonl("observations.jsonl", obs.model_dump(mode="json"))

    def append_incident(self, incident: Incident) -> None:
        state = self._require_state(incident.run_id)
        state.incidents.append(incident)
        self._append_jsonl("incidents.jsonl", incident.model_dump(mode="json"))

    def update_incident(self, incident: Incident) -> None:
        state = self._require_state(incident.run_id)
        state.incidents = [incident if item.id == incident.id else item for item in state.incidents]
        self._rewrite_jsonl("incidents.jsonl", [item.model_dump(mode="json") for item in state.incidents])

    def append_work_order(self, work_order: WorkOrder) -> None:
        state = self._require_state_by_any_run()
        state.work_orders.append(work_order)
        self._append_jsonl("work_orders.jsonl", work_order.model_dump(mode="json"))

    def update_work_order(self, work_order: WorkOrder) -> None:
        state = self._require_state_by_any_run()
        state.work_orders = [
            work_order if item.id == work_order.id else item for item in state.work_orders
        ]
        self._rewrite_jsonl(
            "work_orders.jsonl", [item.model_dump(mode="json") for item in state.work_orders]
        )

    def append_nav_event(self, nav_event: NavEvent) -> None:
        state = self._require_state(nav_event.run_id)
        state.nav_events.append(nav_event)
        self._append_jsonl("nav_events.jsonl", nav_event.model_dump(mode="json"))

    def write_state(self, run_id: str) -> Path:
        state = self._require_state(run_id)
        path = self.root / "state.json"
        self._write_json(path, state.model_dump(mode="json"))
        return path

    def write_report(self, run_id: str) -> tuple[Path, Path]:
        state = self._require_state(run_id)
        report_json = self.root / "report.json"
        report_md = self.root / "report.md"
        self._write_json(report_json, build_report_data(state))
        report_md.write_text(render_report_markdown(state), encoding="utf-8")
        return report_json, report_md

    def load_state(self, run_id: str | None = None) -> DogOpsState:
        path = self.root / "state.json"
        state = DogOpsState.model_validate_json(path.read_text(encoding="utf-8"))
        if run_id is not None and state.run.id != run_id:
            raise ValueError(f"loaded run {state.run.id!r}, expected {run_id!r}")
        self.state = state
        return state

    @classmethod
    def load_existing(cls, root: str | Path) -> DogOpsStore:
        path = Path(root)
        state = DogOpsState.model_validate_json((path / "state.json").read_text(encoding="utf-8"))
        store = cls(
            path,
            site=state.site,
            manifest=state.manifest,
            policy=state.policy,
            mission=state.mission,
        )
        store.state = state
        return store

    def _require_state(self, run_id: str) -> DogOpsState:
        if self.state is None:
            raise RuntimeError("DogOps run has not been created")
        if self.state.run.id != run_id:
            raise ValueError(f"active run {self.state.run.id!r} does not match {run_id!r}")
        return self.state

    def _require_state_by_any_run(self) -> DogOpsState:
        if self.state is None:
            raise RuntimeError("DogOps run has not been created")
        return self.state

    def _append_jsonl(self, filename: str, payload: dict[str, object]) -> None:
        with (self.root / filename).open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, sort_keys=True) + "\n")

    def _rewrite_jsonl(self, filename: str, rows: list[dict[str, object]]) -> None:
        with (self.root / filename).open("w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row, sort_keys=True) + "\n")

    @staticmethod
    def _write_json(path: Path, payload: dict[str, object]) -> None:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
