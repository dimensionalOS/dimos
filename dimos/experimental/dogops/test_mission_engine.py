from dimos.experimental.dogops.mission_engine import run_offline_simulation
from dimos.experimental.dogops.models import IncidentState, PackageState, WorkOrderState


def test_offline_simulation_closes_blocked_cooling_and_leaves_missing_package(tmp_path) -> None:
    state = run_offline_simulation(out=tmp_path / "run")

    incidents = {incident.id: incident for incident in state.incidents}
    work_orders = {work_order.id: work_order for work_order in state.work_orders}

    assert state.run.state == "done"
    assert incidents["INC-001"].state == IncidentState.resolved
    assert incidents["INC-001"].type == "blocked_cooling"
    assert incidents["INC-001"].related_package_id == "PKG-104"
    assert work_orders["WO-001"].state == WorkOrderState.verified_closed
    assert incidents["INC-002"].state == IncidentState.open
    assert incidents["INC-002"].related_package_id == "PKG-103"
    assert state.package_statuses["PKG-103"].state == PackageState.missing
    assert state.package_statuses["PKG-104"].observed_zone_id == "QA_HOLD"
    assert state.nav_summary is not None
    assert state.nav_summary.waypoints_reached == 4
