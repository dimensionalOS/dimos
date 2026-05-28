from dimos.experimental.dogops.mission_engine import run_offline_simulation
from dimos.experimental.dogops.models import NavAction, NavEvent
from dimos.experimental.dogops.nav_eval import DogOpsNavEvalModule, summarize_nav_events


def test_summarize_nav_events_records_retries_and_tag_recovery() -> None:
    events = [
        NavEvent(
            id="NAV-001",
            run_id="run-1",
            ts=1.0,
            action=NavAction.goto,
            target_id="HOME",
            success=True,
            elapsed_s=4.0,
        ),
        NavEvent(
            id="NAV-002",
            run_id="run-1",
            ts=2.0,
            action=NavAction.goto,
            target_id="COOLING_1",
            success=True,
            elapsed_s=10.0,
            retries=1,
            note="tag search recovery used",
        ),
    ]

    summary = summarize_nav_events("run-1", events)

    assert summary.waypoints_total == 2
    assert summary.waypoints_reached == 2
    assert summary.retries_total == 1
    assert summary.tag_reacquisition_attempts == 1
    assert summary.tag_reacquisition_successes == 1
    assert summary.tag_reacquisition_rate == 1.0
    assert summary.route_targets == 2
    assert summary.unique_targets_reached == 2
    assert summary.route_coverage == 1.0
    assert summary.guided_fallback_used is False
    assert summary.worst_target_id == "COOLING_1"


def test_nav_eval_module_summarizes_run_directory(tmp_path) -> None:
    run_offline_simulation(out=tmp_path / "latest")
    module = DogOpsNavEvalModule()

    summary = module.summarize_run(tmp_path / "latest")

    assert '"route_coverage": 1.0' in summary
    assert '"waypoints_reached": 4' in summary
