from dimos.experimental.dogops.mission_engine import run_offline_simulation
from dimos.experimental.dogops.store import DogOpsStore


def test_store_writes_required_run_files(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)

    assert (run_dir / "run.json").is_file()
    assert (run_dir / "observations.jsonl").is_file()
    assert (run_dir / "incidents.jsonl").is_file()
    assert (run_dir / "work_orders.jsonl").is_file()
    assert (run_dir / "nav_events.jsonl").is_file()
    assert (run_dir / "state.json").is_file()
    assert (run_dir / "report.json").is_file()
    assert (run_dir / "report.md").is_file()
    assert (run_dir / "evidence").is_dir()

    state = DogOpsStore.load_existing(run_dir).load_state()
    assert state.run.id == "latest"
    assert len(state.incidents) == 2
