from dimos.experimental.dogops.mission_engine import run_offline_simulation
from dimos.experimental.dogops.report import build_report_data, render_report_markdown


def test_report_contains_closed_loop_facts(tmp_path) -> None:
    state = run_offline_simulation(out=tmp_path / "run")

    data = build_report_data(state)
    report = render_report_markdown(state)

    assert data["manifest_exceptions"] == 2
    assert data["incidents_opened"] == 2
    assert data["work_orders_verified_closed"] == 1
    assert data["checkpoints_total"] == 4
    assert data["checkpoints_verified"] == 4
    assert data["checkpoint_verifications"] == [
        {
            "target_id": "HOME",
            "expected_tag_id": 10,
            "verified": True,
            "observation_id": "OBS-001",
        },
        {
            "target_id": "INBOUND_DOCK",
            "expected_tag_id": 20,
            "verified": True,
            "observation_id": "OBS-002",
        },
        {
            "target_id": "COOLING_1",
            "expected_tag_id": 41,
            "verified": True,
            "observation_id": "OBS-003",
        },
        {
            "target_id": "QA_HOLD",
            "expected_tag_id": 30,
            "verified": True,
            "observation_id": "OBS-005",
        },
    ]
    assert "PKG-104 wrong zone and blocking COOLING_1" in report
    assert "INC-001 P1 blocked_cooling" in report
    assert "PKG-103 missing_package" in report
    assert "What changed: PKG-104 moved from COOLING_1/RACK_ROW_A to QA_HOLD" in report
    assert "Nav: 4/4 waypoints reached, 1 tag-search recovery, 0 safety stops" in report
    assert "Checkpoints: 4/4 tag sign-ins verified" in report
