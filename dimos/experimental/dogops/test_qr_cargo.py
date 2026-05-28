from __future__ import annotations

import json
from pathlib import Path

import pytest

from dimos.experimental.dogops.qr_cargo import (
    append_qr_event,
    get_latest_qr_events,
    load_qr_events,
    validate_qr_payload,
)


def _sample_event() -> dict[str, object]:
    return json.loads(
        Path("examples/dogops/qr_cargo_event_sample.json").read_text(encoding="utf-8")
    )


def test_qr_payload_validation_accepts_valid_cargo_payload() -> None:
    event = _sample_event()
    ok, errors = validate_qr_payload(event["qr_payload"])  # type: ignore[arg-type]

    assert ok is True
    assert errors == []


@pytest.mark.parametrize("field", ["warehouse_id", "location_node_id", "cargo_id"])
def test_qr_payload_validation_rejects_missing_required_fields(field: str) -> None:
    event = _sample_event()
    payload = dict(event["qr_payload"])  # type: ignore[arg-type]
    payload.pop(field)

    ok, errors = validate_qr_payload(payload)

    assert ok is False
    assert f"qr_payload.{field} is required" in errors


def test_qr_event_append_loads_latest_and_writes_state(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    event = _sample_event()

    normalized = append_qr_event(run_dir, event)
    events = load_qr_events(run_dir)
    latest = get_latest_qr_events(run_dir)

    assert normalized["event_id"]
    assert normalized["action_policy"] == "report_only"
    assert (run_dir / "qr_events.jsonl").is_file()
    assert (run_dir / "qr_cargo_state.json").is_file()
    assert events == [normalized]
    assert latest == [normalized]
