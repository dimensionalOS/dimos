from __future__ import annotations

from dimos.experimental.dogops.heatmap_runs import gather_heatmap_run, heatmap_snapshot_for_route_run


def _snapshot(cost: float) -> dict[str, object]:
    return {
        "ok": True,
        "source": "DimOS live LCM topics",
        "status": "receiving",
        "costmap": {
            "source": "DimOS live costmap",
            "columns": 1,
            "rows": 1,
            "cells": [{"x": 1.0, "y": 2.0, "width": 0.5, "height": 0.5, "cost": cost}],
        },
        "path": [],
        "route": [],
        "robot_pose": None,
        "target": None,
    }


def test_gather_heatmap_samples_for_duration_and_merges_max_cost(tmp_path) -> None:
    reads = iter([_snapshot(0.8), _snapshot(0.4)])
    sleeps: list[float] = []

    def read_snapshot() -> dict[str, object]:
        return next(reads)

    def sleep(seconds: float) -> None:
        sleeps.append(seconds)

    result = gather_heatmap_run(
        tmp_path / ".dogops" / "runs" / "latest",
        live_snapshot=_snapshot(0.2),
        live_snapshot_reader=read_snapshot,
        area_id="AISLE_1",
        duration_s=0.2,
        sample_interval_s=0.2,
        sleep_fn=sleep,
    )

    assert result["ok"] is True
    assert sleeps
    cells = result["heatmap"]["costmap"]["cells"]  # type: ignore[index]
    assert cells[0]["cost"] == 0.8
    assert result["heatmap"]["area_id"] == "AISLE_1"  # type: ignore[index]
    assert heatmap_snapshot_for_route_run(
        tmp_path / ".dogops" / "runs" / "latest",
        str(result["route_run_id"]),
    )["costmap"]["cells"][0]["cost"] == 0.8
