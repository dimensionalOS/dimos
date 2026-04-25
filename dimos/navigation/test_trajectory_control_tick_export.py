# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for trajectory control tick JSONL export (P2-2)."""

import json
import math
from pathlib import Path

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_control_tick_export import (
    TRAJECTORY_CONTROL_TICK_JSONL_SCHEMA_VERSION,
    iter_trajectory_control_tick_jsonl,
    trajectory_control_tick_to_jsonl_dict,
    trajectory_control_ticks_to_jsonl_lines,
    write_trajectory_control_ticks_jsonl,
)
from dimos.navigation.trajectory_control_tick_log import (
    TrajectoryControlTick,
    trajectory_control_tick_from_samples,
)
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample

_FIXTURE_DIR = Path(__file__).parent / "fixtures"
SAMPLE_JSONL = _FIXTURE_DIR / "trajectory_control_ticks_sample.jsonl"


def _tick(
    *,
    ref_xyyaw: tuple[float, float, float] = (0.0, 0.0, 0.0),
    meas_xyyaw: tuple[float, float, float] = (0.1, 0.0, 0.0),
    wall: float | None = None,
    sim: float | None = None,
) -> TrajectoryControlTick:
    rx, ry, ryaw = ref_xyyaw
    mx, my, myaw = meas_xyyaw
    ref = TrajectoryReferenceSample(
        time_s=0.0,
        pose_plan=Pose(rx, ry, 0.0, 0.0, 0.0, math.sin(ryaw / 2), math.cos(ryaw / 2)),
        twist_body=Twist(linear=Vector3(0.2, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)),
    )
    meas = TrajectoryMeasuredSample(
        time_s=0.0,
        pose_plan=Pose(mx, my, 0.0, 0.0, 0.0, math.sin(myaw / 2), math.cos(myaw / 2)),
        twist_body=Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)),
    )
    cmd = Twist(linear=Vector3(0.1, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.05))
    return trajectory_control_tick_from_samples(
        ref,
        meas,
        cmd,
        0.05,
        wall_time_s=wall,
        sim_time_s=sim,
    )


def test_jsonl_dict_schema_and_field_order() -> None:
    tick = _tick()
    d = trajectory_control_tick_to_jsonl_dict(tick)
    keys = list(d.keys())
    assert keys[0] == "schema_version"
    assert d["schema_version"] == TRAJECTORY_CONTROL_TICK_JSONL_SCHEMA_VERSION
    assert keys[1] == "ref_time_s"
    assert keys[-2] == "wall_time_s"
    assert keys[-1] == "sim_time_s"


def test_jsonl_lines_roundtrip(tmp_path: Path) -> None:
    a = _tick(wall=None, sim=None)
    b = _tick(
        ref_xyyaw=(1.0, 2.0, math.pi / 4),
        meas_xyyaw=(1.05, 2.01, math.pi / 4),
        wall=1_700_000_000.0,
        sim=42.0,
    )
    text = trajectory_control_ticks_to_jsonl_lines([a, b])
    out = tmp_path / "out.jsonl"
    out.write_text(text, encoding="utf-8")
    rows = list(iter_trajectory_control_tick_jsonl(out))
    assert len(rows) == 2
    assert rows[0]["schema_version"] == 1
    assert rows[0]["wall_time_s"] is None
    assert rows[0]["sim_time_s"] is None
    assert rows[1]["wall_time_s"] == pytest.approx(1_700_000_000.0)
    assert rows[1]["sim_time_s"] == pytest.approx(42.0)


def test_write_trajectory_control_ticks_jsonl(tmp_path: Path) -> None:
    tick = _tick()
    path = tmp_path / "sub" / "ticks.jsonl"
    write_trajectory_control_ticks_jsonl([tick], path)
    assert path.is_file()
    raw = path.read_text(encoding="utf-8")
    assert raw.endswith("\n")
    row = json.loads(raw.strip())
    assert row["schema_version"] == 1
    assert row["dt_s"] == pytest.approx(0.05)


def test_sample_fixture_matches_schema_and_loads() -> None:
    assert SAMPLE_JSONL.is_file()
    rows = list(iter_trajectory_control_tick_jsonl(SAMPLE_JSONL))
    assert len(rows) == 2
    for row in rows:
        assert row["schema_version"] == TRAJECTORY_CONTROL_TICK_JSONL_SCHEMA_VERSION
        assert "planar_position_divergence_m" in row
        assert "commanded_planar_speed_m_s" in row
    expected = trajectory_control_ticks_to_jsonl_lines(
        [_tick(), _tick(ref_xyyaw=(2.0, 0.5, 0.0), meas_xyyaw=(2.02, 0.48, 0.0), wall=99.0, sim=0.5)]
    )
    assert SAMPLE_JSONL.read_text(encoding="utf-8") == expected
