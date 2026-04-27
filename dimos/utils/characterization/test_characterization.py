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

"""Plumbing tests for the characterization harness.

These tests only validate the harness itself — scheduling, recipe
evaluation, artifact shapes. They do not validate dynamics. For
dynamics validation, run ``python -m dimos.utils.characterization.scripts.run_session --backend go2
--simulation`` against the mujoco sim.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from dimos.utils.characterization.recipes import (
    TestRecipe,
    chirp,
    composite,
    constant,
    ramp,
    step,
)


def test_step_signal_holds_amplitude() -> None:
    fn = step(amplitude=0.7, channel="vx")
    assert fn(0.0) == (0.7, 0.0, 0.0)
    assert fn(10.0) == (0.7, 0.0, 0.0)


def test_step_respects_t_start() -> None:
    fn = step(amplitude=0.5, channel="wz", t_start=0.3)
    assert fn(0.1) == (0.0, 0.0, 0.0)
    assert fn(0.4) == (0.0, 0.0, 0.5)


def test_ramp_is_linear() -> None:
    fn = ramp(start=0.0, end=2.0, duration=1.0, channel="vx")
    assert fn(0.0) == pytest.approx((0.0, 0.0, 0.0))
    assert fn(0.25)[0] == pytest.approx(0.5)
    assert fn(0.5)[0] == pytest.approx(1.0)
    assert fn(1.0)[0] == pytest.approx(2.0)
    # Clamp at ends
    assert fn(2.0)[0] == pytest.approx(2.0)
    assert fn(-1.0)[0] == pytest.approx(0.0)


def test_constant_is_constant() -> None:
    fn = constant(vx=0.1, vy=-0.2, wz=0.3)
    for t in (0.0, 0.5, 1.0, 100.0):
        assert fn(t) == (0.1, -0.2, 0.3)


def test_composite_sums_channels() -> None:
    fn = composite(step(0.5, "vx"), step(0.2, "wz"))
    assert fn(1.0) == (0.5, 0.0, 0.2)


def test_chirp_stays_in_amplitude_band() -> None:
    fn = chirp(f_min_hz=0.5, f_max_hz=3.0, duration=2.0, amplitude=1.0, mean=0.5, channel="vx")
    samples = [fn(t)[0] for t in np.linspace(0, 2.0, 200)]
    assert min(samples) >= -0.5 - 1e-6
    assert max(samples) <= 1.5 + 1e-6


def test_recipe_serialize_excludes_callable() -> None:
    recipe = TestRecipe(
        name="x",
        test_type="step",
        duration_s=1.0,
        signal_fn=step(0.3, "vx"),
        metadata={"note": "hi"},
    )
    d = recipe.serialize()
    assert "signal_fn" not in d
    assert d["name"] == "x"
    assert d["metadata"] == {"note": "hi"}


def test_runner_smoke_produces_artifacts(tmp_path: Path) -> None:
    """End-to-end runner test — no coordinator, just publishes into the ether.

    Validates: run dir created, cmd_monotonic.jsonl has expected sample count,
    run.json has begin+end metadata, exit_reason is 'ok'.
    """
    from dimos.utils.characterization.runner import CharacterizationSession

    recipe = TestRecipe(
        name="pytest_step",
        test_type="step",
        duration_s=0.2,
        signal_fn=step(0.3, "vx"),
        sample_rate_hz=50.0,
        pre_roll_s=0.1,
        post_roll_s=0.1,
    )
    with CharacterizationSession(
        cmd_vel_topic="/pytest_cmd_vel",
        output_root=tmp_path,
    ) as sess:
        result = sess.run(recipe)
    assert result.exit_reason == "ok"
    assert result.run_dir.exists()
    assert result.run_json.exists()
    assert result.cmd_monotonic_jsonl.exists()

    # ~0.4s at 50Hz = ~20 samples. Allow ±2 for scheduling jitter.
    assert 18 <= result.n_commanded <= 22, f"got {result.n_commanded}"

    with result.run_json.open() as fh:
        meta = json.load(fh)
    assert meta["exit_reason"] == "ok"
    assert meta["n_commanded"] == result.n_commanded
    assert "clock_anchor" in meta
    assert meta["clock_anchor"]["monotonic"] > 0
    assert meta["recipe"]["name"] == "pytest_step"

    with result.cmd_monotonic_jsonl.open() as fh:
        lines = fh.readlines()
    assert len(lines) == result.n_commanded
    first = json.loads(lines[0])
    assert first["seq"] == 0
    assert first["phase"] == "pre_roll"
    # Monotonic seq is dense — no dropped samples.
    for i, ln in enumerate(lines):
        assert json.loads(ln)["seq"] == i


def test_reconstruct_body_velocities_matches_straight_line() -> None:
    """A straight-line pure-vx motion at 1 m/s should recover vx≈1, vy≈0, wz≈0."""
    from dimos.utils.characterization.scripts.analyze_run import (
        reconstruct_body_velocities,
    )

    ts = np.linspace(0, 2.0, 101)
    x = 1.0 * ts
    y = np.zeros_like(ts)
    yaw = np.zeros_like(ts)
    vx, vy, wz = reconstruct_body_velocities(ts, x, y, yaw)
    # Interior samples (skip edges — SavGol + gradient edge effects).
    assert np.allclose(vx[5:-5], 1.0, atol=1e-3)
    assert np.allclose(vy[5:-5], 0.0, atol=1e-3)
    assert np.allclose(wz[5:-5], 0.0, atol=1e-3)


def test_reconstruct_body_velocities_pure_rotation() -> None:
    """Pure rotation at 1 rad/s about origin: body vx=vy=0, wz=1."""
    from dimos.utils.characterization.scripts.analyze_run import (
        reconstruct_body_velocities,
    )

    ts = np.linspace(0, 2.0, 201)
    yaw = 1.0 * ts  # 1 rad/s
    x = np.zeros_like(ts)
    y = np.zeros_like(ts)
    vx, vy, wz = reconstruct_body_velocities(ts, x, y, yaw)
    assert np.allclose(wz[5:-5], 1.0, atol=1e-3)
    assert np.allclose(vx[5:-5], 0.0, atol=1e-3)
    assert np.allclose(vy[5:-5], 0.0, atol=1e-3)


def test_expand_plan_preserves_order_without_randomize() -> None:
    from dimos.utils.characterization.examples import ramp_vx_0_to_1p5, step_vx_1
    from dimos.utils.characterization.session import expand_plan

    plan = expand_plan([(step_vx_1, 2), (ramp_vx_0_to_1p5, 1)])
    assert [p.label for p in plan] == [
        "step_vx_1.0_r1of2",
        "step_vx_1.0_r2of2",
        "ramp_vx_0_to_1.5_r1of1",
    ]


def test_expand_plan_randomize_is_deterministic_with_seed() -> None:
    from dimos.utils.characterization.examples import step_vx_1, step_wz_1
    from dimos.utils.characterization.session import expand_plan

    entries = [(step_vx_1, 3), (step_wz_1, 2)]
    a = expand_plan(entries, randomize=True, rng_seed=42)
    b = expand_plan(entries, randomize=True, rng_seed=42)
    c = expand_plan(entries, randomize=True, rng_seed=43)
    assert [p.label for p in a] == [p.label for p in b]
    assert [p.label for p in a] != [p.label for p in c]
    assert len(a) == 5


def test_session_blueprint_composes_with_and_without_teleop(tmp_path: Path) -> None:
    from dimos.utils.characterization.session import build_session_blueprint
    from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

    db = tmp_path / "s.db"
    bp = build_session_blueprint(db, backend="go2", include_teleop=True)
    module_names = [a.module.__name__ for a in bp.blueprints]
    assert "KeyboardTeleop" in module_names
    teleop_atom = next(a for a in bp.blueprints if a.module is KeyboardTeleop)
    assert teleop_atom.kwargs.get("publish_only_when_active") is True

    bp2 = build_session_blueprint(db, backend="go2", include_teleop=False)
    module_names2 = [a.module.__name__ for a in bp2.blueprints]
    assert "KeyboardTeleop" not in module_names2


def test_run_json_session_fields_present_for_session_runs(tmp_path: Path) -> None:
    """Runner with session_db_path writes relative db pointer + ts_window_wall."""
    from dimos.utils.characterization.runner import CharacterizationSession

    session_db = tmp_path / "session" / "recording.db"
    session_db.parent.mkdir()
    run_dir = tmp_path / "session" / "000_pytest"
    run_dir.mkdir()

    recipe = TestRecipe(
        name="pytest_session",
        test_type="step",
        duration_s=0.2,
        signal_fn=step(0.3, "vx"),
        sample_rate_hz=50.0,
        pre_roll_s=0.1,
        post_roll_s=0.1,
    )
    with CharacterizationSession(
        cmd_vel_topic="/pytest_session_cmd_vel",
        output_root=tmp_path,
    ) as sess:
        result = sess.run(
            recipe,
            run_dir=run_dir,
            session_db_path=session_db,
            session_id="pytest_session_id",
        )

    assert result.exit_reason == "ok"
    meta = json.loads((run_dir / "run.json").read_text())
    assert meta["session_id"] == "pytest_session_id"
    # Stored as relative path — resolves to the session DB from the run dir.
    assert meta["session_db_path"] == "../recording.db"
    assert meta["recording_db"] is None
    window = meta["ts_window_wall"]
    assert window["end"] > window["start"]
    # Window is padded by 200 ms each side, so at least 0.4s even for a 0.4s recipe.
    assert (window["end"] - window["start"]) >= 0.4
