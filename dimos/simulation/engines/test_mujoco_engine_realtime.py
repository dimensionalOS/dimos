# Copyright 2026 Dimensional Inc.
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

"""Keeping simulated time on the wall clock when camera renders are slow."""

from __future__ import annotations

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.simulation.engines import mujoco_engine
from dimos.simulation.engines.mujoco_engine import CameraConfig, MujocoEngine, _RealtimeKeeper

_DT = 0.002
_CAMERA_XML = f"""
<mujoco>
  <option timestep="{_DT}"/>
  <worldbody>
    <light directional="true" pos="0 0 3" dir="0 0 -1"/>
    <camera name="cam" pos="0 0 1"/>
    <body pos="0 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body>
  </worldbody>
</mujoco>
"""


class _FakeClock:
    """Wall time that only moves when the loop sleeps or a fake render takes time."""

    def __init__(self) -> None:
        self.now = 1000.0

    def time(self) -> float:
        return self.now

    def perf_counter(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(seconds, 0.0)


def _engine(tmp_path: Path, **kwargs: Any) -> MujocoEngine:
    xml = tmp_path / "camera.xml"
    xml.write_text(_CAMERA_XML)
    return MujocoEngine(config_path=xml, headless=True, **kwargs)


def _run_with_slow_renders(
    engine: MujocoEngine,
    monkeypatch: pytest.MonkeyPatch,
    *,
    render_s: float,
    fps: float,
    wall_s: float,
) -> float:
    """Run the sim loop for ``wall_s`` of fake wall time; each due render takes ``render_s``."""
    clock = _FakeClock()
    monkeypatch.setattr(mujoco_engine, "time", clock)
    monkeypatch.setattr(engine, "_init_cameras", lambda: {})
    last = [float("-inf")]

    def slow_render(now: float, cam_renderers: dict[str, Any]) -> None:
        if now - last[0] >= 1.0 / fps:
            last[0] = now
            clock.now += render_s

    monkeypatch.setattr(engine, "_render_cameras", slow_render)
    start = clock.now
    engine.set_step_hooks(
        after=lambda e: e._stop_event.set() if clock.now - start >= wall_s else None
    )
    engine._sim_loop()
    return engine.data.time / (clock.now - start)


def test_keeper_asks_for_nothing_while_the_loop_is_on_time() -> None:
    keeper = _RealtimeKeeper(_DT, start=0.0)

    due = [keeper.steps_due(step * _DT) for step in range(100)]

    assert due == [0] * 100
    assert keeper.made_up == 0


def test_keeper_makes_up_a_render_that_held_the_loop() -> None:
    keeper = _RealtimeKeeper(_DT, start=0.0)

    # The pass made its one step, then a render held it 76.5 ms: 38 steps were due.
    assert keeper.steps_due(0.0765) == 37
    assert keeper.made_up == 37


def test_keeper_caps_one_pass_and_lets_a_long_stall_go() -> None:
    keeper = _RealtimeKeeper(_DT, start=0.0, max_behind_s=0.5, max_steps=100)
    assert keeper.steps_due(0.1001) == 49

    # A 3 s stall: 0.5 s of it is made up, 100 steps per pass; the rest is let go.
    after_stall = [keeper.steps_due(3.1001 + pass_ * 0.001) for pass_ in range(4)]

    assert after_stall == [100, 100, 49, 0]
    assert keeper.skipped_s == pytest.approx(2.5, abs=0.005)


def test_slow_renders_hold_simulated_time_back_without_keep_realtime(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    engine = _engine(tmp_path, keep_realtime="off")

    rtf = _run_with_slow_renders(engine, monkeypatch, render_s=0.05, fps=15.0, wall_s=2.0)

    assert rtf < 0.5
    assert engine._realtime_keeper is None


def test_keep_realtime_follows_the_wall_clock_through_slow_renders(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    engine = _engine(tmp_path, keep_realtime="on")

    rtf = _run_with_slow_renders(engine, monkeypatch, render_s=0.05, fps=15.0, wall_s=2.0)

    assert rtf == pytest.approx(1.0, abs=0.02)
    assert engine._realtime_keeper is not None
    assert engine._realtime_keeper.made_up > 0


@pytest.mark.parametrize(
    ("render_ms", "slow"),
    [(65.7, True), (1.2, False)],  # Mesa llvmpipe and an RTX 5070, xArm7 wrist camera at 15 fps
)
def test_auto_settles_from_one_timed_render(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, render_ms: float, slow: bool
) -> None:
    timed = MagicMock(return_value=render_ms)
    monkeypatch.setattr(mujoco_engine, "render_ms", timed)
    engine = _engine(tmp_path)
    shadow_size = engine.model.vis.quality.shadowsize

    engine._settle_render_settings([(CameraConfig(name="cam", fps=15.0), 0)])

    timed.assert_called_once()
    assert engine._keep_realtime is slow
    assert engine.model.vis.quality.shadowsize == (0 if slow else shadow_size)


def test_forced_settings_time_no_render(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    timed = MagicMock(return_value=65.7)
    monkeypatch.setattr(mujoco_engine, "render_ms", timed)
    engine = _engine(tmp_path, shadows="off", keep_realtime="on")

    engine._settle_render_settings([(CameraConfig(name="cam", fps=15.0), 0)])

    timed.assert_not_called()
    assert engine._keep_realtime is True
    assert engine.model.vis.quality.shadowsize == 0


def test_without_cameras_auto_keeps_the_stock_loop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    timed = MagicMock(return_value=65.7)
    monkeypatch.setattr(mujoco_engine, "render_ms", timed)
    engine = _engine(tmp_path)
    shadow_size = engine.model.vis.quality.shadowsize

    engine._settle_render_settings([])

    timed.assert_not_called()
    assert engine._keep_realtime is False
    assert engine.model.vis.quality.shadowsize == shadow_size
