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

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.simulation.engines import mujoco_engine
from dimos.simulation.engines.mujoco_engine import CameraConfig, MujocoEngine

_DT = 0.002
_XML = f"""
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
    """Wall time that only moves when the loop sleeps, renders or steps."""

    def __init__(self, oversleep_s: float = 0.0) -> None:
        self.now = 1000.0
        self._oversleep_s = oversleep_s

    def time(self) -> float:
        return self.now

    def perf_counter(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds + self._oversleep_s


def _engine(tmp_path: Path, **kwargs: Any) -> MujocoEngine:
    xml = tmp_path / "scene.xml"
    xml.write_text(_XML)
    return MujocoEngine(config_path=xml, headless=True, **kwargs)


def _run(
    engine: MujocoEngine,
    monkeypatch: pytest.MonkeyPatch,
    *,
    wall_s: float,
    render_s: float = 0.0,
    fps: float = 15.0,
    step_s: float = 0.0,
    oversleep_s: float = 0.0,
) -> tuple[float, list[float]]:
    """Run the sim loop for ``wall_s`` of fake wall time.

    Each due camera render takes ``render_s``, each physics step ``step_s``
    and each sleep ``oversleep_s`` longer than asked. Returns the real-time
    factor and the pass start time of every render.
    """
    clock = _FakeClock(oversleep_s)
    monkeypatch.setattr(mujoco_engine, "time", clock)
    monkeypatch.setattr(engine, "_init_cameras", lambda: {})
    renders: list[float] = []

    def render(now: float, _: dict[str, Any]) -> None:
        if not renders or now - renders[-1] >= 1.0 / fps:
            renders.append(now)
            clock.now += render_s

    def physics(_: MujocoEngine) -> None:
        clock.now += step_s

    def stop_at_wall_s(e: MujocoEngine) -> None:
        if clock.now - start >= wall_s:
            e._stop_event.set()

    monkeypatch.setattr(engine, "_render_cameras", render)
    engine.set_step_hooks(before=physics, after=stop_at_wall_s)
    start = clock.now
    engine._sim_loop()
    return engine.data.time / (clock.now - start), renders


def test_slow_renders_do_not_hold_simulated_time_back(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # 50 ms per render at 15 fps, as software GL renders a 640x480 camera.
    rtf, renders = _run(_engine(tmp_path), monkeypatch, wall_s=10.0, render_s=0.05)

    assert rtf == pytest.approx(1.0, abs=0.01)
    assert len(renders) / 10.0 > 14.5


def test_late_wake_ups_are_made_up(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # Every sleep overshoots by 10% of a step, which held the sim at 0.9x real time.
    rtf, _ = _run(_engine(tmp_path), monkeypatch, wall_s=5.0, oversleep_s=0.1 * _DT)

    assert rtf == pytest.approx(1.0, abs=0.01)


def test_slow_physics_keeps_the_camera_rate_and_lets_the_backlog_go(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    logger = MagicMock()
    monkeypatch.setattr(mujoco_engine, "logger", logger)

    # Each 2 ms step costs 3 ms of wall time: no loop can keep real time.
    rtf, renders = _run(_engine(tmp_path), monkeypatch, wall_s=10.0, render_s=0.032, step_s=0.003)

    assert 0.3 < rtf < 0.5
    assert len(renders) / 10.0 > 14.5
    [warning] = [call.args[0] for call in logger.warning.call_args_list]
    assert "behind the wall clock" in warning


@pytest.mark.parametrize(("render_ms", "slow"), [(65.7, True), (1.2, False)])
def test_auto_shadows_go_off_when_shadowed_renders_are_slow(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, render_ms: float, slow: bool
) -> None:
    timed = MagicMock(return_value=render_ms)
    monkeypatch.setattr(mujoco_engine, "measure_render_ms", timed)
    engine = _engine(tmp_path)
    shadow_size = engine.model.vis.quality.shadowsize

    engine._settle_shadows([(CameraConfig(name="cam", fps=15.0), 0)])

    timed.assert_called_once()
    assert engine.model.vis.quality.shadowsize == (0 if slow else shadow_size)


@pytest.mark.parametrize(("shadows", "off"), [("on", False), ("off", True)])
def test_forced_shadows_time_no_render(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, shadows: str, off: bool
) -> None:
    timed = MagicMock(return_value=65.7)
    monkeypatch.setattr(mujoco_engine, "measure_render_ms", timed)
    engine = _engine(tmp_path, shadows=shadows)
    shadow_size = engine.model.vis.quality.shadowsize

    engine._settle_shadows([(CameraConfig(name="cam", fps=15.0), 0)])

    timed.assert_not_called()
    assert engine.model.vis.quality.shadowsize == (0 if off else shadow_size)


def test_auto_shadows_without_cameras_time_no_render(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    timed = MagicMock(return_value=65.7)
    monkeypatch.setattr(mujoco_engine, "measure_render_ms", timed)
    engine = _engine(tmp_path)
    shadow_size = engine.model.vis.quality.shadowsize

    engine._settle_shadows([])

    timed.assert_not_called()
    assert engine.model.vis.quality.shadowsize == shadow_size
