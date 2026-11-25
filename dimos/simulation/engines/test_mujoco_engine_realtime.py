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

from itertools import pairwise
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.simulation.engines import mujoco_engine, mujoco_sim_module
from dimos.simulation.engines.mujoco_engine import CameraConfig, MujocoEngine, _RealtimeKeeper
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

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
    step_s: float = 0.0,
    renders: list[float] | None = None,
) -> float:
    """Run the sim loop for ``wall_s`` of fake wall time and return its real-time factor.

    Each due render takes ``render_s`` and each physics step ``step_s``; the
    fake wall time of every render is appended to ``renders``.
    """
    clock = _FakeClock()
    monkeypatch.setattr(mujoco_engine, "time", clock)
    monkeypatch.setattr(engine, "_init_cameras", lambda: {})
    last = [float("-inf")]
    drawn = renders if renders is not None else []

    def slow_render(now: float, cam_renderers: dict[str, Any]) -> None:
        if now - last[0] >= 1.0 / fps:
            last[0] = now
            drawn.append(clock.now)
            clock.now += render_s

    def physics_cost(_: MujocoEngine) -> None:
        clock.now += step_s

    monkeypatch.setattr(engine, "_render_cameras", slow_render)
    start = clock.now
    engine.set_step_hooks(
        before=physics_cost,
        after=lambda e: e._stop_event.set() if clock.now - start >= wall_s else None,
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
    keeper.made(37)

    assert keeper.steps_due(0.0785) == 0
    assert keeper.made_up == 37


def test_keeper_lets_a_long_stall_go() -> None:
    keeper = _RealtimeKeeper(_DT, start=0.0, max_behind_s=0.5)

    # A 3 s stall: 0.5 s of it is due, the rest is let go.
    assert keeper.steps_due(3.0001) == 250
    assert keeper.skipped_s == pytest.approx(2.5, abs=0.005)


def test_keeper_asks_again_for_steps_a_short_burst_left() -> None:
    keeper = _RealtimeKeeper(_DT, start=0.0)
    assert keeper.steps_due(0.1001) == 49
    keeper.made(20)

    assert keeper.steps_due(0.1011) == 28


def test_slow_renders_hold_simulated_time_back_without_keep_realtime(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    engine = _engine(tmp_path, keep_realtime="off")

    rtf = _run_with_slow_renders(engine, monkeypatch, render_s=0.05, fps=15.0, wall_s=10.0)

    assert rtf < 0.5
    assert engine._realtime_keeper is None


def test_keep_realtime_follows_the_wall_clock_through_slow_renders(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    engine = _engine(tmp_path, keep_realtime="on")

    rtf = _run_with_slow_renders(engine, monkeypatch, render_s=0.05, fps=15.0, wall_s=10.0)

    assert rtf == pytest.approx(1.0, abs=0.02)
    assert engine._realtime_keeper is not None
    assert engine._realtime_keeper.made_up > 0


def test_slow_physics_keeps_the_camera_rate_and_lets_the_backlog_go(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Each 2 ms physics step costs 3 ms of wall time: no loop can keep real time.
    renders: list[float] = []
    engine = _engine(tmp_path, keep_realtime="on")

    rtf = _run_with_slow_renders(
        engine, monkeypatch, render_s=0.032, fps=15.0, wall_s=10.0, step_s=0.003, renders=renders
    )

    gaps = [later - earlier for earlier, later in pairwise(renders)]
    assert len(renders) / 10.0 > 11.0
    assert max(gaps) < 1.0 / 15.0 + mujoco_engine._MAX_CATCH_UP_S + 2 * 0.003
    assert 0.3 < rtf < 0.5
    keeper = engine._realtime_keeper
    assert keeper is not None
    assert keeper.skipped_s > 0.0


def test_a_stop_ends_a_catch_up_burst(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    clock = _FakeClock()
    monkeypatch.setattr(mujoco_engine, "time", clock)
    engine = _engine(tmp_path, keep_realtime="on")
    monkeypatch.setattr(engine, "_init_cameras", lambda: {})
    stalled: list[bool] = []

    def one_long_render(now: float, cam_renderers: dict[str, Any]) -> None:
        if not stalled:
            stalled.append(True)
            clock.now += 0.4  # 200 steps come due

    monkeypatch.setattr(engine, "_render_cameras", one_long_render)
    steps: list[int] = []

    def stop_after_ten(e: MujocoEngine) -> None:
        steps.append(1)
        if len(steps) == 10:
            e._stop_event.set()

    engine.set_step_hooks(after=stop_after_ten)
    engine._sim_loop()

    assert len(steps) == 10


def test_sim_module_hands_render_settings_to_the_engine(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    xml = tmp_path / "camera.xml"
    xml.write_text(_CAMERA_XML)
    seen: dict[str, Any] = {}

    class _BuiltError(Exception):
        pass

    def fake_engine(**kwargs: Any) -> MujocoEngine:
        seen.update(kwargs)
        raise _BuiltError

    monkeypatch.setattr(mujoco_sim_module, "MujocoEngine", fake_engine)
    module = MujocoSimModule(address=str(xml), camera_name="cam", shadows="off", keep_realtime="on")
    try:
        with pytest.raises(_BuiltError):
            module.start()
    finally:
        module.stop()

    assert seen["shadows"] == "off"
    assert seen["keep_realtime"] == "on"


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


@pytest.mark.parametrize(
    ("shadows", "keep_realtime", "says", "suggests"),
    [
        (
            "auto",
            "auto",
            "rendering without shadows, keeping the sim on the wall clock",
            "--mujocosimmodule.shadows on, --mujocosimmodule.keep-realtime off",
        ),
        (
            "on",
            "auto",
            "rendering with shadows (forced on), keeping the sim on the wall clock",
            "--mujocosimmodule.keep-realtime off",
        ),
        (
            "auto",
            "off",
            "rendering without shadows, letting the sim fall behind the wall clock",
            "--mujocosimmodule.shadows on",
        ),
        (
            "off",
            "auto",
            "rendering without shadows (forced off), keeping the sim on the wall clock",
            "--mujocosimmodule.keep-realtime off",
        ),
        (
            "auto",
            "on",
            "rendering without shadows, keeping the sim on the wall clock (forced on)",
            "--mujocosimmodule.shadows on",
        ),
    ],
)
def test_slow_render_warning_names_the_settings_applied(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    shadows: str,
    keep_realtime: str,
    says: str,
    suggests: str,
) -> None:
    monkeypatch.setattr(mujoco_engine, "render_ms", MagicMock(return_value=65.7))
    logger = MagicMock()
    monkeypatch.setattr(mujoco_engine, "logger", logger)
    engine = _engine(tmp_path, shadows=shadows, keep_realtime=keep_realtime)

    engine._settle_render_settings([(CameraConfig(name="cam", fps=15.0), 0)])

    [message] = [call.args[0] for call in logger.warning.call_args_list]
    assert says in message
    assert message.endswith(f"(force with {suggests})")
    assert (engine.model.vis.quality.shadowsize == 0) is (shadows != "on")
    assert engine._keep_realtime is (keep_realtime != "off")
