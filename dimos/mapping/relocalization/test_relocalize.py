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

"""Invariant tests for the fiducial prior and the per-prior accept path.

All inputs are CONSTRUCTED: each test injects a known transform or monkeypatches the
solve and asserts the compose / accept / reject logic directly.
"""

from __future__ import annotations

from typing import cast

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import pytest
from reactivex import Subject

import dimos.mapping.relocalization.module as module_mod
from dimos.mapping.relocalization.module import Config, RelocalizationModule
from dimos.mapping.relocalization.priors import (
    FiducialPrior,
    FiducialPriorConfig,
    RansacPrior,
    RansacPriorConfig,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _rot_z(deg: float) -> np.ndarray:
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rigid(yaw_deg: float, t: tuple[float, float, float]) -> np.ndarray:
    """A gravity-upright (yaw-only) 4x4 rigid transform."""
    T = np.eye(4)
    T[:3, :3] = _rot_z(yaw_deg)
    T[:3, 3] = t
    return T


def test_fiducial_composes_map_T_world_then_consumes_it_once() -> None:
    """One aggregated world_T_marker composes to map_T_world = map_T_marker @
    inv(world_T_marker), and the fix is proposed exactly once (consume-on-use)."""
    empty = o3d.geometry.PointCloud()
    map_T_marker = _rigid(30.0, (5.0, -2.0, 0.5))
    world_T_marker = _rigid(-40.0, (1.0, 0.5, 0.8))
    truth = map_T_marker @ np.linalg.inv(world_T_marker)

    prior = FiducialPrior({7: map_T_marker})
    assert prior.observe(7, world_T_marker) is None

    candidates = prior.propose(empty, empty)
    assert len(candidates) == 1 and candidates[0].source == "fiducial"
    np.testing.assert_allclose(candidates[0].T, truth, atol=1e-12)
    assert prior.propose(empty, empty) == []


# --- RelocalizationModule: bare shell (no coordinator, no start() wiring) -------


def _bare_module(config: Config) -> RelocalizationModule:
    """A RelocalizationModule with only the attributes the pure helpers read; mirrors
    __init__'s derived state. A test overrides _premap/_fiducial_prior as needed."""
    m = object.__new__(RelocalizationModule)
    m.config = config
    m._last_skip_log = 0.0
    ransac_entry = next(
        (p for p in config.priors if isinstance(p, RansacPriorConfig)), RansacPriorConfig()
    )
    m._ransac_prior = RansacPrior(interval_s=ransac_entry.interval_s)
    m._accept_threshold = {p.type: p.fitness_threshold for p in config.priors}
    m._ransac_min_local_points = ransac_entry.min_local_points
    m._premap = None
    m._fiducial_prior = None
    m._now_fn = lambda: 100.0  # driven clock; a trigger test swaps it
    m._world_to_map = Subject()
    m._last_fix_map_T_world = None  # no previous fix -> jump guard in acquisition
    m._last_fix_ts_s = 0.0
    m._last_local_map = None  # no cloud yet -> a burst cannot fire until one arrives
    return m


class _StubCloud:
    """Minimal PointCloud2 stand-in: __len__ plus a `.pointcloud` sentinel."""

    def __init__(self, n: int) -> None:
        self._n = n
        self.pointcloud = object()

    def __len__(self) -> int:
        return self._n


class _ModuleLogRecorder:
    """Captures module.py logger .info/.warning as (event, kwargs); the real logger
    sets propagate=False, so caplog can't see these and monkeypatching is needed."""

    def __init__(self) -> None:
        self.infos: list[tuple[str, dict[str, object]]] = []
        self.warnings: list[tuple[str, dict[str, object]]] = []

    def info(self, event: str, *args: object, **kwargs: object) -> None:
        self.infos.append((event, kwargs))

    def warning(self, event: str, *args: object, **kwargs: object) -> None:
        self.warnings.append((event, kwargs))

    def debug(self, event: str, *args: object, **kwargs: object) -> None:
        pass

    def exception(self, msg: str, *args: object, **kwargs: object) -> None:
        raise AssertionError(f"unexpected logger.exception: {msg}")


@pytest.mark.parametrize(
    "winning_source, fitness, accepted",
    [
        pytest.param("fiducial", 0.40, True, id="fiducial_clears_its_own_0.35_bar"),
        pytest.param("ransac", 0.50, False, id="ransac_fails_its_own_0.55_bar"),
    ],
)
def test_accept_gate_is_per_prior(
    monkeypatch: pytest.MonkeyPatch, winning_source: str, fitness: float, accepted: bool
) -> None:
    """The accept gate is PER-SOURCE: a fiducial fix at 0.40 clears its 0.35 bar while
    a ransac fix at 0.50 fails its 0.55 bar -- the same fitnesses flip under one gate."""
    m = _bare_module(
        Config(
            priors=[
                RansacPriorConfig(fitness_threshold=0.55),
                FiducialPriorConfig(fitness_threshold=0.35),
            ]
        )
    )
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._fiducial_prior = FiducialPrior({7: np.eye(4)})  # enables the fiducial entry
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(
        module_mod,
        "_relocalize_with_priors",
        lambda *a, **k: (_rigid(12.0, (0.4, -0.3, 0.02)), fitness, winning_source),
    )

    tf = m._try_relocalize(cast("PointCloud2", _StubCloud(1234)), m._enabled_prior_objects())

    if accepted:
        assert tf is not None
        assert rec.infos[0] == ("relocalize accepted", rec.infos[0][1])
        assert rec.infos[0][1]["source"] == winning_source and not rec.warnings
    else:
        assert tf is None
        assert rec.warnings[0] == (
            "relocalize rejected",
            {"source": "ransac", "fitness": 0.5, "threshold": 0.55},
        )


def _tracking_module(monkeypatch: pytest.MonkeyPatch, now: list[float]) -> RelocalizationModule:
    """A module that has already accepted one fix at the identity, so the next call is a
    tracking fix; the clock is driven, never slept, so the jump budgets are exact."""
    m = _bare_module(Config(priors=[RansacPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: now[0]
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())
    monkeypatch.setattr(
        module_mod, "_relocalize", lambda *a, **k: (_rigid(0.0, (0.0, 0.0, 0.0)), 0.9)
    )
    assert m._try_relocalize(cast("PointCloud2", _StubCloud(1234)), m._enabled_prior_objects())
    return m


@pytest.mark.parametrize(
    "seed_fix, dt_s, T, accepted",
    [
        pytest.param(True, 0.2, _rigid(180.0, (18.0, 0.0, 0.0)), False, id="gross_flip_rejected"),
        pytest.param(True, 2.0, _rigid(3.0, (0.4, 0.0, 0.0)), True, id="normal_drift_passes"),
        pytest.param(False, 0.0, _rigid(170.0, (40.0, -12.0, 0.0)), True, id="first_fix_exempt"),
    ],
)
def test_jump_guard(
    monkeypatch: pytest.MonkeyPatch, seed_fix: bool, dt_s: float, T: np.ndarray, accepted: bool
) -> None:
    """The tracking jump guard rejects a gross mis-localization, passes a normal
    sub-metre drift, and never blocks the first fix (acquisition is unguarded)."""
    now = [100.0]
    if seed_fix:
        m = _tracking_module(monkeypatch, now)
        now[0] = 100.0 + dt_s
    else:
        m = _bare_module(Config(priors=[RansacPriorConfig()]))
        m._premap = _StubCloud(10)  # type: ignore[assignment]
        m._now_fn = lambda: now[0]
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(module_mod, "_relocalize", lambda *a, **k: (T, 0.9))

    tf = m._try_relocalize(cast("PointCloud2", _StubCloud(1234)), m._enabled_prior_objects())

    if accepted:
        assert tf is not None and rec.infos[0][0] == "relocalize accepted" and not rec.warnings
    else:
        assert tf is None and rec.warnings[0][0] == "relocalize jump rejected"
