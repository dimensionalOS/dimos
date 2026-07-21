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

"""PGOVoxelMapper: scripted-PGO step units, no-loop over() synthetics, hk loop eval."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure.modules import pgo_mapper
from dimos.pure.modules.pgo_mapper import PGOVoxelMapper

if TYPE_CHECKING:
    from collections.abc import Iterator

VOXEL = 0.5


def _cloud(pts: list[list[float]], ts: float) -> PointCloud2:
    return PointCloud2.from_numpy(np.array(pts, dtype=np.float32), timestamp=ts)


def _pose(x: float, ts: float) -> Transform:
    """world <- sensor body pose at x; small yaw so it survives the placeholder filter."""
    tf = Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0998, 0.995),
        frame_id="world",
        child_frame_id="sensor",
        ts=ts,
    )
    tf.ts = ts  # ctor swaps an explicit ts=0.0 for wall clock; force the data ts
    return tf


def _centers(out: Any) -> np.ndarray:
    pts = out.global_map.points_f32()
    return pts[np.lexsort(pts.T[::-1])]


# ── unit: step against a scripted PGO core (no gtsam state, no engine) ────────


class _ScriptedPGO:
    """Stand-in for _PGOState: keyframe per process() call, loop at a chosen call."""

    loop_at: int = -1  # process() call index (1-based) that closes a loop
    shift: float = 0.0  # x-shift applied to every keyframe cloud once the loop lands
    last: _ScriptedPGO | None = None

    def __init__(self, cfg: Any) -> None:
        self.cfg = cfg
        self.clouds: list[PointCloud2] = []
        self.n_loops = 0
        self.calls = 0
        _ScriptedPGO.last = self

    @property
    def n_keyframes(self) -> int:
        return len(self.clouds)

    def process(self, pose: Any, ts: float, cloud: PointCloud2) -> None:
        self.calls += 1
        self.clouds.append(cloud)
        if self.calls == self.loop_at:
            self.n_loops += 1

    def correction(self, ts: float) -> Transform:
        tf = Transform(
            translation=Vector3(self.shift if self.n_loops else 0.0, 0.0, 0.0),
            frame_id="world_corrected",
            child_frame_id="world_raw",
            ts=ts,
        )
        tf.ts = ts
        return tf

    def keyframe_world_clouds(self, start: int = 0) -> Iterator[PointCloud2]:
        for c in self.clouds[start:]:
            yield c.transform(self.correction(c.ts)) if self.n_loops else c

    def snapshot(self) -> Any:
        from dimos.mapping.loop_closure.pgo import PoseGraph

        return PoseGraph()  # empty viz payloads; counts come from the attrs above


@pytest.fixture()
def scripted(monkeypatch: pytest.MonkeyPatch) -> type[_ScriptedPGO]:
    monkeypatch.setattr(pgo_mapper, "_PGOState", _ScriptedPGO)
    _ScriptedPGO.loop_at = -1
    _ScriptedPGO.shift = 0.0
    return _ScriptedPGO


def test_step_maps_keyframes_only(scripted: type[_ScriptedPGO]) -> None:
    # every process() call keyframes here, so each scan's cloud lands in the grid
    m = PGOVoxelMapper(voxel_size=VOXEL, emit_every=2)
    s = PGOVoxelMapper.State()
    s, out = m.step(
        s, PGOVoxelMapper.In(ts=1.0, scan=_cloud([[0.1, 0.1, 0.1]], 1.0), pose=_pose(0.1, 1.0))
    )
    assert out is None and s.n_scans == 1  # below cadence
    s, out = m.step(
        s, PGOVoxelMapper.In(ts=2.0, scan=_cloud([[5.0, 0.0, 0.0]], 2.0), pose=_pose(5.0, 2.0))
    )
    assert out is not None and out.n_keyframes == 2 and out.n_loops == 0
    np.testing.assert_allclose(_centers(out), [[0.25, 0.25, 0.25], [5.25, 0.25, 0.25]], atol=1e-6)


def test_step_loop_closure_rebuilds_and_emits_immediately(
    scripted: type[_ScriptedPGO],
) -> None:
    scripted.loop_at = 3
    scripted.shift = 10.0
    m = PGOVoxelMapper(voxel_size=VOXEL, emit_every=0)  # cadence off: only the loop emits
    s = PGOVoxelMapper.State()
    outs = []
    for i in range(4):
        # x starts at 2.0: an x=0 pose would hit the zero-translation placeholder filter
        scan = _cloud([[(i + 1) * 2.0, 0.0, 0.0]], float(i + 1))
        s, out = m.step(
            s,
            PGOVoxelMapper.In(ts=float(i + 1), scan=scan, pose=_pose((i + 1) * 2.0, float(i + 1))),
        )
        outs.append(out)
    assert [o is not None for o in outs] == [False, False, True, False]  # loop tick only
    loop_out = outs[2]
    assert loop_out is not None and loop_out.n_loops == 1
    assert loop_out.correction.translation.x == pytest.approx(10.0)
    # the rebuild re-registered every keyframe cloud through the loop correction
    np.testing.assert_allclose(
        _centers(loop_out),
        [[12.25, 0.25, 0.25], [14.25, 0.25, 0.25], [16.25, 0.25, 0.25]],
        atol=1e-5,
    )


def test_step_placeholder_pose_skips_pgo(scripted: type[_ScriptedPGO]) -> None:
    m = PGOVoxelMapper(voxel_size=VOXEL, emit_every=1)
    s = PGOVoxelMapper.State()
    zero = Transform(frame_id="world", child_frame_id="sensor", ts=1.0)
    zero.ts = 1.0
    s, out = m.step(s, PGOVoxelMapper.In(ts=1.0, scan=_cloud([[0.1, 0.1, 0.1]], 1.0), pose=zero))
    assert scripted.last is not None and scripted.last.calls == 0  # filtered
    assert out is not None and out.n_keyframes == 0 and out.n_voxels == 0  # empty map


def test_finish_flushes_final_map(scripted: type[_ScriptedPGO]) -> None:
    m = PGOVoxelMapper(voxel_size=VOXEL, emit_every=0)  # cadence off: only finish emits
    s = PGOVoxelMapper.State()
    for i in range(3):
        scan = _cloud([[(i + 1) * 2.0, 0.0, 0.0]], float(i + 1))
        s, out = m.step(
            s,
            PGOVoxelMapper.In(ts=float(i + 1), scan=scan, pose=_pose((i + 1) * 2.0, float(i + 1))),
        )
        assert out is None  # emit_every=0 → the step never emits
    tail = m.finish(s)
    assert tail is not None and tail.n_scans == 3 and tail.n_keyframes == 3 and tail.n_voxels == 3
    assert tail.correction is not None  # a final corrected-frame transform is present
    # nothing mapped → no final map to flush
    assert PGOVoxelMapper(emit_every=0).finish(PGOVoxelMapper.State()) is None


# ── engine: over() with the real PGO core, no loops on a straight line ────────


def test_over_no_loop_keyframe_map() -> None:
    # 2 m strides: every scan keyframes, none revisits — the map is exactly the
    # keyframe clouds re-registered through identity (up to submap downsampling).
    scans = [_cloud([[i * 2.0, 0.0, 0.0]], float(i + 1)) for i in range(6)]
    tfs = [_pose(i * 2.0, float(i)) for i in range(8)]  # brackets every tick
    m = PGOVoxelMapper(voxel_size=VOXEL, emit_every=2)
    rows = list(m.over(scan=scans, tf=tfs))
    assert [r.n_scans for r in rows] == [2, 4, 6]
    assert all(r.n_loops == 0 for r in rows)
    assert rows[-1].n_keyframes == 6
    assert rows[-1].n_voxels == 6
    assert float(np.abs(rows[-1].correction.translation.to_numpy()).max()) < 1e-2
    assert rows[-1].global_map.frame_id == "world"


def test_over_finish_emits_final_map() -> None:
    # emit_every=4 over 6 scans: step emits at 4, finish() flushes the final map at 6
    scans = [_cloud([[i * 2.0, 0.0, 0.0]], float(i + 1)) for i in range(6)]
    tfs = [_pose(i * 2.0, float(i)) for i in range(8)]  # brackets every tick
    m = PGOVoxelMapper(voxel_size=VOXEL, emit_every=4)
    rows = list(m.over(scan=scans, tf=tfs))
    assert [r.n_scans for r in rows] == [4, 6]  # cadence 4 + finish tail 6
    tail = rows[-1]
    assert tail.n_keyframes == 6 and tail.n_loops == 0 and tail.n_voxels == 6
    assert tail.correction is not None  # a final corrected-frame transform
    assert tail.keyframes is not None and len(tail.keyframes.nodes) == 6  # viz rebuilt at the tail
    assert tail.global_map.frame_id == "world"


# ── real data: go2_hongkong_office closes loops (skip-guarded, self-hosted) ───


def _get_data(name: str) -> Any:
    from dimos.utils.data import get_data

    try:
        return get_data(name)
    except Exception as exc:  # LFS unavailable, offline, ... — skip, don't fail
        pytest.skip(f"dataset {name} unavailable: {exc}")


@pytest.mark.self_hosted
def test_hk_office_closes_loops_and_matches_reference() -> None:
    """First 600 scans: 2 loop closures by scan ~561 (~10 s CPU); deselected by default."""
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    path = _get_data("go2_hongkong_office.db")
    reference = PointCloud2.lcm_decode(
        _get_data("go2_hongkong_office_twopass_map.pc2.lcm").read_bytes()
    )
    with SqliteStore(path=str(path)) as store:
        scans = [o.data for o in store.stream("lidar", PointCloud2).range_seek(0, 600)]
        tfs = []
        for o in store.stream("odom", PoseStamped):
            p = o.data
            tf = Transform(
                translation=p.position,
                rotation=p.orientation,
                frame_id="world",
                child_frame_id="sensor",
                ts=o.ts,
            )
            tf.ts = o.ts
            tfs.append(tf)
        m = PGOVoxelMapper(voxel_size=0.1, emit_every=100)
        rows = list(m.over(scan=scans, tf=tfs))

    final = rows[-1]
    assert final.n_loops >= 2  # measured: loops at scans 523 and 561
    assert final.n_keyframes > 100
    loop_rows = [r for r in rows if r.n_loops > 0 and r.n_scans % 100 != 0]
    assert loop_rows  # closures emit immediately, off the cadence grid
    # keyframe map vs the offline two-pass PGO reference; measured 0.884 / 0.965
    d = np.asarray(final.global_map.pointcloud.compute_point_cloud_distance(reference.pointcloud))
    assert (d <= 0.1).mean() >= 0.75
    assert (d <= 0.2).mean() >= 0.9
