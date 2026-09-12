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

import json
from pathlib import Path
import struct
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

pytest.importorskip("dimos_voxel_ray_tracing")

from dimos.teleop.memory_world import replay as replay_module
from dimos.teleop.memory_world.replay import (
    TAG_ADDED,
    TAG_REMOVED,
    RayTracedGrid,
    SensorScan,
    VoxelReplay,
    build_replay_streams,
    pack_keys,
    sensor_scan,
    unpack_centres,
)
from dimos.teleop.memory_world.tf_tree import pose_matrix

VOXEL = 0.1
AT_ORIGIN = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _scans(seed: int, count: int, points: int = 300) -> list[np.ndarray]:
    """Random walls that drift a little, so scans overlap yet keep changing."""
    rng = np.random.default_rng(seed)
    scans = []
    for i in range(count):
        base = rng.uniform(-2, 2, size=(points, 3)).astype(np.float32)
        base[:, 2] = np.abs(base[:, 2])  # above the floor
        base[:, 0] += 0.02 * i
        scans.append(base)
    return scans


def _wall(x: float) -> np.ndarray:
    """A wall at x facing the origin, every voxel of it hit: (N, 3) float32.

    Walls at different x subtend the same angles from the origin, so rays to
    a farther wall pass through every voxel of a nearer one.
    """
    half = x / 3.0
    side = np.arange(-half, half, VOXEL / 2, dtype=np.float32) + VOXEL / 4
    y, z = np.meshgrid(side, side + half)
    return np.stack([np.full(y.size, x, np.float32), y.ravel(), z.ravel()], axis=1)


def test_keys_round_trip_through_centres() -> None:
    points = np.array([[0.31, -0.29, 1.04], [0.31, -0.29, 1.09], [5.0, 5.0, 5.0]], np.float32)
    keys = pack_keys(points, VOXEL)
    assert len(keys) == 2  # the first two share a voxel
    np.testing.assert_allclose(
        unpack_centres(keys, VOXEL), [[0.35, -0.25, 1.05], [5.05, 5.05, 5.05]]
    )


def test_sensor_scan_moves_a_world_scan_back_to_the_sensor() -> None:
    world_from_sensor = pose_matrix((10.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    scan = sensor_scan(np.array([[12.0, 1.0, 0.5]], np.float32), world_from_sensor, in_world=True)
    np.testing.assert_allclose(scan.points, [[2.0, 1.0, 0.5]])
    assert scan.position == (10.0, 0.0, 0.0)
    kept = sensor_scan(np.array([[2.0, 1.0, 0.5]], np.float32), world_from_sensor, in_world=False)
    np.testing.assert_allclose(kept.points, [[2.0, 1.0, 0.5]])


def test_diffs_replay_to_the_same_set() -> None:
    """Applying every (added, removed) pair from empty reproduces the grid."""
    grid = RayTracedGrid(VOXEL, max_range=10.0)
    state: set[int] = set()
    for scan in _scans(2, 10):
        added, removed = grid.add_scan(SensorScan(scan, *AT_ORIGIN))
        assert not (set(added.tolist()) & set(removed.tolist()))
        state -= set(removed.tolist())
        state |= set(added.tolist())
        assert state == set(grid.keys.tolist())
    assert state


def test_a_wall_seen_through_is_cleared() -> None:
    """Column carving would keep a wall that later scans see straight past;
    ray tracing removes it, and never touches voxels beyond the ray range."""
    grid = RayTracedGrid(VOXEL, max_range=10.0)
    near = _wall(3.0)
    for _ in range(8):
        grid.add_scan(SensorScan(near, *AT_ORIGIN))
    near_keys = set(pack_keys(near, VOXEL).tolist())
    held = set(grid.keys.tolist())
    assert len(near_keys & held) >= 0.9 * len(near_keys)  # the head-on wall is in the map

    far = _wall(6.0)
    removed_total: set[int] = set()
    for _ in range(12):
        _, removed = grid.add_scan(SensorScan(far, *AT_ORIGIN))
        removed_total |= set(removed.tolist())
    assert near_keys & removed_total  # the old wall came out
    assert not (near_keys & set(grid.keys.tolist()))


@pytest.fixture
def store(tmp_path):  # type: ignore[no-untyped-def]
    store = SqliteStore(path=str(tmp_path / "replay.db"))
    lidar = store.stream("lidar", PointCloud2)
    for i, scan in enumerate(_scans(4, 30)):
        ts = 100.0 + i * 0.1
        lidar.append(PointCloud2.from_numpy(scan, frame_id="world", timestamp=ts), ts=ts)
    yield store
    store.stop()


def test_replay_built_in_one_world_frame_serves_only_that_one(store) -> None:  # type: ignore[no-untyped-def]
    build_replay_streams(
        store,
        lidar_stream_name="lidar",
        to_scan=lambda obs: SensorScan(obs.data.points_f32(), *AT_ORIGIN),
        voxel_size=VOXEL,
        max_range=10.0,
        keyframe_interval_s=1.0,
        world_frame="odom",
    )
    assert VoxelReplay.available(
        store, voxel_size=VOXEL, lidar_stream_name="lidar", world_frame="odom"
    )
    assert VoxelReplay.available(
        store, voxel_size=VOXEL, lidar_stream_name="lidar"
    )  # no preference
    assert not VoxelReplay.available(
        store, voxel_size=VOXEL, lidar_stream_name="lidar", world_frame="map"
    )


def test_a_changed_keyframe_interval_is_a_rebuild_like_its_four_siblings(store) -> None:  # type: ignore[no-untyped-def]
    """The spacing is baked in at build time, so serving cannot change it.

    voxel_size, max_range, lidar_stream and world_frame all mark the streams stale when
    they change; keyframe_interval_s was written into the tags and then never read, so
    `--replay-keyframe-interval-s` had NO effect on a recording that was already built.
    No rebuild, no log line, and the streams kept the spacing of whichever run built them
    first -- the operator's setting simply did nothing, and nothing said so.
    """
    build_replay_streams(
        store,
        lidar_stream_name="lidar",
        to_scan=lambda obs: SensorScan(obs.data.points_f32(), *AT_ORIGIN),
        voxel_size=VOXEL,
        max_range=10.0,
        keyframe_interval_s=1.0,
    )
    same = VoxelReplay.available(
        store, voxel_size=VOXEL, lidar_stream_name="lidar", keyframe_interval_s=1.0
    )
    assert same, "the interval it was built with must not force a rebuild"
    assert VoxelReplay.available(store, voxel_size=VOXEL, lidar_stream_name="lidar"), (
        "no preference must still fit, as it does for world_frame"
    )
    assert not VoxelReplay.available(
        store, voxel_size=VOXEL, lidar_stream_name="lidar", keyframe_interval_s=999.0
    ), "a different interval cannot be served from these streams, so it is a rebuild"


def _sensor_frame_recording(path: Path, tf_child: str) -> None:
    """Three scans in the 'lidar' frame and a tf stream odom -> *tf_child* at their stamps."""
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

    store = SqliteStore(path=str(path))
    store.start()
    lidar, tf = store.stream("lidar", PointCloud2), store.stream("tf", TFMessage)
    for ts in (1.0, 2.0, 3.0):
        points = np.array([[2.0, 0.0, 0.0], [0.0, 2.0, 0.0]], np.float32)
        lidar.append(PointCloud2.from_numpy(points, frame_id="lidar", timestamp=ts), ts=ts)
        tf.append(
            TFMessage(
                Transform(
                    translation=Vector3(ts, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="odom",
                    child_frame_id=tf_child,
                    ts=ts,
                )
            ),
            ts=ts,
        )
    store.stop()


def test_cli_falls_back_to_the_tf_root_and_refuses_an_empty_build(tmp_path: Path, capsys) -> None:  # type: ignore[no-untyped-def]
    _sensor_frame_recording(tmp_path / "placed.db", "lidar")
    replay_module.main([str(tmp_path / "placed.db"), "--world-frame", "world", "--dry-run"])
    out = capsys.readouterr().out
    assert "using its root 'odom'" in out and "3 scans" in out
    _sensor_frame_recording(tmp_path / "unplaced.db", "base_link")  # tf never reaches 'lidar'
    with pytest.raises(SystemExit, match="no voxel came out"):
        replay_module.main([str(tmp_path / "unplaced.db"), "--world-frame", "odom", "--dry-run"])
    with pytest.raises(SystemExit, match="no voxel came out"):  # written, then taken back
        replay_module.main([str(tmp_path / "unplaced.db"), "--world-frame", "odom"])
    store = SqliteStore(path=str(tmp_path / "unplaced.db"))
    assert not {"voxel_diff", "voxel_keyframe"} & set(store.list_streams())
    store.stop()


def test_cli_refuses_scans_already_in_the_world_frame(tmp_path: Path) -> None:
    store = SqliteStore(path=str(tmp_path / "aligned.db"))
    store.start()
    store.stream("lidar", PointCloud2).append(
        PointCloud2.from_numpy(np.zeros((1, 3), np.float32), frame_id="odom", timestamp=1.0), ts=1.0
    )
    store.stop()
    with pytest.raises(SystemExit, match="sensor pose"):
        replay_module.main([str(tmp_path / "aligned.db"), "--world-frame", "odom", "--dry-run"])
    store = SqliteStore(path=str(tmp_path / "stitched.db"))
    store.start()
    store.stream("lidar", PointCloud2).append(
        PointCloud2.from_numpy(
            np.zeros((1, 3), np.float32), frame_id="corrected_odom", timestamp=1.0
        ),
        ts=1.0,
    )
    store.stop()
    with pytest.raises(SystemExit, match="corrected_odom"):  # a stitched frame, any world name
        replay_module.main([str(tmp_path / "stitched.db"), "--world-frame", "map", "--dry-run"])


def test_build_streams_and_serve_segments(store) -> None:  # type: ignore[no-untyped-def]
    stats = build_replay_streams(
        store,
        lidar_stream_name="lidar",
        to_scan=lambda obs: SensorScan(obs.data.points_f32(), *AT_ORIGIN),
        voxel_size=VOXEL,
        max_range=10.0,
        keyframe_interval_s=1.0,
    )
    assert stats.scans == 30
    assert stats.keyframes == 4  # t=100.0, 101.0, 102.0 and the last scan
    assert VoxelReplay.available(store, voxel_size=VOXEL, lidar_stream_name="lidar")
    assert not VoxelReplay.available(store, voxel_size=VOXEL * 2, lidar_stream_name="lidar")
    assert not VoxelReplay.available(store, voxel_size=VOXEL, lidar_stream_name="other_lidar")
    # Built without a frame: accepted in any frame (built in one: the test below).
    assert VoxelReplay.available(
        store, voxel_size=VOXEL, lidar_stream_name="lidar", world_frame="map"
    )

    diffs = list(store.streams["voxel_diff"])
    assert len(diffs) == 30
    tags = diffs[5].data.tags_u8()
    assert set(tags.tolist()) <= {TAG_ADDED, TAG_REMOVED}

    replay = VoxelReplay(store)
    index = replay.index
    assert index.scan_at(100.55) == 5
    assert index.segment_of(5) == 0 and index.segment_of(10) == 1 and index.segment_of(29) == 3
    assert index.segment_scans(1) == (10, 20)

    # replaying a segment's diffs on the viewer's terms reaches the next keyframe
    header, payload = replay.segment(1)
    raw = VoxelReplay.encode_segment(header, payload)
    header_length = struct.unpack("<I", raw[:4])[0]
    parsed = json.loads(raw[4 : 4 + header_length])
    assert parsed == header
    assert (4 + header_length) % 4 == 0
    body = raw[4 + header_length :]
    table = np.frombuffer(body[: header["slots"] * 6], dtype="<i2").reshape(-1, 3)
    entries = sum(scan["n"] for scan in header["scans"])
    slots = np.frombuffer(body[header["slots_offset"] :][: entries * 4], dtype="<u4")
    ops = np.frombuffer(body[header["slots_offset"] + entries * 4 :][:entries], dtype=np.uint8)

    visible = np.zeros(header["slots"], dtype=bool)
    visible[: header["keyframe"]["n"]] = True
    cursor = 0
    for scan in header["scans"]:
        if scan["index"] > header["keyframe"]["scan"]:
            for e in range(cursor, cursor + scan["n"]):
                visible[slots[e]] = ops[e] == TAG_ADDED
        cursor += scan["n"]
    shown = {tuple(int(v) for v in row) for row in table[visible] + np.asarray(index.origin)}

    def voxels(points: np.ndarray) -> set[tuple[int, ...]]:
        return {tuple(int(v) for v in row) for row in np.floor(points / VOXEL).astype(int)}

    # the state after the last scan of segment 1 (scan 19) is one scan before keyframe 2 (scan 20)
    keyframe2 = store.streams["voxel_keyframe"].at(102.0, tolerance=1e-3).first().data.points_f32()
    scan20 = diffs[20].data
    tags20 = scan20.tags_u8()
    points20 = scan20.points_f32()
    expected = voxels(keyframe2) - voxels(points20[tags20 == TAG_ADDED]) | voxels(
        points20[tags20 == TAG_REMOVED]
    )
    assert shown == expected


def test_build_cut_short_is_not_available(store) -> None:  # type: ignore[no-untyped-def]
    build_replay_streams(
        store,
        lidar_stream_name="lidar",
        to_scan=lambda obs: SensorScan(obs.data.points_f32(), *AT_ORIGIN),
        voxel_size=VOXEL,
        max_range=10.0,
        keyframe_interval_s=1.0,
    )
    # a build that died before the last scan never wrote the keyframe marked last
    kept = [(obs.data, obs.ts, obs.tags) for obs in store.streams["voxel_keyframe"]][:-1]
    store.delete_stream("voxel_keyframe")
    keyframes = store.stream("voxel_keyframe", PointCloud2)
    for data, ts, tags in kept:
        keyframes.append(data, ts=ts, tags=tags)
    assert not VoxelReplay.available(store, voxel_size=VOXEL, lidar_stream_name="lidar")
    # ... and one that died before its first keyframe left the streams empty
    store.delete_stream("voxel_keyframe")
    store.stream("voxel_keyframe", PointCloud2)
    assert not VoxelReplay.available(store, voxel_size=VOXEL, lidar_stream_name="lidar")


def test_wire_cache_is_bounded(store, monkeypatch) -> None:  # type: ignore[no-untyped-def]
    build_replay_streams(
        store,
        lidar_stream_name="lidar",
        to_scan=lambda obs: SensorScan(obs.data.points_f32(), *AT_ORIGIN),
        voxel_size=VOXEL,
        max_range=10.0,
        keyframe_interval_s=1.0,
    )
    monkeypatch.setattr(replay_module, "WIRE_CACHE_BYTES", 1)
    replay = VoxelReplay(store)
    first = replay.encoded_segment(0)
    replay.encoded_segment(1)
    assert list(replay._wire) == [1]  # the older one made room
    assert replay.encoded_segment(0) == first  # rebuilt, byte for byte


def test_keyframes_are_found_by_scan_index_when_stamps_repeat(tmp_path: Path) -> None:
    """Two scans stamped alike: the interval keyframe and the forced final one share a
    stamp, so the final map and each segment's keyframe go by scan index."""
    store = SqliteStore(path=str(tmp_path / "dup.db"))
    store.start()
    lidar = store.stream("lidar", PointCloud2)
    for ts, x in ((100.0, 1.0), (105.0, 2.0), (105.0, 3.0)):
        lidar.append(
            PointCloud2.from_numpy(np.array([[x, 0.0, 0.0]]), frame_id="lidar", timestamp=ts),
            ts=ts,
        )
    try:
        stats = build_replay_streams(
            store,
            lidar_stream_name="lidar",
            to_scan=lambda obs: SensorScan(obs.data.points_f32(), *AT_ORIGIN),
            voxel_size=VOXEL,
            max_range=10.0,
            keyframe_interval_s=5.0,
        )
        assert stats.keyframes == 3
        assert VoxelReplay.available(store, voxel_size=VOXEL, lidar_stream_name="lidar")
        replay = VoxelReplay(store)
        assert int(replay.final_keyframe().tags["scan_index"]) == 2
        header, _ = replay.segment(2)
        assert header["keyframe"]["scan"] == 2
    finally:
        store.stop()


def test_a_gap_before_the_first_known_position_is_not_the_world_origin() -> None:
    """The same array is the path a route is planned over, so an invented pose is a route.

    Reporting the origin for stamps before the first successful lookup puts the robot
    somewhere it has never been, indistinguishable from somewhere it has, and the
    costmap planner then treats that straight line through unmapped space as passable.
    """
    from dimos.teleop.memory_world.replay import frame_positions, stamped_positions

    def pose_at(ts: float) -> np.ndarray | None:
        if ts < 3.0:
            return None  # tf cannot place the frame yet
        matrix = np.eye(4)
        matrix[:3, 3] = [10.0 + ts, 20.0, 0.0]
        return matrix

    positions = frame_positions([0.0, 1.0, 2.0, 3.0, 4.0], pose_at)
    assert positions[0] == positions[1] == positions[2] == [13.0, 20.0, 0.0]  # the first known
    assert positions[3] == [13.0, 20.0, 0.0] and positions[4] == [14.0, 20.0, 0.0]
    assert [0.0, 0.0, 0.0] not in positions  # never a place the robot was not

    # A gap AFTER a known position still holds that one, and the same rule applies to
    # the poses stamped on observations.
    held = frame_positions([3.0, 9.9, 4.0], lambda ts: None if ts > 9 else pose_at(ts))
    assert held == [[13.0, 20.0, 0.0], [13.0, 20.0, 0.0], [14.0, 20.0, 0.0]]
    stamped = stamped_positions(
        [
            SimpleNamespace(pose_tuple=None),
            SimpleNamespace(pose_tuple=(5.0, 6.0, 7.0)),
            SimpleNamespace(pose_tuple=None),
        ]
    )
    assert stamped == [[5.0, 6.0, 7.0]] * 3


def test_a_voxel_already_held_is_never_inserted_twice() -> None:
    """A duplicate key survives every later clear: a ghost voxel nothing can remove.

    The "is it near?" test here and the mapper's own test run over different float32
    centres, so a voxel on the boundary can be reported as newly seen while already
    being held. Two copies then break the uniqueness the removal path relies on. Driven
    through the real add_scan with a mapper that reports exactly that boundary voxel.
    """
    from dimos.teleop.memory_world.replay import RayTracedGrid, SensorScan, unpack_centres

    class BoundaryMapper:
        """Reports one voxel just outside this grid's own cylinder test, then inside it."""

        def __init__(self, centre: np.ndarray) -> None:
            self.centre = centre

        def add_frame(self, *_: object) -> None:
            return None

        def local_map(self, *_: object, **__: object) -> np.ndarray:
            return self.centre.reshape(1, 3)

    grid = RayTracedGrid.__new__(RayTracedGrid)
    grid.voxel_size = 0.1
    grid.max_range = 5.0
    grid.keys = np.empty(0, dtype=np.int64)
    grid._centres = np.empty((0, 3), dtype=np.float32)
    # Exactly max_range away in x, where float32 rounding decides the cylinder test.
    centre = np.array([5.0, 0.0, 0.0], dtype=np.float32)
    grid.mapper = BoundaryMapper(centre)
    scan = SensorScan(points=np.zeros((1, 3), np.float32), position=np.zeros(3), orientation=None)

    for _ in range(4):  # the same voxel, seen again and again
        grid.add_scan(scan)
    assert len(grid.keys) == len(set(grid.keys.tolist())), "a key was inserted twice"
    assert len(grid.keys) == 1
    assert grid._centres.shape == (1, 3)
    assert np.allclose(unpack_centres(grid.keys, grid.voxel_size)[0], centre, atol=grid.voxel_size)


def test_a_path_nothing_could_place_is_empty_rather_than_a_line_of_origins() -> None:
    """A tf frame can exist and still be unreachable from the world frame.

    Every lookup then fails, and reporting the world origin for the whole recording
    would put the robot somewhere it has never been for its entire run -- and the
    costmap planner treats that path as the known free corridor, so a route would be
    offered from it. Both callers already read an empty path as "not known yet".
    """
    from dimos.teleop.memory_world.replay import frame_positions, stamped_positions

    assert frame_positions([1.0, 2.0, 3.0], lambda ts: None) == []
    assert frame_positions([], lambda ts: None) == []
    assert stamped_positions([SimpleNamespace(pose_tuple=None)] * 3) == []


def test_a_viewer_request_cannot_build_a_replay_the_operator_turned_off(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """`build_replay_on_start = False` must mean the streams are not written AT ALL.

    It only ever gated the startup build. `_replay_if_ready` -- which every
    `/replay/index` request goes through -- started a build thread whenever the replay
    was not loaded, without consulting the flag, so the first viewer to ask for a
    timeline rebuilt it anyway.

    That is how a demo launched with the flag OFF, specifically so a deliberate deletion
    of `voxel_diff`/`voxel_keyframe` would stick, wrote 3,838 diffs and 77 keyframes back
    into the recording -- while another process was writing to the same file. The refusal
    is worded as a settled answer so the viewer stops polling instead of waiting for a
    build that will never start.
    """
    import threading
    from types import SimpleNamespace

    from dimos.teleop.memory_world.module import MemoryWorldModule

    module = MemoryWorldModule.__new__(MemoryWorldModule)
    module._replay = None
    module._replay_error = None
    module._replay_index = None
    module._replay_lock = threading.Lock()
    module._workers_lock = threading.Lock()
    module._stopping = threading.Event()
    module._replay_thread = None
    module._replay_progress = "not started"
    module.config = SimpleNamespace(build_replay_on_start=False)

    with pytest.raises(RuntimeError, match="turned off"):
        module._replay_if_ready()
    assert module._replay_thread is None, "a build thread was started anyway"
    assert module._replay_lock.acquire(blocking=False), "the lock was not released"
    module._replay_lock.release()

    # And the refusal has to reach the BROWSER. The routes discard this exception and
    # answer `f"replay {self._replay_progress}"`, so a refusal that only raises left the
    # viewer reading "replay not started" -- its "turned off" branch unreachable, polling
    # for ever for a build that is refused by design.
    detail = f"replay {module._replay_progress}"
    assert "turned off" in detail, (
        f"the viewer would receive {detail!r}, which it treats as a build that has not"
        " started yet rather than one that will never happen"
    )


def test_a_started_build_reports_building_not_not_started(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """While a build thread is alive the status must not say "not started".

    `_replay_locked` sets "building" only when the voxel streams are MISSING. On a
    recording that already has them -- every run after the first -- it goes straight from
    "not started" to "ready", while `_replay_if_ready` has already started the thread and
    every poll for its whole duration fails the non-blocking lock and answers
    503 "replay not started".

    That was survivable while the viewer treated "not started" as "not yet". It is not
    now: the viewer reads it as settled and prints "this recording has no timeline" over
    a build that may be minutes from finishing -- and a first connection on a long mcap
    is budgeted at about a minute.
    """
    import threading
    from types import SimpleNamespace

    from dimos.teleop.memory_world.module import MemoryWorldModule

    started = threading.Event()
    release = threading.Event()

    module = MemoryWorldModule.__new__(MemoryWorldModule)
    module._replay = None
    module._replay_error = None
    module._replay_index = None
    module._replay_lock = threading.Lock()
    module._workers_lock = threading.Lock()
    module._stopping = threading.Event()
    module._replay_thread = None
    module._replay_progress = "not started"
    module.config = SimpleNamespace(build_replay_on_start=True)
    module._build_replay = lambda: (started.set(), release.wait(5))  # type: ignore[assignment]

    with pytest.raises(RuntimeError) as first:
        module._replay_if_ready()
    assert started.wait(2), "no build thread was started"
    assert "building" in str(first.value), str(first.value)

    # And every poll while it runs says the same, rather than "not started".
    with pytest.raises(RuntimeError) as again:
        module._replay_if_ready()
    assert "not started" not in str(again.value), (
        f"a live build reported {str(again.value)!r}, which the viewer treats as settled"
    )
    release.set()
    if module._replay_thread is not None:
        module._replay_thread.join(5)
