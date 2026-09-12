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

"""The pure parts of the mount calibration: which edge, which stream, which sample.

The fit itself needs a real recording and is exercised by running the command; what
is pinned here is everything that decides WHERE the measurement is applied, because
a right number written to the wrong edge is worse than no measurement at all.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.teleop.memory_world.calibrate_static_tf import (
    _nearest,
    _rotation,
    _thinned,
    camera_mount_edge,
    rigidly_joined,
)
from dimos.teleop.memory_world.tf_tree import TfTree

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def cart_tree() -> TfTree:
    """The cart's own shape: two sensor chains that meet at base_link."""
    tree = TfTree()
    # The odometry edge moves; everything below the body is bolted down.
    tree.add("map", "odom", 0.0, (0.0, 0.0, 0.0), IDENTITY, static=True)
    tree.add("odom", "base_link", 0.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("odom", "base_link", 1.0, (2.0, 0.0, 0.0), IDENTITY)
    for parent, child in (
        ("base_link", "sensor_mount_link"),
        ("sensor_mount_link", "livox_link"),
        ("livox_link", "livox_frame"),
        ("base_link", "handle_bar_link"),
        ("handle_bar_link", "camera_link"),
        ("camera_link", "camera_depth_optical_frame"),
        ("camera_depth_optical_frame", "camera_color_optical_frame"),
    ):
        tree.add(parent, child, 0.0, (0.1, 0.0, 0.0), IDENTITY, static=True)
    return tree


def test_the_mount_is_the_first_edge_descending_towards_the_camera() -> None:
    """Correcting it carries colour, depth and infra together and nothing else."""
    mount, root = camera_mount_edge(cart_tree(), "camera_depth_optical_frame", "livox_frame")
    assert (mount, root) == ("base_link", "handle_bar_link")


def test_a_camera_named_nothing_like_a_camera_still_finds_its_mount() -> None:
    """The edge is found by walking the tf path, never by what a frame is called."""
    tree = TfTree()
    for parent, child in (
        ("odom", "body"),
        ("body", "lidar_mount"),
        ("lidar_mount", "lidar"),
        ("body", "realsense_link"),
        ("realsense_link", "realsense_depth_frame"),
        ("realsense_depth_frame", "realsense_depth_optical_frame"),
    ):
        tree.add(parent, child, 0.0, (0.1, 0.0, 0.0), IDENTITY, static=True)
    assert camera_mount_edge(tree, "realsense_depth_optical_frame", "lidar") == (
        "body",
        "realsense_link",
    )


def test_an_unreachable_camera_names_no_mount() -> None:
    tree = cart_tree()
    assert camera_mount_edge(tree, "camera_depth_optical_frame", "nowhere") == (
        None,
        "camera_depth_optical_frame",
    )


def test_a_frame_counts_only_when_it_holds_still_against_the_camera() -> None:
    """That is the property a calibration needs, whatever a frame is called."""
    tree = cart_tree()
    assert rigidly_joined(tree, "livox_frame", "camera_depth_optical_frame")
    # odom is a frame the body moves under, so a cloud stamped there cannot calibrate.
    assert not rigidly_joined(tree, "odom", "camera_depth_optical_frame")
    assert not rigidly_joined(tree, "nowhere", "camera_depth_optical_frame")


def test_a_lidar_above_the_body_still_holds_still_against_the_camera() -> None:
    """pointlio tracks the lidar on some rigs, so base_link hangs under it."""
    tree = TfTree()
    tree.add("world", "lidar_link", 0.0, (0.0, 0.0, 0.0), IDENTITY)  # moving: the odometry
    tree.add("world", "lidar_link", 1.0, (1.0, 0.0, 0.0), IDENTITY)
    for parent, child in (
        ("lidar_link", "base_link"),
        ("base_link", "cam"),
        ("cam", "cam_optical"),
    ):
        tree.add(parent, child, 0.0, (0.1, 0.0, 0.0), IDENTITY, static=True)
    assert rigidly_joined(tree, "lidar_link", "cam_optical")
    assert not rigidly_joined(tree, "world", "cam_optical")  # the odometry edge moves
    # And the correction must not land on the body's own edge, which would rotate
    # every base_link pose by the camera's error.
    assert camera_mount_edge(tree, "cam_optical", "lidar_link") == ("base_link", "cam")


def test_a_republished_static_edge_still_counts_as_rigid() -> None:
    """A stitched recording puts its static edges in the moving stream, many times."""
    tree = TfTree()
    for ts in (0.0, 1.0, 2.0):
        tree.add("body", "lidar", ts, (0.5, 0.0, 0.0), IDENTITY)
        tree.add("body", "cam", ts, (0.0, 0.2, 0.0), IDENTITY)
    assert rigidly_joined(tree, "lidar", "cam")


def test_the_nearest_sample_wins_not_the_first_in_the_window() -> None:
    """Anything else absorbs the robot's motion during the window into the mount."""

    class Stream:
        def at(self, ts: float, tolerance: float) -> list[object]:
            return [obs for obs in samples if abs(obs.ts - ts) <= tolerance]

    class Sample:
        def __init__(self, ts: float) -> None:
            self.ts = ts

    samples = [Sample(0.9), Sample(1.0), Sample(1.1)]
    assert _nearest(Stream(), 1.0, 0.2).ts == 1.0
    assert _nearest(Stream(), 1.09, 0.2).ts == 1.1
    with pytest.raises(LookupError):
        _nearest(Stream(), 5.0, 0.2)


def test_the_rotation_helper_is_a_rotation_and_turns_the_right_way() -> None:
    for angles in ((0.0, 0.0, 0.0), (0.3, -1.1, 2.0), (np.pi / 2, 0.0, -np.pi / 4)):
        r = _rotation(*angles)
        assert np.allclose(r.T @ r, np.eye(3), atol=1e-12)
        assert np.linalg.det(r) == pytest.approx(1.0)
    # A quarter turn about z takes x to y, which fixes the sign convention.
    assert np.allclose(
        _rotation(0.0, 0.0, np.pi / 2) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12
    )


def test_thinning_keeps_one_point_per_cube_and_obeys_the_cap() -> None:
    dense = np.repeat(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]), 50, axis=0)
    assert len(_thinned(dense, 0.5, 100)) == 3  # three cubes, however many points
    spread = np.arange(3000, dtype=np.float64).reshape(-1, 1) * [[1.0, 1.0, 1.0]]
    thinned = _thinned(spread, 0.5, 500)
    assert len(thinned) == 500  # capped, and every survivor is one of the originals
    assert {tuple(p) for p in thinned} <= {tuple(p) for p in spread}


def test_a_measured_mount_is_written_into_the_recordings_own_tf(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The fix goes in the tf everything already reads, not in a stream beside it.

    The map, the markers, the pictures and Hyperspace all read the recording's `tf`.
    A correction parked anywhere else is a second source for the same joint that can
    disagree with the first, and Hyperspace, which reads `tf` alone, would never see
    it. The rig republishes the mount with every sample, so every sample of that edge
    is rewritten; every other edge, moving or not, survives untouched.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.calibrate_static_tf import write_mount_into_tf
    from dimos.teleop.memory_world.recording import build_tf_tree

    def edge(parent: str, child: str, x: float, ts: float = 1.0) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=ts,
        )

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    try:
        tf = store.stream("tf", TFMessage)
        for step in range(5):  # a moving joint, and the mount republished beside it
            tf.append(
                TFMessage(
                    edge("odom", "lidar", float(step), float(step)),
                    edge("lidar", "mount", 1.0, float(step)),
                    edge("mount", "cam", 9.0, float(step)),
                    edge("mount", "imu", 4.0, float(step)),
                ),
                ts=float(step),
            )
        assert build_tf_tree(store, "tf").lookup("lidar", "cam", 2.0)[0, 3] == 10.0

        fixed = np.eye(4)
        fixed[0, 3] = 2.0
        assert write_mount_into_tf(store, "tf", "mount", "cam", fixed) == 5

        tree = build_tf_tree(store, "tf")
        assert tree.lookup("lidar", "cam", 2.0)[0, 3] == 3.0  # the measured mount
        assert tree.lookup("mount", "imu", 2.0)[0, 3] == 4.0  # every other edge untouched
        assert tree.lookup("odom", "lidar", 3.0)[0, 3] == 3.0  # the moving joint still moves

        # Running it again is not a second correction: it replaces, never accumulates.
        assert write_mount_into_tf(store, "tf", "mount", "cam", fixed) == 5
        assert build_tf_tree(store, "tf").lookup("lidar", "cam", 2.0)[0, 3] == 3.0
    finally:
        store.stop()


def test_the_tf_keeps_its_sample_timeline_when_the_mount_is_corrected(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """Correcting one edge must not collapse the stream it lives in.

    tf is a timeline: the moving joints are only interpolable because each sample
    keeps its own timestamp. Flattening the stream into one message, the way a latched
    static tf can be rewritten, would freeze every moving joint in the recording.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.calibrate_static_tf import write_mount_into_tf

    def edge(parent: str, child: str, x: float, ts: float = 1.0) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=ts,
        )

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    try:
        tf = store.stream("tf", TFMessage)
        for step in range(5):
            tf.append(
                TFMessage(
                    edge("odom", "lidar", float(step), float(step)),
                    edge("lidar", "cam", 9.0, float(step)),
                ),
                ts=float(step),
            )

        write_mount_into_tf(store, "tf", "lidar", "cam", np.eye(4))

        rows = list(store.streams["tf"])
        assert [float(obs.ts) for obs in rows] == [0.0, 1.0, 2.0, 3.0, 4.0]
        assert all(len(obs.data.transforms) == 2 for obs in rows)  # no edge duplicated
        assert "tf__rebuilt" not in store.list_streams()  # the staged copy is cleaned up
        for step, obs in enumerate(rows):  # and the correction really is in every sample
            by_edge = {(str(t.frame_id), str(t.child_frame_id)): t for t in obs.data.transforms}
            assert by_edge[("lidar", "cam")].translation.x == 0.0  # was 9.0
            assert by_edge[("odom", "lidar")].translation.x == float(step)  # untouched
    finally:
        store.stop()


def test_the_mount_correction_reproduces_what_was_measured(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """corrected_mount turns a lidar-to-camera measurement into one edge of the tree."""
    from dimos.teleop.memory_world.calibrate_static_tf import corrected_mount

    tree = cart_tree()
    measured = np.eye(4)
    measured[:3, :3] = _rotation(0.0, 0.0, np.pi / 2)
    measured[:3, 3] = [0.3, -0.2, 0.1]

    mount, child, matrix = corrected_mount(
        tree, measured, "camera_depth_optical_frame", "livox_frame", 0.5
    )
    assert (mount, child) == ("base_link", "handle_bar_link")
    tree._edges.pop((mount, child))
    tree.add(mount, child, 0.0, tuple(matrix[:3, 3]), _quat(matrix[:3, :3]), static=True)
    # The whole point: the tree now produces exactly the transform that was measured.
    assert np.allclose(tree.lookup("livox_frame", "camera_depth_optical_frame", 0.5), measured)


def _quat(rotation: np.ndarray) -> tuple[float, float, float, float]:
    from dimos.teleop.memory_world.tf_tree import quaternion_from_matrix

    return quaternion_from_matrix(rotation)


def test_writing_a_mount_removes_only_what_it_invalidated(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The command that invalidates the caches clears them, and nothing else.

    A search index and Hyperspace's keyframes store camera poses as computed and never
    re-place them, so moving the mount leaves both a whole correction away from the map
    while every surface still says ready. Those keyframes now live in the recording, so
    this is the destructive half of --write operating on the one file everything reads:
    what it must NOT touch matters more than what it must.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.calibrate_static_tf import drop_what_the_mount_invalidates
    from dimos.teleop.memory_world.hyperspace_search import (
        KEYFRAME_STREAM,
        PATCH_STREAM,
        memory_db_for,
    )
    from dimos.teleop.memory_world.visual_search import PatchGrid

    recording = tmp_path / "rec.db"
    store = SqliteStore(path=str(recording), must_exist=False)
    store.start()
    try:
        row = TFMessage(
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="a",
                child_frame_id="b",
                ts=1.0,
            )
        )
        # An index is what it holds, not what it is called: one built with
        # --index-stream can be named anything, and one missed here keeps poses from
        # the mount that was just replaced.
        grid = PatchGrid(source_id=1, rows=2, cols=2, patches=np.zeros((4, 2), dtype=np.float16))
        for name in ("color_image_index_siglip2_so400m_p16_384", "camera_search", PATCH_STREAM):
            store.stream(name, PatchGrid).append(grid, ts=1.0)
        store.stream(KEYFRAME_STREAM, TFMessage).append(row, ts=1.0)
        for name in (
            "voxel_keyframe",  # stays: the map is lidar, which the mount does not move
            "voxel_diff",
            "livox_lidar",
            "pointlio_odometry_corrected",
        ):
            store.stream(name, TFMessage).append(row, ts=1.0)
        assert memory_db_for(recording) == recording  # one db: there is nowhere else

        dropped = drop_what_the_mount_invalidates(store, str(recording))

        assert set(dropped) == {
            KEYFRAME_STREAM,
            PATCH_STREAM,
            "color_image_index_siglip2_so400m_p16_384",
            "camera_search",
        }
        assert {
            "voxel_keyframe",
            "voxel_diff",
            "livox_lidar",
            "pointlio_odometry_corrected",
        } <= set(store.list_streams())
        assert recording.exists()  # the recording itself is never the thing removed
        assert drop_what_the_mount_invalidates(store, str(recording)) == []  # nothing left to do
    finally:
        store.stop()


def test_a_tf_write_that_fails_early_takes_nothing_away(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The corrected copy is written first, under another name, so a failure there is
    free: the recording still has its tf and the half-written copy is swept up.

    This is the ordinary failure -- a transient write error, a Ctrl-C -- and it must cost
    nothing at all.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.calibrate_static_tf import write_mount_into_tf

    def edge(parent: str, child: str, x: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=1.0,
        )

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    try:
        store.stream("tf", TFMessage).append(
            TFMessage(
                edge("mount", "cam", 9.0), edge("mount", "imu", 4.0), edge("mount", "gps", 2.0)
            ),
            ts=1.0,
        )
        real_stream = store.stream

        def dying_stream(name: str, *args, **kwargs):  # type: ignore[no-untyped-def]
            # Only the WRITE call names a payload type; reads do not. This dies on the
            # staged copy, before the recording's own tf has been touched.
            if name == "tf__rebuilt" and args:
                raise OSError(28, "No space left on device")
            return real_stream(name, *args, **kwargs)

        store.stream = dying_stream  # type: ignore[assignment]
        with pytest.raises(OSError):
            write_mount_into_tf(store, "tf", "mount", "cam", np.eye(4))
        store.stream = real_stream  # type: ignore[assignment]

        assert "tf__rebuilt" not in store.list_streams()  # no half-written copy left over
        surviving = {
            (str(t.frame_id), str(t.child_frame_id), t.translation.x)
            for obs in store.streams["tf"]
            for t in obs.data.transforms
        }
        assert surviving == {("mount", "cam", 9.0), ("mount", "imu", 4.0), ("mount", "gps", 2.0)}
    finally:
        store.stop()


def test_correcting_an_edge_the_tf_never_carried_is_refused(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A typo'd frame name must not silently rewrite the tf into an unchanged copy.

    Reporting success while nothing was corrected is the worst outcome here: the
    measurement is thrown away and the recording still says the wrong thing.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.calibrate_static_tf import write_mount_into_tf

    def edge(parent: str, child: str, x: float, ts: float = 1.0) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=ts,
        )

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    try:
        store.stream("tf", TFMessage).append(TFMessage(edge("mount", "cam", 9.0)), ts=1.0)
        with pytest.raises(SystemExit):
            write_mount_into_tf(store, "tf", "mount", "camera", np.eye(4))
        rows = list(store.streams["tf"])
        assert len(rows) == 1 and rows[0].data.transforms[0].translation.x == 9.0
    finally:
        store.stop()


def test_answer_positions_reads_both_places_an_answer_can_point() -> None:
    """It is the only input to the marker fallback, so both shapes have to reach it."""
    from dimos.teleop.memory_world.query import HighlightPoint, MemoryQueryResult, answer_positions

    assert answer_positions(MemoryQueryResult(answer="a")) == []
    assert answer_positions(MemoryQueryResult(answer="a", focus_point=(1.0, 2.0, 3.0))) == [
        (1.0, 2.0, 3.0)
    ]
    both = MemoryQueryResult(
        answer="a",
        focus_point=(9.0, 9.0, 9.0),
        points=[HighlightPoint(position=(1.0, 0.0, 0.0), label="x")],
    )
    assert answer_positions(both) == [(1.0, 0.0, 0.0), (9.0, 9.0, 9.0)]


def test_a_tf_write_that_dies_leaves_the_corrected_copy_in_the_recording(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A write that fails persistently must still not leave the recording without a tf.

    Putting the original back is only possible while the disk still takes writes. If it
    does not -- a full disk, a read-only remount -- the restore fails too and the tf is
    gone for good. So the corrected copy is written under another name first: the worst
    case is then a recording that holds its tf under the wrong name, which is a minute's
    work to undo, instead of one that holds no tf at all.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.calibrate_static_tf import write_mount_into_tf

    def edge(parent: str, child: str, x: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=1.0,
        )

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    try:
        store.stream("tf", TFMessage).append(
            TFMessage(edge("mount", "cam", 9.0), edge("mount", "imu", 4.0)), ts=1.0
        )
        real_stream = store.stream

        def dying_stream(name: str, *args, **kwargs):  # type: ignore[no-untyped-def]
            # Every write of "tf" fails, forever: the disk is not coming back.
            if name == "tf" and args:
                raise OSError(28, "No space left on device")
            return real_stream(name, *args, **kwargs)

        store.stream = dying_stream  # type: ignore[assignment]
        with pytest.raises(SystemExit):
            write_mount_into_tf(store, "tf", "mount", "cam", np.eye(4))
        store.stream = real_stream  # type: ignore[assignment]

        assert "tf" not in store.list_streams()  # the window this is about
        surviving = [
            (str(t.frame_id), str(t.child_frame_id), t.translation.x)
            for obs in store.streams["tf__rebuilt"]
            for t in obs.data.transforms
        ]
        assert ("mount", "cam", 0.0) in surviving  # corrected, and nothing was lost
        assert ("mount", "imu", 4.0) in surviving

        # And a second run refuses rather than overwriting the only copy there is.
        store.stream = real_stream  # type: ignore[assignment]
        with pytest.raises(SystemExit):
            write_mount_into_tf(store, "tf", "mount", "cam", np.eye(4))
    finally:
        store.stop()


def test_a_stale_static_mount_cannot_outvote_the_correction(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """build_tf_tree lays tf_static over the moving tf, so a static copy of the corrected
    edge would go on winning and the write would look like it had done nothing.

    This is the whole failure the one-tf-tree change is about, in miniature: two places
    declaring one joint, and the reader picking the stale one.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.calibrate_static_tf import write_mount_into_tf
    from dimos.teleop.memory_world.recording import build_tf_tree

    def edge(parent: str, child: str, x: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=1.0,
        )

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    try:
        store.stream("tf", TFMessage).append(
            TFMessage(edge("lidar", "mount", 1.0), edge("mount", "cam", 9.0)), ts=1.0
        )
        store.stream("tf_static", TFMessage).append(
            TFMessage(edge("mount", "cam", 9.0), edge("mount", "imu", 4.0)), ts=1.0
        )

        fixed = np.eye(4)
        fixed[0, 3] = 2.0
        write_mount_into_tf(store, "tf", "mount", "cam", fixed)

        tree = build_tf_tree(store, "tf")
        assert tree.lookup("lidar", "cam", 1.0)[0, 3] == 3.0  # the correction, not 10.0
        assert tree.lookup("mount", "imu", 1.0)[0, 3] == 4.0  # the rest of the statics stay
    finally:
        store.stop()


def test_a_static_tf_holding_only_the_corrected_edge_goes_away(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A static stream left behind holding nothing is a stream the next reader must still
    open, decode and reason about. If the corrected edge was all it declared, it goes."""
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.calibrate_static_tf import write_mount_into_tf

    def edge(parent: str, child: str, x: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=1.0,
        )

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    try:
        store.stream("tf", TFMessage).append(TFMessage(edge("mount", "cam", 9.0)), ts=1.0)
        store.stream("tf_static", TFMessage).append(TFMessage(edge("mount", "cam", 9.0)), ts=1.0)
        write_mount_into_tf(store, "tf", "mount", "cam", np.eye(4))
        assert "tf_static" not in store.list_streams()
    finally:
        store.stop()
