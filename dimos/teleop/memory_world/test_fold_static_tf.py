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


"""Folding a recording's static tf edges into its moving tf.

Split out of `test_recording.py`, which had grown past the repo's 75 KB file hook.
`fold_static_tf` rewrites the recording in place and deletes the static stream, so it is
the one function here that can destroy data it cannot get back -- which is why it has
this many tests, and why most of them were written after it did.
"""

from __future__ import annotations


def _tf_store(tmp_path):  # type: ignore[no-untyped-def]
    from dimos.memory.store.sqlite import SqliteStore

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    return store


def _edge(parent: str, child: str, x: float, ts: float = 1.0):  # type: ignore[no-untyped-def]
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    return Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def test_folding_compares_every_sample_the_moving_stream_carries(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A rig that republishes its mounts gets a stale one in there often enough that
    checking the first sample proves nothing about the rest.

    Leaving such an edge alone and deleting the static stream that disagreed with it is the
    one outcome that loses the right answer outright.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0)  # agrees
        tf.append(TFMessage(_edge("base", "cam", 9.0, 2.0)), ts=2.0)  # and then does not
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )

        assert fold_static_tf(store, "tf", "tf_static") == 1
        assert build_tf_tree(store, "tf").lookup("base", "cam", 2.0)[0, 3] == 2.0
    finally:
        store.stop()


def test_a_static_edge_is_folded_even_where_the_moving_stream_agrees(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """An edge is static because it holds for all time. A moving stream that happens to
    carry the same value in the samples it carries does not say that anywhere.

    A rig that republishes a mount for the first half of a recording and stops leaves the
    second half with no mount at all, and comparing values would call that agreement and
    delete the static stream that was the only thing making it hold throughout.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(
            TFMessage(_edge("base", "cam", 2.0, 1.0), _edge("odom", "base", 1.0, 1.0)), ts=1.0
        )
        tf.append(TFMessage(_edge("odom", "base", 2.0, 2.0)), ts=2.0)  # the mount stops
        tf.append(TFMessage(_edge("odom", "base", 3.0, 3.0)), ts=3.0)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )

        assert fold_static_tf(store, "tf", "tf_static") == 1
        assert "tf_static" not in store.list_streams()
        tree = build_tf_tree(store, "tf")
        assert tree.lookup("odom", "cam", 3.0)[0, 3] == 5.0  # placed where it had stopped
        assert tree.lookup("odom", "cam", 2.0)[0, 3] == 4.0  # and in the middle
        # Once, not once per sample: a single-sample series holds from its stamp forward
        # for ever, so one copy says what a static says and a long tf is not multiplied
        # by its mounts.
        mounts = [
            t
            for obs in store.streams["tf"]
            for t in obs.data.transforms
            if (str(t.frame_id), str(t.child_frame_id)) == ("base", "cam")
        ]
        assert len(mounts) == 1
    finally:
        store.stop()


def test_folding_into_an_empty_tf_refuses_rather_than_destroying_the_statics(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """There is nowhere to fold into, and going ahead writes nothing while deleting the
    static stream on the way out -- every mount in the recording, gone, reported as done."""
    import pytest

    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import fold_static_tf

    store = _tf_store(tmp_path)
    try:
        store.stream("tf", TFMessage)  # declared and empty
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        with pytest.raises(SystemExit):
            fold_static_tf(store, "tf", "tf_static")
        assert "tf_static" in store.list_streams()
    finally:
        store.stop()


def test_a_static_tf_cut_short_by_a_dead_rebuild_is_refused_too(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The moving tf is not the only stream a rebuild can die in the middle of.

    A truncated static tf is worse than a truncated moving one, because it is silent in
    both directions: the tree loads with a mount missing, and a fold would then delete the
    only complete copy there is.
    """
    import pytest

    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        store.stream("tf", TFMessage).append(TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0)
        store.stream("tf_static", TFMessage).append(TFMessage(_edge("base", "imu", 4.0)), ts=1.0)
        staged = store.stream("tf_static__rebuilt", TFMessage)
        staged.append(TFMessage(_edge("base", "imu", 4.0), _edge("base", "gps", 2.0)), ts=1.0)

        with pytest.raises(SystemExit):
            build_tf_tree(store, "tf")
        with pytest.raises(SystemExit):
            fold_static_tf(store, "tf", "tf_static")
        assert "tf_static__rebuilt" in store.list_streams()  # the complete copy stays
    finally:
        store.stop()


def test_a_folded_mount_covers_the_whole_span_the_tf_can_be_asked_about(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A tf message carries its own stamp, and it is not the stamp it was recorded at.

    TfTree reads the message's. Stamping the folded copies by the recording stamp alone
    leaves the edge holding over a window slightly inside the recording, and the frames at
    either end -- the first and last thing the camera saw -- are placed nowhere.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(TFMessage(_edge("odom", "base", 1.0, 10.0)), ts=11.0)  # said at 10, kept at 11
        tf.append(TFMessage(_edge("odom", "base", 2.0, 20.0)), ts=21.0)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 10.0)[0, 3] == 3.0

        fold_static_tf(store, "tf", "tf_static")
        tree = build_tf_tree(store, "tf")
        assert tree.lookup("odom", "cam", 10.0)[0, 3] == 3.0  # the first frame, still placed
        assert tree.lookup("odom", "cam", 20.0)[0, 3] == 4.0  # and the last
    finally:
        store.stop()


def test_a_static_folded_into_a_one_sample_tf_still_holds_for_all_time(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A recording whose odometry produced one tf message still has pictures all through it.

    _Edge.at holds a single-sample series for all time, which is exactly what a static edge
    means. Two samples at the same instant do not: they expire a tolerance either side. So
    where the two ends of the span coincide, one copy goes in and not two.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        store.stream("tf", TFMessage).append(TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 100.0)[0, 3] == 3.0
    finally:
        store.stop()


def test_an_unstamped_transform_does_not_move_the_folded_edge_past_the_first_frame(  # type: ignore[no-untyped-def]
    tmp_path,
) -> None:
    """TfTree reads a transform's own stamp and falls back to the observation's when it is 0.

    Measuring the span any other way makes the earliest stamp look like 0, the folded copy
    is written there, and the reader turns that back into the recording stamp of the first
    row -- later than the first frame the camera took, which is then placed nowhere.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        # One transform unstamped (the reader will call it 10, the row's own stamp) and one
        # stamped at 5. The earliest moment the tree can be asked about is 5, not 0.
        tf.append(
            TFMessage(_edge("odom", "base", 1.0, 0.0), _edge("base", "wheel", 1.0, 5.0)),
            ts=10.0,
        )
        tf.append(TFMessage(_edge("odom", "base", 2.0, 20.0)), ts=20.0)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        assert build_tf_tree(store, "tf").lookup("base", "cam", 5.0)[0, 3] == 2.0

        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("base", "cam", 5.0)[0, 3] == 2.0
    finally:
        store.stop()


def test_a_static_declared_twice_folds_the_first_value(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A latched static tf republished with a different value: the tree keeps the FIRST.

    Folding has to keep the same one, or the recording means something different after it
    was folded than it did before -- and an mcap, which is folded nowhere because it cannot
    be written to, would then disagree with the .db beside it.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        for step in (1.0, 2.0):
            tf.append(TFMessage(_edge("odom", "base", step, step)), ts=step)
        latched = store.stream("tf_static", TFMessage)
        latched.append(TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0)
        latched.append(TFMessage(_edge("base", "cam", 9.0, 2.0)), ts=2.0)  # a later, different one
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 2.0)[0, 3] == 4.0

        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 2.0)[0, 3] == 4.0
    finally:
        store.stop()


def test_a_folded_mount_outlives_the_tf_the_way_a_static_did(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A static edge holds for all time, and folding must not quietly put an end on it.

    An odometry that published once still places at any later moment, because a
    single-sample series holds forward for ever. A mount folded in as two samples would
    stop holding after the last of them, so the camera would go missing from a recording
    whose robot never moved again -- while the odometry beside it carried on.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0)  # published once
        tf.append(TFMessage(_edge("base", "wheel", 1.0, 2.0)), ts=2.0)  # something else, later
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 100.0)[0, 3] == 3.0

        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 100.0)[0, 3] == 3.0
    finally:
        store.stop()


def test_folding_a_static_edge_spelled_two_ways_does_not_corrupt_the_tf(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """One recording can spell the same edge both ways -- `/base -> /cam` on tf_static and
    `base -> cam` on tf -- and `TfTree` reads them as one edge while `fold_static_tf` keyed
    them raw and read them as two.

    The consequence is not a missed fold, it is a corrupted one: the stale MOVING copy
    survived the filter, then outvoted the folded static (it is a later sample of what
    the tree sees as the same edge), and `tf_static` -- the only record of the right
    value -- was deleted. Measured `odom -> cam` at t=2: 4.0 before the fold, 11.0 after,
    with the evidence gone. There is no recovering from that without the original file.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    def edge(parent: str, child: str, x: float, ts: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=ts,
        )

    store = SqliteStore(path=str(tmp_path / "mixed.db"), must_exist=False)
    store.start()
    try:
        # tf_static says base -> cam is 3.0, for all time, spelled with slashes.
        store.stream("tf_static", TFMessage).append(
            TFMessage(edge("/base", "/cam", 3.0, 1.0)), ts=1.0
        )
        # tf carries a stale moving copy of the same edge, spelled without.
        tf = store.stream("tf", TFMessage)
        for step in range(1, 4):
            tf.append(
                TFMessage(
                    edge("odom", "base", 1.0, float(step)),
                    edge("base", "cam", 3.0 + step, float(step)),  # the stale copy
                ),
                ts=float(step),
            )

        before = build_tf_tree(store, "tf").lookup("odom", "cam", 2.0)[0, 3]
        fold_static_tf(store, "tf", "tf_static")
        after = build_tf_tree(store, "tf").lookup("odom", "cam", 2.0)[0, 3]

        assert after == 4.0, (
            f"folding changed odom -> cam from {before} to {after}: the stale moving copy"
            " outvoted the static one, and tf_static has been deleted"
        )
    finally:
        store.stop()


def test_a_folded_static_holds_from_zero_on_a_recording_that_starts_at_zero(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """`TfTree.from_stream` reads `transform.ts or obs.ts`, so a transform stamped
    exactly 0.0 is read as UNSTAMPED and falls back to its observation's stamp.

    On a recording whose timebase starts at zero -- a synthetic or simulated one -- that
    is precisely the moment `fold_static_tf` restates the static to hold from. The static
    then did not hold at the moment it was written to hold from, and a camera pose in
    that gap stopped resolving after a fold that is supposed to change nothing.

    The existing fold tests all start at 1.0, where `or` cannot fire.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    def edge(parent: str, child: str, x: float, ts: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=ts,
        )

    # The rig latches its statics before it starts moving, so the static's stamp is
    # EARLIER than the first tf observation -- which is what makes the fold restate it at
    # a moment no tf row is stamped at. With `latched` at 1.0 the restated stamp is 1.0
    # and reads back as 1.0; with `latched` at 0.0 it reads back as the first tf row's
    # stamp instead, and the gap the fold exists to preserve is gone.
    for latched in (1.0, 0.0):
        store = SqliteStore(path=str(tmp_path / f"t{latched}.db"), must_exist=False)
        store.start()
        try:
            store.stream("tf_static", TFMessage).append(
                TFMessage(edge("base", "cam", 3.0, latched)), ts=latched
            )
            tf = store.stream("tf", TFMessage)
            for step in range(3):
                at = latched + 2.0 + step  # the robot starts moving two seconds later
                tf.append(TFMessage(edge("world", "base", 0.0, at)), ts=at)

            # The FOLDED EDGE itself, not a chain through it: `world -> base` only
            # exists once the robot moves, so a chain would be None at this moment for a
            # reason that has nothing to do with the fold.
            fold_static_tf(store, "tf", "tf_static")
            after = build_tf_tree(store, "tf").lookup("base", "cam", latched)

            assert after is not None, (
                f"latched at {latched}: the folded static stopped holding at the moment it"
                " was restated to hold from"
            )
            assert after[0, 3] == 3.0
        finally:
            store.stop()


def test_folding_does_not_retime_the_moving_transforms_it_keeps(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A fold rewrites where the STATIC edges sit. It must not move anything else.

    `TfTree.from_stream` reads `transform.ts or obs.ts`, so a moving transform that
    carries no stamp of its own takes its row's. Putting the folded statics INTO the
    first existing row and moving that row's stamp back -- which is how the zero-timebase
    fix was first written -- therefore retimed every unstamped moving transform in it:
    `world -> base` read 0 m at t=2 before the fold and 1 m after, with a pose appearing
    at t=1 where there had been none. The statics get their own row instead.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    def unstamped(parent: str, child: str, x: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=0.0,  # no stamp of its own: it takes its row's
        )

    store = SqliteStore(path=str(tmp_path / "retime.db"), must_exist=False)
    store.start()
    try:
        store.stream("tf_static", TFMessage).append(
            TFMessage(unstamped("base", "cam", 5.0)), ts=0.0
        )
        tf = store.stream("tf", TFMessage)
        tf.append(TFMessage(unstamped("world", "base", 0.0)), ts=2.0)
        tf.append(TFMessage(unstamped("world", "base", 2.0)), ts=4.0)

        tree = build_tf_tree(store, "tf")
        before = {t: tree.lookup("world", "base", t) for t in (1.0, 2.0, 3.0)}
        assert before[1.0] is None, "the fixture does not start where it says it does"
        assert before[2.0][0, 3] == 0.0
        assert before[3.0][0, 3] == 1.0

        fold_static_tf(store, "tf", "tf_static")

        tree = build_tf_tree(store, "tf")
        after = {t: tree.lookup("world", "base", t) for t in (1.0, 2.0, 3.0)}
        assert after[1.0] is None, "the fold invented a moving pose before the drive began"
        assert after[2.0][0, 3] == 0.0, (
            f"the fold moved the moving poses: {after[2.0][0, 3]} at t=2, was 0.0"
        )
        assert after[3.0][0, 3] == 1.0

        # And the static it was folding still holds from the moment it was restated at.
        assert tree.lookup("base", "cam", 0.0) is not None
    finally:
        store.stop()


def test_a_folded_static_is_written_in_the_moving_streams_spelling(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """`TfTree` canonicalises, and a reader outside this package need not.

    dimos' own `MultiTBuffer` keys the RAW `(frame_id, child_frame_id)` pair, so a
    recording that spells the same edge both ways -- `/base -> /cam` on tf_static and
    `base -> cam` on tf -- came out of the fold holding `odom -> base` and `/base -> /cam`
    together, with no chain through it for such a reader at all.

    So the folded edge is written in the spelling the MOVING stream uses. Taking the
    static's spelling unconditionally would break an all-slashed recording instead, which
    is why it is the moving stream that decides.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import fold_static_tf

    def edge(parent: str, child: str, x: float, ts: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=ts,
        )

    for moving_slash, static_slash in (("", "/"), ("/", "")):
        store = SqliteStore(
            path=str(tmp_path / f"spell{len(moving_slash)}{len(static_slash)}.db"),
            must_exist=False,
        )
        store.start()
        try:
            store.stream("tf_static", TFMessage).append(
                TFMessage(edge(f"{static_slash}base", f"{static_slash}cam", 3.0, 1.0)), ts=1.0
            )
            tf = store.stream("tf", TFMessage)
            for step in range(1, 4):
                tf.append(
                    TFMessage(edge(f"{moving_slash}odom", f"{moving_slash}base", 1.0, float(step))),
                    ts=float(step),
                )

            fold_static_tf(store, "tf", "tf_static")

            # Read it the way a raw-keyed reader does: one spelling throughout, or no chain.
            pairs = {
                (str(t.frame_id), str(t.child_frame_id))
                for obs in store.streams["tf"]
                for t in obs.data.transforms
            }
            frames = {name for pair in pairs for name in pair}
            slashed = {name for name in frames if name.startswith("/")}
            assert slashed in (set(), frames), (
                f"moving={moving_slash!r} static={static_slash!r}: the fold left two"
                f" spellings in one stream: {sorted(frames)}"
            )
            assert (f"{moving_slash}base", f"{moving_slash}cam") in pairs
        finally:
            store.stop()
