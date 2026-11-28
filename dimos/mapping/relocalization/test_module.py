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

import time
from types import SimpleNamespace
from typing import get_type_hints

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.core.stream import In
from dimos.mapping.relocalization import module as relocalization
from dimos.mapping.relocalization.cloud_relocalizer import MID360, CloudRelocalizer
from dimos.mapping.relocalization.module import PremapModule
from dimos.mapping.relocalization.utils import LiveMap
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


@pytest.fixture
def module():
    """Build a real module (and dispose it); `cls, **config` picks the class."""
    built = []

    def build(cls=PremapModule, **config):
        m = cls(**config)
        built.append(m)
        return m

    yield build
    for m in built:
        m.dispose()


def fixes(m):
    """Collect the transforms a module accepts."""
    got = []
    m.fixes.subscribe(got.append)
    return got


def test_submit_publishes_and_checks_frames(module):
    """submit does not second-guess the fix; the implementation already decided."""
    m = module()
    got = fixes(m)
    tf = Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map")
    m.submit(tf, "x")
    m.submit(tf, "x")
    assert got == [tf, tf]
    # A strategy handing over the placement instead of the frame transform is
    # caught here rather than publishing a backwards TF.
    with pytest.raises(AssertionError):
        m.submit(Transform.from_matrix(np.eye(4), frame_id="map", child_frame_id="world"))


def test_relocalize_once_stops_after_the_first_fix(module, monkeypatch):
    """With it on, nothing is attempted after a fix; with it off, the fix is refined from."""
    from reactivex import Subject

    attempts, strengths = [], []

    class FakeRelocalizer:
        def __init__(self, *_):
            pass

        def attempt(self, cloud, world_frame, map_frame, initial_fix=None):
            attempts.append(initial_fix)
            return SimpleNamespace(fix=None, result=SimpleNamespace(fitness=0.5))

    monkeypatch.setattr(relocalization, "CloudRelocalizer", FakeRelocalizer)
    tf = Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map")
    cloud = PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32))
    for once in (True, False):
        m = module(live_cloud="lidar", relocalize_once=once)
        m.relocalization_strength = SimpleNamespace(publish=strengths.append)
        # Synchronous, so the attempts land before the assert and no worker thread is left behind
        clouds = Subject()
        m._clouds = lambda clouds=clouds: clouds
        m.premap = cloud
        m._start_matching()
        m.submit(tf)
        clouds.on_next(cloud)
    assert attempts == [tf]
    # Refused or not, every attempt's fitness goes out
    assert [s.data for s in strengths] == [0.5]


def test_premap_defines_the_map_frame_and_waits_for_a_fix(module, tmp_path):
    """The base loads the premap and publishes it once placed."""
    from dimos.memory.store.sqlite import SqliteStore

    path = tmp_path / "somewhere.db"
    with SqliteStore(path=str(path)) as store:
        store.stream("map_v1_cloud", PointCloud2).append(
            PointCloud2.from_numpy(np.zeros((5, 3), dtype=np.float32), timestamp=0.0), ts=0.0
        )
    m = module()
    published, disposables = [], []
    # Capture publishes without a bus.
    m.loaded_map = SimpleNamespace(publish=published.append)
    m.register_disposable = disposables.append

    m._resolve_map(str(path))
    m._load_premap(str(path))
    assert m.premap is not None and len(m.premap) == 5
    assert m.premap.frame_id == "map/v1"
    assert len(disposables) == 1  # the gated publish
    assert published == []  # ... which stays silent until a fix lands
    m.submit(Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map/v1"))
    assert published == [m.premap]  # ... once, on that fix
    m.submit(Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map/v1"))
    assert published == [m.premap]  # ... and never again
    disposables[0].dispose()


def test_saved_premap_flags_survive_loading(module, tmp_path):
    """A saved cloud's per-point flags (premap bit) reach the bus, so the raytracer seeds only those."""
    from dimos.memory.store.sqlite import SqliteStore

    flags = np.array([0, 1, 0, 1, 1], dtype=np.uint32)
    path = tmp_path / "flagged.db"
    with SqliteStore(path=str(path)) as store:
        store.stream("map_v1_cloud", PointCloud2).append(
            PointCloud2.from_numpy(np.zeros((5, 3), dtype=np.float32), timestamp=0.0, flags=flags),
            ts=0.0,
        )
    m = module()
    m.register_disposable = lambda _: None

    m._resolve_map(str(path))
    m._load_premap(str(path))
    on_the_bus = PointCloud2.lcm_decode(m.premap.lcm_encode())
    np.testing.assert_array_equal(on_the_bus.flags_u32(), flags)


def test_relocalizer_refuses_below_its_own_threshold(monkeypatch):
    """One config surface: the relocalizer holds the knobs and the accept decision."""
    from dimos.mapping.relocalization import cloud_relocalizer

    placement = np.eye(4)
    placement[:3, 3] = [3.0, -1.0, 0.0]  # the map is 3 m +x of where the robot thought
    result = SimpleNamespace(transformation=placement, fitness=0.4, inlier_rmse=0.1)
    monkeypatch.setattr(cloud_relocalizer.CloudRelocalizer, "_prepare", lambda self, cloud: None)
    monkeypatch.setattr(
        cloud_relocalizer.CloudRelocalizer, "align", lambda self, cloud, initial=None: result
    )

    def relocalizer(threshold):
        return cloud_relocalizer.CloudRelocalizer(
            None, cloud_relocalizer.MID360.model_copy(update={"fitness_threshold": threshold})
        )

    assert relocalizer(0.5).relocalize(None, "world", "map") is None
    refused = relocalizer(0.5).attempt(None, "world", "map")
    assert refused.fix is None and refused.result.fitness == 0.4

    # Accepted: open3d places the live cloud in the map, the TF tree wants the
    # other direction, and relocalize() is what turns one into the other.
    tf = relocalizer(0.3).relocalize(None, "world", "map")
    assert (tf.frame_id, tf.child_frame_id) == ("world", "map")
    np.testing.assert_allclose(tf.to_matrix(), np.linalg.inv(placement), atol=1e-9)


def test_no_config_without_naming_a_rig():
    """The scales are per-rig, so there is no bare RelocalizeConfig() to fall into."""
    import pydantic

    from dimos.mapping.relocalization import cloud_relocalizer

    assert cloud_relocalizer.DEFAULT_PRESET in cloud_relocalizer.PRESETS
    assert cloud_relocalizer.PRESETS["mid360"] is cloud_relocalizer.MID360
    with pytest.raises(pydantic.ValidationError):
        cloud_relocalizer.RelocalizeConfig()


@pytest.fixture
def immediate_handoff(monkeypatch):
    """Inputs reach a busy worker's pending slot on the sending thread, so a test knows when one is waiting."""
    from reactivex.scheduler import ImmediateScheduler

    from dimos.utils import reactive

    monkeypatch.setattr(reactive, "get_scheduler", ImmediateScheduler)


def wait_for(condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.001)


def test_the_match_runs_on_the_latest_window_of_scans_once_the_last_match_ends(immediate_handoff):
    """A match sees the last `max_frames` scans; windows that land while it runs are dropped but the latest."""
    import threading

    from reactivex import Subject

    from dimos.mapping.relocalization.cloud_relocalizer import MID360
    from dimos.mapping.relocalization.utils import window

    cfg = MID360.model_copy(update={"min_frames": 2, "max_frames": 3})
    scans = Subject()
    matched, busy, release = [], threading.Event(), threading.Event()

    def match(cloud):
        busy.set()
        release.wait()
        matched.append(cloud)

    disposable = window(scans, cfg).subscribe(match)

    def scan(i):
        scans.on_next(PointCloud2.from_numpy(np.full((4, 3), i, dtype=np.float32), timestamp=0.0))

    scan(0)  # below min_frames
    scan(1)
    busy.wait(5.0)
    for i in range(2, 5):
        scan(i)
    release.set()
    wait_for(lambda: len(matched) == 2)
    disposable.dispose()
    # The second match holds the *last* three scans; the windows at 2 and 3 never ran.
    assert [sorted(set(c.points_f32()[:, 0])) for c in matched] == [[0.0, 1.0], [2.0, 3.0, 4.0]]


def test_continuous_matching_starts_on_the_latest_live_map_as_the_last_attempt_ends(
    module, monkeypatch, immediate_handoff
):
    import threading

    from reactivex import Subject

    seen, busy, release = [], threading.Event(), threading.Event()

    class FakeRelocalizer:
        def __init__(self, *_):
            pass

        def attempt(self, cloud, cloud_frame, map_frame, initial_fix=None):
            seen.append(len(cloud.points))
            busy.set()
            release.wait()
            return SimpleNamespace(fix=None, result=SimpleNamespace(fitness=0.1))

    monkeypatch.setattr(relocalization, "CloudRelocalizer", FakeRelocalizer)
    m = module(
        live_cloud="local_map", relocalize_once=False, min_local_points=0, match_interval_s=0.0
    )
    m.relocalization_strength = SimpleNamespace(publish=lambda _: None)
    local_maps = Subject()
    m.local_map = SimpleNamespace(observable=lambda: local_maps)
    m.lidar = SimpleNamespace(observable=Subject)
    m.premap = local_map([[0, 0, 0]])
    m._start_matching()

    # Each local map holds the ones before it, as the ray tracer's window does
    def send(n):
        local_maps.on_next(local_map([[i, 0, 0] for i in range(1, n + 1)]))

    send(1)
    busy.wait(5.0)
    for n in (2, 3, 4):
        send(n)
    wait_for(lambda: len(m.live) == 4)
    release.set()
    wait_for(lambda: len(seen) == 2)
    # One attempt on the first map, then one on the latest, none for those between
    assert seen == [1, 4]


def test_continuous_lidar_matches_the_latest_window_of_placed_scans(
    module, monkeypatch, immediate_handoff
):
    import threading

    from reactivex import Subject

    from dimos.mapping.relocalization.cloud_relocalizer import MID360

    seen, busy, release = [], threading.Event(), threading.Event()

    class FakeRelocalizer:
        def __init__(self, *_):
            pass

        def attempt(self, cloud, cloud_frame, map_frame, initial_fix=None):
            seen.append(sorted(np.asarray(cloud.points)[:, 0].round(2).tolist()))
            busy.set()
            release.wait()
            return SimpleNamespace(fix=None, result=SimpleNamespace(fitness=0.1))

    monkeypatch.setattr(relocalization, "CloudRelocalizer", FakeRelocalizer)
    m = module(
        map_frame="premap",
        relocalize_once=False,
        live_cloud="lidar",
        match_interval_s=0.0,
        relocalize=MID360.model_copy(update={"min_frames": 2, "max_frames": 2}),
    )
    m.relocalization_strength = SimpleNamespace(publish=lambda _: None)
    odom = Transform(translation=Vector3(1.0, 0.0, 0.0), frame_id="map", child_frame_id="base_link")
    m._tf = SimpleNamespace(
        get=lambda parent, child, *_: odom if (parent, child) == ("map", "base_link") else None,
        dispose=lambda: None,
    )
    scans = Subject()
    m.lidar = SimpleNamespace(observable=lambda: scans)
    m.local_map = SimpleNamespace(observable=Subject)
    m.premap = local_map([[0, 0, 0]])
    m._start_matching()

    def send(x):
        points = np.asarray([[x, 0, 0]], dtype=np.float32)
        scans.on_next(PointCloud2.from_numpy(points, frame_id="base_link", timestamp=float(x)))

    send(1)
    send(2)
    busy.wait(5.0)
    for x in (3, 4, 5):
        send(x)
    release.set()
    wait_for(lambda: len(seen) == 2)
    # Scans placed 1 m along by odometry; the second match takes only the latest window
    assert seen == [[2.0, 3.0], [5.0, 6.0]]


def test_from_matrix_inverse_matches_linalg_inv():
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", [0.1, -0.2, 1.3]).as_matrix()
    T[:3, 3] = [1.5, -2.0, 0.3]
    tf = Transform.from_matrix(T, frame_id="map", child_frame_id="world").inverse()
    assert (tf.frame_id, tf.child_frame_id) == ("world", "map")
    np.testing.assert_allclose(tf.to_matrix(), np.linalg.inv(T), atol=1e-9)


def test_dual_strategy_merges_ports():
    class FakeImpl(PremapModule):
        detections: In[Detection3DArray]

    class Dual(FakeImpl):
        pass

    hints = get_type_hints(Dual)
    assert {"tf", "lidar", "loaded_map", "detections"} <= hints.keys()
    assert Dual.blueprint() is not None


def test_a_map_file_that_is_not_a_recording_is_refused():
    from dimos.mapping.relocalization.utils import load_premap

    with pytest.raises(ValueError, match="recording"):
        load_premap("office.pc2.lcm")


def test_a_premap_mcap_decodes_its_cloud_past_unrelated_name_clashes(tmp_path):
    mcap_writer = pytest.importorskip("mcap.writer", reason="mcap not installed")
    from dimos.mapping.relocalization.utils import load_premap

    path = tmp_path / "premap.mcap"
    cloud = PointCloud2.from_numpy(np.eye(3, dtype=np.float32), frame_id="map/v1", timestamp=1.0)
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start()
        payload_type = {"dimos.payload_type": "dimos.msgs.sensor_msgs.PointCloud2.PointCloud2"}
        channel = writer.register_channel("/map/v1/cloud", "lcm", 0, payload_type)
        writer.add_message(channel, 1_000_000_000, cloud.lcm_encode(), 1_000_000_000)
        # Two camera topics that fold to one stream name
        for topic in ("rt/camera_info", "camera_info"):
            clash = writer.register_channel(topic, "cdr", 0)
            writer.add_message(clash, 1_000_000_000, b"\0", 1_000_000_000)
        writer.finish()

    loaded = load_premap(str(path))
    assert isinstance(loaded, PointCloud2)
    assert np.allclose(loaded.as_numpy()[0], np.eye(3))


def test_sparse_clouds_are_logged_once_per_change_and_warn_once_past_the_grace(module, mocker):
    m = module(live_cloud="local_map", min_local_points=3, sparse_cloud_grace_s=10.0)
    logger = mocker.patch.object(relocalization, "logger")
    clock = mocker.patch.object(relocalization, "time").monotonic
    sparse = PointCloud2.from_numpy(np.zeros((2, 3), dtype=np.float32))
    ready = PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32))

    for now, cloud in [
        (100.0, ready),
        (101.0, sparse),
        (105.0, sparse),
        (111.0, sparse),
        (120.0, sparse),
        (121.0, ready),
        (122.0, ready),
        (123.0, sparse),
        (130.0, sparse),
    ]:
        clock.return_value = now
        m._log_point_count_change(cloud)

    sparse_fields = {"n_pts": 2, "min_local_points": 3}
    skipped = mocker.call.info(
        "relocalize skipped until there are enough local points", **sparse_fields
    )
    still = mocker.call.warning(
        "relocalize still skipped after 10s: not enough local points", **sparse_fields
    )
    resumed = mocker.call.info(
        "relocalize resumed: enough local points", n_pts=3, min_local_points=3
    )
    # A new sparse spell gets its own grace: 130 is only 7 s into it
    assert logger.method_calls == [skipped, still, resumed, skipped]


PREMAP = 1


def local_map(points, flags=None):
    points = np.asarray(points, dtype=np.float32)
    flags = (
        np.zeros(len(points), dtype=np.uint32)
        if flags is None
        else np.asarray(flags, dtype=np.uint32)
    )
    return PointCloud2.from_numpy(points, frame_id="world", flags=flags)


def held(live):
    return sorted(map(tuple, live.cloud("world", None, 0).points_f32().round(2).tolist()))


def test_live_map_keeps_only_live_voxels():
    live = LiveMap(0.1)
    live.fold(local_map([[1, 0, 0], [2, 0, 0], [3, 0, 0]], [0, PREMAP, 0]))
    assert held(live) == [(1, 0, 0), (3, 0, 0)]


def test_a_local_map_without_the_premap_bit_is_an_error():
    cloud = PointCloud2.from_numpy(np.asarray([[1, 0, 0]], dtype=np.float32), frame_id="world")
    with pytest.raises(ValueError, match="premap"):
        LiveMap(0.1).fold(cloud)


def test_one_point_per_voxel():
    live = LiveMap(1.0)
    live.fold(local_map([[1.2, 0.2, 0.2], [1.4, 0.4, 0.4]]))
    assert len(live) == 1


def test_the_match_cloud_keeps_near_the_robot():
    live = LiveMap(0.1)
    live.fold(local_map([[1, 0, 0], [50, 0, 0]]))
    near = live.cloud("world", (0.0, 0.0), 10.0).points_f32().tolist()
    assert near == [[1, 0, 0]]


def _room(rng):
    """Points on the walls, floor and a few boxes of a 12 x 8 m room."""
    faces = []
    for _ in range(6000):
        u, v = rng.uniform(0, 1, 2)
        faces.append(
            [
                [u * 12, v * 8, 0],
                [0, u * 8, v * 3],
                [12, u * 8, v * 3],
                [u * 12, 0, v * 3],
                [u * 12, 8, v * 3],
                [2 + u, 2 + v, 1],
                [7 + 2 * u, 5, v * 2],
            ][rng.integers(7)]
        )
    return np.asarray(faces, dtype=np.float32)


def test_a_near_fix_is_refined_by_icp_alone():
    rng = np.random.default_rng(0)
    room = _room(rng)
    relocalizer = CloudRelocalizer(PointCloud2.from_numpy(room).pointcloud, MID360)
    # The live world sits 0.3 m and 3 deg off the map; the current fix is off by that much.
    truth = Transform(
        translation=Vector3(0.3, -0.2, 0.0),
        rotation=Quaternion.from_euler(Vector3(0.0, 0.0, np.radians(3.0))),
        frame_id="world",
        child_frame_id="map",
    )
    world_room = (truth.to_matrix() @ np.c_[room, np.ones(len(room))].T).T[:, :3]
    live = PointCloud2.from_numpy(world_room.astype(np.float32)).pointcloud
    near = Transform(frame_id="world", child_frame_id="map")
    fix = relocalizer.attempt(live, "world", "map", near).fix
    assert fix is not None
    assert (fix.translation - truth.translation).length() < 0.05


def test_world_to_map_waits_for_the_first_fix(module):
    m = module()
    sent = []
    m.tf = SimpleNamespace(publish=sent.append)
    m.publish_tf()
    assert sent == []
    m.submit(Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map"))
    m.publish_tf()
    (fix,) = sent[-1].transforms
    assert (fix.frame_id, fix.child_frame_id) == ("world", "map")


def test_tf_goes_out_at_start_with_every_scan_at_its_stamp_and_on_every_fix(module):
    """world -> map is on tf from the start; each scan carries it stamped like the scan, for stamped lookups."""
    from reactivex import Subject

    m = module(map_frame="premap")
    sent, scans = [], Subject()
    m.tf = SimpleNamespace(publish=sent.append)
    m.lidar = SimpleNamespace(observable=lambda: scans)
    m._start_tf()

    def edges():
        return [(t.frame_id, t.child_frame_id) for t in sent[-1].transforms]

    assert len(sent) == 1 and edges() == [("world", "map")]
    scans.on_next(PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32), timestamp=5.0))
    assert len(sent) == 2 and edges() == [("world", "map")]
    assert sent[-1].transforms[0].ts == 5.0
    m.submit(_fix(2.0))
    assert len(sent) == 3 and edges() == [("world", "map"), ("world", "premap")]
    scans.on_next(PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32), timestamp=6.0))
    assert [t.ts for t in sent[-1].transforms] == [6.0, 6.0]


def _fix(x, yaw_deg=0.0, frame_id="world", child_frame_id="premap"):
    return Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=Quaternion.from_euler(Vector3(0.0, 0.0, np.radians(yaw_deg))),
        frame_id=frame_id,
        child_frame_id=child_frame_id,
    )


def test_the_premap_is_placed_once_and_later_fixes_move_world_to_map(module):
    m = module(map_frame="premap")
    sent = []
    m.tf = SimpleNamespace(publish=sent.append)

    def edges():
        m.publish_tf()
        return {(t.frame_id, t.child_frame_id): t for t in sent[-1].transforms}

    assert list(edges()) == [("world", "map")]
    m.submit(_fix(2.0, 90.0))
    placed = edges()
    np.testing.assert_allclose(placed[("world", "map")].to_matrix(), np.eye(4), atol=1e-9)
    np.testing.assert_allclose(placed[("world", "premap")].to_matrix(), _fix(2.0, 90.0).to_matrix())
    # A refinement (map -> premap, world -> map still identity) moves world -> map only
    m.submit(_fix(2.1, 91.0))
    refined = edges()
    np.testing.assert_allclose(
        refined[("world", "premap")].to_matrix(), _fix(2.0, 90.0).to_matrix()
    )
    chain = refined[("world", "map")] + _fix(2.1, 91.0, "map")
    np.testing.assert_allclose(chain.to_matrix(), _fix(2.0, 90.0).to_matrix(), atol=1e-9)


def test_local_map_is_kept_in_map(module):
    """The ray tracer maps in world; its voxels are kept in map, the odometry frame no fix moves."""
    m = module(map_frame="premap", relocalize_once=False)
    m.tf = SimpleNamespace(publish=lambda _: None)
    m.submit(_fix(2.0))
    m.submit(_fix(2.5))  # world -> map is now -0.5
    m._on_local_map(local_map([[1, 0, 0]]))
    assert held(m.live) == [(1.5, 0, 0)]


def test_a_cloud_in_the_map_frame_is_refined_into_a_world_fix(module, monkeypatch):
    """The kept live map is in map, so its fix is map -> premap, and world -> map follows it."""

    m = module(map_frame="premap", relocalize_once=False)
    m.relocalization_strength = SimpleNamespace(publish=lambda _: None)
    m.tf = SimpleNamespace(publish=lambda _: None)
    m.submit(_fix(2.0))
    m.submit(_fix(2.5))
    seen = []

    class FakeRelocalizer:
        def attempt(self, cloud, cloud_frame, map_frame, initial_fix=None):
            seen.append((cloud_frame, initial_fix))
            found = Transform(
                translation=Vector3(2.4, 0.0, 0.0), frame_id=cloud_frame, child_frame_id=map_frame
            )
            return SimpleNamespace(fix=found, result=SimpleNamespace(fitness=0.9))

    m._relocalizer = FakeRelocalizer()
    m._relocalize(PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), frame_id="map"))
    ((cloud_frame, initial_fix),) = seen
    # ICP starts from the latest map -> premap fix
    assert cloud_frame == "map"
    np.testing.assert_allclose(initial_fix.translation.x, 2.5)
    # The premap stays put; world -> map = world -> premap (2.0) - map -> premap (2.4)
    np.testing.assert_allclose(m.fix.translation.x, 2.0)
    np.testing.assert_allclose(m._world_to_map().translation.x, -0.4)


def test_refused_refinements_fall_back_to_a_global_search(module):
    """An error ICP cannot reach (a bad first fix, a jump) is found again by searching the whole premap."""
    m = module(map_frame="premap", relocalize_once=False, global_after_refusals=2)
    m.relocalization_strength = SimpleNamespace(publish=lambda _: None)
    m.tf = SimpleNamespace(publish=lambda _: None)
    m.submit(_fix(2.0))
    seen = []
    right = _fix(5.0, frame_id="map")

    class FakeRelocalizer:
        def attempt(self, cloud, cloud_frame, map_frame, initial_fix=None):
            seen.append(initial_fix is None)
            found = right if initial_fix is None else None
            return SimpleNamespace(fix=found, result=SimpleNamespace(fitness=0.1))

    m._relocalizer = FakeRelocalizer()
    cloud = PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), frame_id="map")
    for _ in range(4):
        m._relocalize(cloud)
    # two refused refinements, then a global search that is accepted, then refining again
    assert seen == [False, False, True, False]
    np.testing.assert_allclose(m._world_to_map().translation.x, -3.0)


def test_a_refinement_moves_the_fix_part_way_and_a_global_search_whole(module):
    m = module(map_frame="premap", relocalize_once=False, fix_smoothing_s=2.0)
    m.relocalization_strength = SimpleNamespace(publish=lambda _: None)
    m.tf = SimpleNamespace(publish=lambda _: None)
    searches = iter([2.0, 5.0])

    class FakeRelocalizer:
        def attempt(self, cloud, cloud_frame, map_frame, initial_fix=None):
            if initial_fix is None:
                found = _fix(next(searches), frame_id="map")
            else:
                found = _fix(3.0, 10.0, frame_id="map")
            return SimpleNamespace(fix=found, result=SimpleNamespace(fitness=0.9))

    m._relocalizer = FakeRelocalizer()

    def cloud(ts):
        points = np.zeros((3, 3), dtype=np.float32)
        return PointCloud2.from_numpy(points, frame_id="map", timestamp=ts)

    def map_fix():
        return m._map_fix.translation.x, np.degrees(m._map_fix.rotation.euler.z)

    m._relocalize(cloud(100.0))
    np.testing.assert_allclose(map_fix(), (2.0, 0.0), atol=1e-9)
    # One time constant later a refinement 1 m and 10 deg off moves the fix 1 - 1/e of the way
    m._relocalize(cloud(102.0))
    np.testing.assert_allclose(
        map_fix(), (3.0 - np.exp(-1.0), 10.0 * (1 - np.exp(-1.0))), atol=1e-6
    )
    # Lost, the next match searches the whole premap and its fix applies whole
    m._refusals = m.config.global_after_refusals
    m._relocalize(cloud(102.1))
    np.testing.assert_allclose(map_fix(), (5.0, 0.0), atol=1e-9)


def test_scans_are_kept_in_map_whatever_world_to_map_does(module):
    """A body scan is placed by map -> base_link at its stamp, so a world -> map jump moves nothing kept."""
    m = module(map_frame="premap", relocalize_once=False, live_cloud="lidar")
    m.tf = SimpleNamespace(publish=lambda _: None)
    odom = Transform(translation=Vector3(1.0, 0.0, 0.0), frame_id="map", child_frame_id="base_link")
    m._tf = SimpleNamespace(
        get=lambda parent, child, *_: odom if (parent, child) == ("map", "base_link") else None,
        dispose=lambda: None,
    )
    scan = PointCloud2.from_numpy(np.asarray([[2, 0, 0]], dtype=np.float32), frame_id="base_link")
    m.submit(_fix(2.0))
    m.submit(_fix(5.0, 90.0))  # world -> map jumps
    placed = m._place_scan(scan)
    assert placed.frame_id == "map"
    assert placed.points_f32().tolist() == [[3, 0, 0]]


def test_clouds_without_a_transform_are_skipped(module):
    m = module(map_frame="premap", relocalize_once=False, live_cloud="lidar")
    m._tf = SimpleNamespace(get=lambda *_: None, dispose=lambda: None)
    scan = PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32), frame_id="base_link")
    assert m._place_scan(scan) is None
    attempts = []
    m._relocalizer = SimpleNamespace(attempt=lambda *args: attempts.append(args))
    m._relocalize(PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), frame_id="nowhere"))
    assert attempts == []
    assert m._refusals == 0


def test_folded_scans_are_kept_in_map_whatever_world_to_map_does(module):
    """lidar_live_map: a body scan folds into the live map placed by map -> base_link at its stamp."""
    m = module(map_frame="premap", relocalize_once=False, live_cloud="lidar", lidar_live_map=True)
    m.tf = SimpleNamespace(publish=lambda _: None)
    odom = Transform(translation=Vector3(1.0, 0.0, 0.0), frame_id="map", child_frame_id="base_link")
    m._tf = SimpleNamespace(
        get=lambda parent, child, *_: odom if (parent, child) == ("map", "base_link") else None,
        dispose=lambda: None,
    )
    scan = PointCloud2.from_numpy(np.asarray([[2, 0, 0]], dtype=np.float32), frame_id="base_link")
    m.submit(_fix(2.0))
    m.submit(_fix(5.0, 90.0))  # world -> map jumps
    m._on_scan(scan)
    m._keep_scans()
    assert held(m.live) == [(3, 0, 0)]
    assert m._center == (1.0, 0.0)


def _scan_from(frame: str) -> PointCloud2:
    return PointCloud2.from_numpy(np.asarray([[2, 0, 0]], dtype=np.float32), frame_id=frame)


@pytest.mark.parametrize(
    ("attempt_after_frames", "frames", "attempts"),
    [
        # every scan starts an attempt by default
        ([], ["front", "rear", "front"], 3),
        # one attempt per sweep of both lidars, whichever lands first, and a lidar seen twice still waits for the other
        (["front", "rear"], ["front", "rear", "rear", "front", "front"], 2),
    ],
)
def test_an_attempt_waits_for_a_scan_from_every_configured_frame(
    module, attempt_after_frames, frames, attempts
):
    m = module(
        map_frame="premap",
        relocalize_once=False,
        live_cloud="lidar",
        lidar_live_map=True,
        attempt_after_frames=attempt_after_frames,
    )
    m._tf = SimpleNamespace(
        get=lambda *_: Transform(frame_id="map", child_frame_id="x"), dispose=lambda: None
    )
    changes = []
    m._live_changed.subscribe(changes.append)
    for frame in frames:
        m._on_scan(_scan_from(frame))
        m._keep_scans()
    assert len(changes) == attempts
    # every scan still joins the live map
    assert len(held(m.live)) == 1


def test_an_attempt_comes_on_every_nth_complete_sweep(module):
    m = module(
        map_frame="premap",
        relocalize_once=False,
        live_cloud="lidar",
        lidar_live_map=True,
        attempt_after_frames=["front", "rear"],
        attempt_every_sweeps=3,
    )
    m._tf = SimpleNamespace(
        get=lambda *_: Transform(frame_id="map", child_frame_id="x"), dispose=lambda: None
    )
    changes = []
    m._live_changed.subscribe(changes.append)
    for _ in range(7):
        for frame in ("front", "rear"):
            m._on_scan(_scan_from(frame))
            m._keep_scans()
    # sweeps 3 and 6 of 7
    assert len(changes) == 2
