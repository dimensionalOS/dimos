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
from dimos.mapping.relocalization.lidar_relocalizer import MID360, LidarRelocalizer
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

    monkeypatch.setattr(relocalization, "LidarRelocalizer", FakeRelocalizer)
    # Synchronous, so the attempts land before the assert and no worker thread is left behind
    monkeypatch.setattr(relocalization, "backpressure", lambda clouds: clouds)
    tf = Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map")
    cloud = PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32))
    for once in (True, False):
        m = module(live_cloud="lidar", relocalize_once=once, min_local_points=0)
        m.relocalization_strength = SimpleNamespace(publish=strengths.append)
        clouds = Subject()
        m._clouds = lambda clouds=clouds: clouds
        m.premap = cloud
        m._start_matching()
        m.submit(tf)
        clouds.on_next(cloud)
    assert attempts == [tf]
    # Refused or not, every attempt's fitness goes out
    assert [s.data for s in strengths] == [0.5]


@pytest.mark.parametrize("interval", [3600.0, 0.0])
def test_the_fix_goes_out_the_moment_it_is_accepted(interval):
    """Not on the next interval tick, and with or without republishing (<= 0 = once per fix)."""
    from reactivex import Subject

    from dimos.mapping.relocalization.utils import fix_stream

    fixes, sent = Subject(), []
    disposable = fix_stream(fixes, interval=interval).subscribe(sent.append)
    tf = Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map")
    fixes.on_next(tf)
    assert sent == [tf]
    disposable.dispose()


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
    assert published == [m.premap]  # republish_loaded_map=0: once, on that fix
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
    from dimos.mapping.relocalization import lidar_relocalizer as lidar

    placement = np.eye(4)
    placement[:3, 3] = [3.0, -1.0, 0.0]  # the map is 3 m +x of where the robot thought
    result = SimpleNamespace(transformation=placement, fitness=0.4, inlier_rmse=0.1)
    monkeypatch.setattr(lidar.LidarRelocalizer, "_prepare", lambda self, cloud: None)
    monkeypatch.setattr(lidar.LidarRelocalizer, "align", lambda self, cloud, initial=None: result)

    def relocalizer(threshold):
        return lidar.LidarRelocalizer(
            None, lidar.MID360.model_copy(update={"fitness_threshold": threshold})
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

    from dimos.mapping.relocalization import lidar_relocalizer as lidar

    assert lidar.DEFAULT_PRESET in lidar.PRESETS
    assert lidar.PRESETS["mid360"] is lidar.MID360
    with pytest.raises(pydantic.ValidationError):
        lidar.RelocalizeConfig()


def test_the_match_runs_on_a_window_of_the_last_scans():
    """Every scan enters the window; a match sees the last `max_frames` of them."""

    from reactivex import Subject

    from dimos.mapping.relocalization.lidar_relocalizer import MID360
    from dimos.mapping.relocalization.utils import window

    cfg = MID360.model_copy(update={"min_frames": 2, "max_frames": 3})
    scans = Subject()
    matched = []
    window(scans, cfg, interval=0.001).subscribe(matched.append)

    for i in range(5):
        scans.on_next(PointCloud2.from_numpy(np.full((4, 3), i, dtype=np.float32), timestamp=0.0))
        time.sleep(0.01)  # clear the throttle, so every scan gets its attempt

    # The first scan is below min_frames; from then on the window is full.
    assert [len(c) for c in matched] == [8, 12, 12, 12]
    # ... and holds the *last* three scans, not the first.
    assert set(matched[-1].points_f32()[:, 0]) == {2.0, 3.0, 4.0}


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


@pytest.fixture
def sparse_module(module):
    return module(live_cloud="local_map", min_local_points=3)


def test_sparse_clouds_warn_only_after_grace_and_remain_throttled(sparse_module, mocker):
    logger = mocker.patch.object(relocalization, "logger")
    clock = mocker.patch.object(relocalization.time, "monotonic")
    cloud = PointCloud2.from_numpy(np.zeros((2, 3), dtype=np.float32))

    for now in (100.0, 101.0, 105.0, 109.999, 110.0, 111.0, 115.0):
        clock.return_value = now
        sparse_module._maybe_log_skip(cloud)

    message = "relocalize skipped: not enough local points"
    fields = {"n_pts": 2, "min_local_points": 3}
    assert logger.method_calls == [
        mocker.call.info(message, **fields),
        mocker.call.info(message, **fields),
        mocker.call.warning(message, **fields),
        mocker.call.warning(message, **fields),
    ]
    assert not sparse_module._has_enough_points(cloud)


def test_enough_points_clear_sparse_cloud_warning_state(sparse_module, mocker):
    logger = mocker.patch.object(relocalization, "logger")
    clock = mocker.patch.object(relocalization.time, "monotonic", return_value=100.0)
    sparse = PointCloud2.from_numpy(np.zeros((2, 3), dtype=np.float32))
    ready = PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32))
    sparse_module._maybe_log_skip(sparse)
    clock.return_value = 105.0
    sparse_module._maybe_log_skip(ready)
    clock.return_value = 120.0
    sparse_module._maybe_log_skip(sparse)

    assert logger.method_calls == [
        mocker.call.info(
            "relocalize skipped: not enough local points", n_pts=2, min_local_points=3
        ),
        mocker.call.info(
            "relocalize skipped: not enough local points", n_pts=2, min_local_points=3
        ),
    ]
    assert sparse_module._has_enough_points(ready)


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
    relocalizer = LidarRelocalizer(PointCloud2.from_numpy(room).pointcloud, MID360)
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


def test_world_to_map_is_always_on_tf_beside_the_premap(module):
    m = module(map_frame="premap", tf_interval=3600.0)
    sent = []
    m.tf = SimpleNamespace(publish=sent.append)
    disposables = []
    m.register_disposable = disposables.append
    m._start_tf()
    for disposable in disposables:
        disposable.dispose()
    edges = [(t.frame_id, t.child_frame_id) for msg in sent for t in msg.transforms]
    assert edges == [("world", "map")]


def test_the_premap_is_placed_once_and_later_fixes_move_world_to_map(module):
    m = module(map_frame="premap")
    sent = []
    m.tf = SimpleNamespace(publish=sent.append)

    def fix(x, yaw_deg):
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion.from_euler(Vector3(0.0, 0.0, np.radians(yaw_deg))),
            frame_id="world",
            child_frame_id="premap",
        )

    def edges():
        m.publish_tf()
        return {(t.frame_id, t.child_frame_id): t for t in sent[-1].transforms}

    assert list(edges()) == [("world", "map")]
    m.submit(fix(2.0, 90.0))
    placed = edges()
    np.testing.assert_allclose(placed[("world", "map")].to_matrix(), np.eye(4), atol=1e-9)
    np.testing.assert_allclose(placed[("map", "premap")].to_matrix(), fix(2.0, 90.0).to_matrix())
    # A refinement moves world -> map only; the chain still lands on the new fix
    m.submit(fix(2.1, 91.0))
    refined = edges()
    np.testing.assert_allclose(refined[("map", "premap")].to_matrix(), fix(2.0, 90.0).to_matrix())
    chain = refined[("world", "map")] + refined[("map", "premap")]
    np.testing.assert_allclose(chain.to_matrix(), fix(2.1, 91.0).to_matrix(), atol=1e-9)


def test_a_cloud_in_the_map_frame_is_refined_into_a_world_fix(module, monkeypatch):
    """The ray tracer maps in map, so its cloud's fix is map -> premap, turned into world -> premap."""

    m = module(map_frame="premap", relocalize_once=False)
    m.relocalization_strength = SimpleNamespace(publish=lambda _: None)
    m.tf = SimpleNamespace(publish=lambda _: None)
    first = Transform(translation=Vector3(2.0, 0.0, 0.0), frame_id="world", child_frame_id="premap")
    m.submit(first)
    m.submit(
        Transform(translation=Vector3(2.5, 0.0, 0.0), frame_id="world", child_frame_id="premap")
    )
    seen = []

    class FakeRelocalizer:
        def attempt(self, cloud, cloud_frame, map_frame, initial_fix=None):
            seen.append((cloud_frame, initial_fix))
            # the map frame is 0.5 m off by now, so the cloud's map -> premap reads 2.4
            found = Transform(
                translation=Vector3(2.4, 0.0, 0.0), frame_id=cloud_frame, child_frame_id=map_frame
            )
            return SimpleNamespace(fix=found, result=SimpleNamespace(fitness=0.9))

    m._relocalizer = FakeRelocalizer()
    m._relocalize(PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), frame_id="map"))
    ((cloud_frame, initial_fix),) = seen
    # ICP starts from the placement (map -> premap), the fix expressed in the cloud's frame
    assert cloud_frame == "map"
    np.testing.assert_allclose(initial_fix.translation.x, 2.0)
    # world -> premap = world -> map (0.5) + map -> premap (2.4)
    np.testing.assert_allclose(m.fix.translation.x, 2.9)
    assert (m.fix.frame_id, m.fix.child_frame_id) == ("world", "premap")


def test_refused_refinements_fall_back_to_a_global_search(module):
    """An error ICP cannot reach (a bad first fix, a jump) is found again by searching the whole premap."""
    m = module(map_frame="premap", relocalize_once=False, global_after_refusals=2)
    m.relocalization_strength = SimpleNamespace(publish=lambda _: None)
    m.tf = SimpleNamespace(publish=lambda _: None)
    m.submit(
        Transform(translation=Vector3(2.0, 0.0, 0.0), frame_id="world", child_frame_id="premap")
    )
    m.live.fold(local_map([[1, 0, 0]]))
    seen = []
    right = Transform(translation=Vector3(5.0, 0.0, 0.0), frame_id="world", child_frame_id="premap")

    class FakeRelocalizer:
        def attempt(self, cloud, cloud_frame, map_frame, initial_fix=None):
            seen.append(initial_fix is None)
            found = right if initial_fix is None else None
            return SimpleNamespace(fix=found, result=SimpleNamespace(fitness=0.1))

    m._relocalizer = FakeRelocalizer()
    cloud = PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), frame_id="world")
    for _ in range(4):
        m._relocalize(cloud)
    # two refused refinements, then a global search that is accepted, then refining again
    assert seen == [False, False, True, False]
    np.testing.assert_allclose(m.fix.translation.x, 5.0)
    # voxels kept under the old frame are dropped, and stay dropped when the ray tracer resends them
    assert len(m.live) == 0
    m.live.fold(local_map([[1, 0, 0], [3, 0, 0]]))
    assert held(m.live) == [(3, 0, 0)]
