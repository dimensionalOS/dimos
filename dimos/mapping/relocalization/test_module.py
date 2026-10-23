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

from contextlib import ExitStack
from threading import Event
from types import SimpleNamespace
from typing import get_type_hints

from dimos_cdrgen.registry import encode
import numpy as np
import open3d as o3d
import pydantic
import pytest
from reactivex import Subject
from reactivex.testing import TestScheduler
from scipy.spatial.transform import Rotation

from dimos.core.stream import In
from dimos.core.transport import LCMTransport
from dimos.mapping.relocalization.go2.module import Go2Relocalization
from dimos.mapping.relocalization.lidar import relocalize as lidar
from dimos.mapping.relocalization.lidar.module import LidarWindowRelocalization, window
from dimos.mapping.relocalization.lidar.relocalize import MID360
from dimos.mapping.relocalization.module import RelocalizationModule, fix_stream
from dimos.message_utils.geometry import inverse_transform, transform_from_matrix, transform_matrix
from dimos.message_utils.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.message_utils.time import time_from_nanoseconds
from dimos.messages.geometry_msgs.msg import TransformStamped
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.messages.vision_msgs.msg import Detection3DArray
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def _transform(matrix, *, frame_id, child_frame_id):
    return TransformStamped(
        header=Header(stamp=time_from_nanoseconds(0), frame_id=frame_id),
        child_frame_id=child_frame_id,
        transform=transform_from_matrix(matrix),
    )


@pytest.fixture
def module():
    """Build a real module (and dispose it); `cls, **config` picks the class."""
    built = []

    def build(cls=RelocalizationModule, **config):
        m = cls(rpc_transport=MemoryRPC, **config)
        built.append(m)
        return m

    yield build
    for m in built:
        m.stop()


def fixes(m):
    """Collect the transforms a module accepts."""
    got = []
    m.register_disposable(m.fixes.subscribe(got.append))
    return got


def test_submit_publishes_and_checks_frames(module):
    """submit does not second-guess the fix; the implementation already decided."""
    m = module()
    got = fixes(m)
    tf = _transform(np.eye(4), frame_id="world", child_frame_id="map")
    m.submit(tf, "x")
    m.submit(tf, "x")
    assert got == [tf, tf]
    # A strategy handing over the placement instead of the frame transform is
    # caught here rather than publishing a backwards TF.
    with pytest.raises(AssertionError):
        m.submit(_transform(np.eye(4), frame_id="map", child_frame_id="world"))


def test_relocalize_once_stops_after_the_first_fix(module):
    """What the flag does, either way. Which one is the default is a policy call."""
    tf = _transform(np.eye(4), frame_id="world", child_frame_id="map")

    once = module(relocalize_once=True)
    assert once.keep_relocalizing() and not once.placed
    once.submit(tf)
    assert once.placed and not once.keep_relocalizing()

    forever = module(relocalize_once=False)
    forever.submit(tf)
    assert forever.placed and forever.keep_relocalizing()


@pytest.mark.parametrize("interval", [3600.0, 0.0])
def test_the_fix_goes_out_the_moment_it_is_accepted(interval):
    """Not on the next interval tick, and with or without republishing (<= 0 = once per fix)."""

    fixes, sent = Subject(), []
    disposable = fix_stream(fixes, interval=interval).subscribe(sent.append)
    tf = _transform(np.eye(4), frame_id="world", child_frame_id="map")
    fixes.on_next(tf)
    assert sent == [tf]
    disposable.dispose()


def test_premap_defines_the_map_frame_and_waits_for_a_fix(module, tmp_path, mocker):
    """Loading is the base's: every strategy reads a premap and publishes it, once placed."""
    path = tmp_path / "somewhere.pc2.cdr"
    path.write_bytes(
        encode(
            pointcloud_from_xyz(
                np.zeros((5, 3), dtype=np.float32),
                header=Header(stamp=time_from_nanoseconds(123456789), frame_id="original"),
            )
        )
    )
    m = module()
    published = mocker.patch.object(m.loaded_map, "publish")
    m._load_premap(str(path))
    assert m.premap is not None and m.premap.width * m.premap.height == 5
    assert m.premap.header.frame_id == "map"
    assert m.premap.header.stamp == time_from_nanoseconds(123456789)
    published.assert_not_called()
    m.submit(_transform(np.eye(4), frame_id="world", child_frame_id="map"))
    published.assert_called_once_with(m.premap)


def test_relocalizer_refuses_below_its_own_threshold(monkeypatch):
    """One config surface: the relocalizer holds the knobs and the accept decision."""

    placement = np.eye(4)
    placement[:3, 3] = [3.0, -1.0, 0.0]  # the map is 3 m +x of where the robot thought
    result = SimpleNamespace(transformation=placement, fitness=0.4, inlier_rmse=0.1)
    monkeypatch.setattr(lidar.LidarRelocalizer, "_prepare", lambda self, cloud: None)
    monkeypatch.setattr(lidar.LidarRelocalizer, "align", lambda self, cloud: result)

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
    assert (tf.header.frame_id, tf.child_frame_id) == ("world", "map")
    np.testing.assert_allclose(transform_matrix(tf.transform), np.linalg.inv(placement), atol=1e-9)


def test_no_config_without_naming_a_rig():
    """The scales are per-rig, so there is no bare RelocalizeConfig() to fall into."""

    assert lidar.DEFAULT_PRESET in lidar.PRESETS
    assert lidar.PRESETS["mid360"] is lidar.MID360
    with pytest.raises(pydantic.ValidationError):
        lidar.RelocalizeConfig()


def test_the_match_runs_on_a_window_of_the_last_scans():
    """Every scan enters the window; a match sees the last `max_frames` of them."""

    cfg = MID360.model_copy(update={"min_frames": 2, "max_frames": 3})
    scans = Subject()
    matched = []
    scheduler = TestScheduler()
    disposable = window(scans, cfg, interval=0.001).subscribe(matched.append, scheduler=scheduler)
    try:
        for i in range(5):
            scans.on_next(
                pointcloud_from_xyz(
                    np.full((4, 3), i, dtype=np.float32),
                    header=Header(stamp=time_from_nanoseconds(i), frame_id="world"),
                )
            )
            scheduler.advance_by(0.01)
        assert [c.width * c.height for c in matched] == [8, 12, 12, 12]
        assert set(pointcloud_xyz(matched[-1])[:, 0]) == {2.0, 3.0, 4.0}
    finally:
        disposable.dispose()


def test_from_matrix_inverse_matches_linalg_inv():
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", [0.1, -0.2, 1.3]).as_matrix()
    T[:3, 3] = [1.5, -2.0, 0.3]
    tf = inverse_transform(_transform(T, frame_id="map", child_frame_id="world"))
    assert (tf.header.frame_id, tf.child_frame_id) == ("world", "map")
    np.testing.assert_allclose(transform_matrix(tf.transform), np.linalg.inv(T), atol=1e-9)


def test_dual_strategy_merges_ports():
    class FakeImpl(RelocalizationModule):
        detections: In[Detection3DArray]

    class Dual(LidarWindowRelocalization, FakeImpl):
        pass

    hints = get_type_hints(Dual)
    assert {"tf", "lidar", "loaded_map", "detections"} <= hints.keys()
    assert Dual.blueprint() is not None


def test_real_cdr_premap_and_fix_ports_keep_metadata_and_own_published_stamp(
    module, tmp_path, mocker
):
    path = tmp_path / "canonical.pc2.cdr"
    cloud = pointcloud_from_xyz(
        np.array([[1, 2, 3]], dtype=np.float32),
        header=Header(frame_id="exported", stamp=time_from_nanoseconds(123456789)),
    )
    payload = encode(cloud)
    path.write_bytes(payload)
    instance = module(map_file=str(tmp_path / "canonical"), tf_interval=0)
    stamp = 1700000000123456789
    mocker.patch("dimos.message_utils.time.time.time_ns", return_value=stamp)
    with ExitStack() as cleanup:
        received, ready = {}, {}

        def collect(name):
            def receive(value):
                received[name] = value
                ready[name].set()

            return receive

        for name, dtype in (("tf", TFMessage), ("loaded_map", PointCloud2)):
            assert getattr(instance, name).type is dtype
            transport = LCMTransport("relocalization_" + name, dtype, url="memq://")
            cleanup.callback(transport.stop)
            getattr(instance, name).transport = transport
            ready[name] = Event()
            cleanup.callback(transport.subscribe(collect(name)))
        cleanup.callback(instance.stop)
        instance.start()
        assert not received
        fix = _transform(np.eye(4), frame_id="world", child_frame_id="map")
        before = encode(fix)
        instance.submit(fix)
        assert ready["tf"].wait(5) and ready["loaded_map"].wait(5)
        [sent] = received["tf"].transforms
        assert sent.header.stamp == time_from_nanoseconds(stamp)
        assert sent.header.frame_id == "world" and sent.child_frame_id == "map"
        loaded = received["loaded_map"]
        assert loaded.header.frame_id == "map" and loaded.header.stamp == cloud.header.stamp
        np.testing.assert_array_equal(pointcloud_xyz(loaded), [[1, 2, 3]])
        assert encode(fix) == before and path.read_bytes() == payload


@pytest.mark.parametrize("carving,count", [(True, 2), (False, 4)])
def test_go2_merge_preserves_map_placement_and_carving_policy(module, mocker, carving, count):
    # Exercise the real CPU carving backend independently of host GPU availability.
    mocker.patch("open3d.core.cuda.is_available", return_value=False)
    instance = module(Go2Relocalization, use_carving=carving)
    premap = pointcloud_from_xyz(
        np.array([[0.025, 0.025, 0.025], [0.025, 0.025, 0.075]], dtype=np.float32),
        header=Header(frame_id="map", stamp=time_from_nanoseconds(100)),
    )
    local = pointcloud_from_xyz(
        np.array([[1.025, 0.025, 0.125], [2.025, 0.025, 0.025]], dtype=np.float32),
        header=Header(frame_id="world", stamp=time_from_nanoseconds(101)),
    )
    instance.premap = premap
    publish = mocker.patch.object(instance.merged_map, "publish")
    matrix = np.eye(4)
    matrix[0, 3] = 1
    fix = _transform(matrix, frame_id="world", child_frame_id="map")
    originals = encode(premap), encode(local), encode(fix)
    instance._on_merge_input((local, fix))
    publish.assert_called_once()
    [result] = publish.call_args.args
    assert result.width * result.height == count
    assert result.header == local.header
    points = pointcloud_xyz(result)
    assert set(np.round(points[:, 0], 3)) == {1.025, 2.025}
    if carving:
        np.testing.assert_allclose(points[points[:, 0] < 1.5, 2], [0.125], atol=1e-6)
    assert (encode(premap), encode(local), encode(fix)) == originals


def test_real_preparation_produces_features_for_every_coarse_point():
    rng = np.random.default_rng(17)
    points = rng.uniform(-3, 3, size=(500, 3))
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    relocalizer = lidar.LidarRelocalizer(cloud, lidar.MID360)
    prepared = relocalizer._prepare(cloud)
    assert len(prepared.coarse.points) > 50 and len(prepared.fine.points) > 50
    assert prepared.fpfh.dimension() == 33
    assert prepared.fpfh.num() == len(prepared.coarse.points)
    assert np.isfinite(np.asarray(prepared.fpfh.data)).all()
