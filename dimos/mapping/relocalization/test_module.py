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

from types import SimpleNamespace
from typing import get_type_hints

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import TransformStamped
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection3DArray
from dimos_message_build.registry import encode as cdr_encode
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.core.stream import In
from dimos.core.transport_factory import rpc_backend
from dimos.mapping.relocalization.lidar.module import LidarWindowRelocalization
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.msgs.geometry import inverse_transform, transform_from_matrix, transform_matrix
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz


@pytest.fixture
def module(mocker):
    """Build a real module (and dispose it); `cls, **config` picks the class."""
    backend = rpc_backend()
    for method in ("start", "serve_module_rpc", "stop"):
        mocker.patch.object(backend, method)
    built = []

    def build(cls=RelocalizationModule, **config):
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
    tf = TransformStamped(
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="map",
        transform=transform_from_matrix(np.eye(4)),
    )
    m.submit(tf, "x")
    m.submit(tf, "x")
    assert got == [tf, tf]
    # A strategy handing over the placement instead of the frame transform is
    # caught here rather than publishing a backwards TF.
    with pytest.raises(AssertionError):
        m.submit(
            TransformStamped(
                header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
                child_frame_id="world",
                transform=transform_from_matrix(np.eye(4)),
            )
        )


def test_relocalize_once_stops_after_the_first_fix(module):
    """What the flag does, either way. Which one is the default is a policy call."""
    tf = TransformStamped(
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="map",
        transform=transform_from_matrix(np.eye(4)),
    )

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
    from reactivex import Subject

    from dimos.mapping.relocalization.module import fix_stream

    fixes, sent = Subject(), []
    disposable = fix_stream(fixes, interval=interval).subscribe(sent.append)
    tf = TransformStamped(
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="map",
        transform=transform_from_matrix(np.eye(4)),
    )
    fixes.on_next(tf)
    assert sent == [tf]
    disposable.dispose()


def test_premap_defines_the_map_frame_and_waits_for_a_fix(module, tmp_path):
    """Loading is the base's: every strategy reads a premap and publishes it, once placed."""
    path = tmp_path / "somewhere.pc2.cdr"
    path.write_bytes(
        cdr_encode(
            pointcloud_from_xyz(
                np.zeros((5, 3), dtype=np.float32),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
        )
    )
    m = module()
    published, disposables = [], []
    # The one collaborator worth faking: a real Out port would publish onto a
    # bus nothing in this test is listening to.
    m.loaded_map = SimpleNamespace(publish=published.append)
    m.register_disposable = disposables.append

    m._load_premap(str(path))
    assert m.premap is not None and m.premap.width * m.premap.height == 5
    assert m.premap.header.frame_id == "map"
    assert len(disposables) == 1  # the gated publish
    assert published == []  # ... which stays silent until a fix lands
    m.submit(
        TransformStamped(
            header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
            child_frame_id="map",
            transform=transform_from_matrix(np.eye(4)),
        )
    )
    assert published == [m.premap]  # republish_loaded_map=0: once, on that fix
    disposables[0].dispose()


def test_relocalizer_refuses_below_its_own_threshold(monkeypatch):
    """One config surface: the relocalizer holds the knobs and the accept decision."""
    from dimos.mapping.relocalization.lidar import relocalize as lidar

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
    import pydantic

    from dimos.mapping.relocalization.lidar import relocalize as lidar

    assert lidar.DEFAULT_PRESET in lidar.PRESETS
    assert lidar.PRESETS["mid360"] is lidar.MID360
    with pytest.raises(pydantic.ValidationError):
        lidar.RelocalizeConfig()


def test_the_match_runs_on_a_window_of_the_last_scans():
    """Every scan enters the window; a match sees the last `max_frames` of them."""
    import time

    from reactivex import Subject

    from dimos.mapping.relocalization.lidar.module import window
    from dimos.mapping.relocalization.lidar.relocalize import MID360

    cfg = MID360.model_copy(update={"min_frames": 2, "max_frames": 3})
    scans = Subject()
    matched = []
    window(scans, cfg, interval=0.001).subscribe(matched.append)

    for i in range(5):
        scans.on_next(
            pointcloud_from_xyz(
                np.full((4, 3), i, dtype=np.float32),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
        )
        time.sleep(0.01)  # clear the throttle, so every scan gets its attempt

    # The first scan is below min_frames; from then on the window is full.
    assert [c.width * c.height for c in matched] == [8, 12, 12, 12]
    # ... and holds the *last* three scans, not the first.
    assert set(pointcloud_xyz(matched[-1])[:, 0]) == {2.0, 3.0, 4.0}


def test_from_matrix_inverse_matches_linalg_inv():
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", [0.1, -0.2, 1.3]).as_matrix()
    T[:3, 3] = [1.5, -2.0, 0.3]
    tf = inverse_transform(
        TransformStamped(
            header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
            child_frame_id="world",
            transform=transform_from_matrix(T),
        )
    )
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
