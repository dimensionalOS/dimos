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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.core.stream import In
from dimos.mapping.relocalization.lidar.module import LidarWindowRelocalization
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


@pytest.fixture
def module():
    """Build a real module (and dispose it); `cls, **config` picks the class."""
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
    tf = Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map")
    m.submit(tf, "x")
    m.submit(tf, "x")
    assert got == [tf, tf]
    # A strategy handing over the placement instead of the frame transform is
    # caught here rather than publishing a backwards TF.
    with pytest.raises(AssertionError):
        m.submit(Transform.from_matrix(np.eye(4), frame_id="map", child_frame_id="world"))


def test_relocalize_once_stops_after_the_first_fix(module):
    """What the flag does, either way. Which one is the default is a policy call."""
    tf = Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map")

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
    tf = Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map")
    fixes.on_next(tf)
    assert sent == [tf]
    disposable.dispose()


def test_premap_defines_the_map_frame_and_waits_for_a_fix(module, tmp_path):
    """Loading is the base's: every strategy reads a premap and publishes it, once placed."""
    path = tmp_path / "somewhere.pc2.lcm"
    path.write_bytes(
        PointCloud2.from_numpy(np.zeros((5, 3), dtype=np.float32), timestamp=0.0).lcm_encode()
    )
    m = module()
    published, disposables = [], []
    # The one collaborator worth faking: a real Out port would publish onto a
    # bus nothing in this test is listening to.
    m.loaded_map = SimpleNamespace(publish=published.append)
    m.register_disposable = disposables.append

    m._load_premap(str(path))
    assert m.premap is not None and len(m.premap) == 5
    assert m.premap.frame_id == "map"
    assert len(disposables) == 1  # the gated publish
    assert published == []  # ... which stays silent until a fix lands
    m.submit(Transform.from_matrix(np.eye(4), frame_id="world", child_frame_id="map"))
    assert published == [m.premap]  # republish_loaded_map=0: once, on that fix
    disposables[0].dispose()


def test_premap_stem_resolves_from_the_cwd(module, tmp_path, monkeypatch):
    """A bare stem gets the suffix and is looked up beside the caller before the data dir."""
    (tmp_path / "site.pc2.lcm").write_bytes(
        PointCloud2.from_numpy(np.zeros((5, 3), dtype=np.float32), timestamp=0.0).lcm_encode()
    )
    monkeypatch.chdir(tmp_path)
    m = module()
    m.register_disposable = lambda d: None
    m._load_premap("site")
    assert m.premap is not None and len(m.premap) == 5


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

    # Accepted: open3d places the live cloud in the map, the TF tree wants the
    # other direction, and relocalize() is what turns one into the other.
    tf = relocalizer(0.3).relocalize(None, "world", "map")
    assert (tf.frame_id, tf.child_frame_id) == ("world", "map")
    np.testing.assert_allclose(tf.to_matrix(), np.linalg.inv(placement), atol=1e-9)


def _fix(yaw_deg, x=0.0):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    m[0, 3] = x
    return Transform.from_matrix(m, frame_id="world", child_frame_id="map")


def test_a_fix_is_published_only_once_a_second_one_agrees(module):
    """RANSAC lands wrong hypotheses in different places; two agreeing fixes are one place."""
    cloud = PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), timestamp=0.0)
    answers = iter([_fix(0.0), _fix(120.0), _fix(121.0, x=0.05), _fix(120.5)])
    m = module(LidarWindowRelocalization, relocalize_once=True)
    m._relocalizer = SimpleNamespace(relocalize=lambda c, w, mp: next(answers))
    got = fixes(m)

    m._relocalize(cloud)
    m._relocalize(cloud)
    assert got == []  # 0 then 120 degrees: disagree, nothing believed yet
    m._relocalize(cloud)
    assert len(got) == 1 and not m.keep_relocalizing()
    np.testing.assert_allclose(got[0].to_matrix(), _fix(121.0, x=0.05).to_matrix())


def test_confirm_fixes_of_one_publishes_the_first_fix(module):
    cloud = PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32), timestamp=0.0)
    m = module(LidarWindowRelocalization, confirm_fixes=1)
    m._relocalizer = SimpleNamespace(relocalize=lambda c, w, mp: _fix(45.0))
    got = fixes(m)
    m._relocalize(cloud)
    assert len(got) == 1


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
    class FakeImpl(RelocalizationModule):
        detections: In[Detection3DArray]

    class Dual(LidarWindowRelocalization, FakeImpl):
        pass

    hints = get_type_hints(Dual)
    assert {"tf", "lidar", "loaded_map", "detections"} <= hints.keys()
    assert Dual.blueprint() is not None
