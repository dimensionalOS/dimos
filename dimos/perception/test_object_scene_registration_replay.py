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

"""Full-pipeline replay validation for skill 2 (identify, list, locate).

Drives the production ``ObjectSceneRegistrationModule`` in lidar-localization
mode over an entire recorded Go2 session (camera + mid360 lidar + odom), the
same code path the ``unitree-go2-scene`` blueprint runs live:

    YOLO-E detect -> project world-frame lidar into each detection ->
    world-frame Object -> ObjectDB dedup/promote -> list_observed_items catalog

Marked self_hosted: downloads the dataset and YOLO-E weights, and wants a GPU.

Run with a visible catalog:

    uv run pytest dimos/perception/test_object_scene_registration_replay.py -s
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.protocol.tf.tf import TF
from dimos.robot.unitree.go2 import connection
from dimos.robot.unitree.type.odometry import Odometry
from dimos.utils.data import get_data
from dimos.utils.testing.legacy_pickle import LegacyPickleStore

pytestmark = pytest.mark.self_hosted

DATA_DIR = "unitree_go2_lidar_corrected"
# Process every Nth lidar frame (~7.3 Hz recording -> ~2.4 Hz replay).
LIDAR_STRIDE = 3
# Max camera/lidar timestamp gap, mirrors the module's align_timestamped tolerance.
MATCH_TOLERANCE_S = 0.25


class ReplayStats:
    frames_processed: int = 0
    frames_skipped: int = 0
    detections_2d_total: int = 0
    published_object_batches: int = 0


def _load_odom_restamped() -> list[tuple[float, Odometry]]:
    """Load odometry restamped to recording (arrival) time.

    The Go2's odometry messages are stamped with the robot's internal clock,
    ~1 minute off the lidar/camera stamps in this recording. The store's own
    per-item timestamps are arrival time on the recording host — the same
    clock the lidar/camera stamps use — so restamp each pose with those to
    make tf lookups at sensor time work.
    """
    odom_store = LegacyPickleStore(f"{DATA_DIR}/odom", autocast=Odometry.from_msg)
    items: list[tuple[float, Odometry]] = []
    for file_ts, odom in odom_store.iterate_items():
        odom.ts = file_ts
        items.append((file_ts, odom))
    return items


def _find_odom(odom_items: list[tuple[float, Odometry]], ts: float) -> Odometry | None:
    """Nearest odom pose by restamped time (odom_items is time-sorted)."""
    import bisect

    idx = bisect.bisect_left(odom_items, ts, key=lambda item: item[0])
    best = None
    for j in (idx - 1, idx):
        if 0 <= j < len(odom_items):
            if best is None or abs(odom_items[j][0] - ts) < abs(best[0] - ts):
                best = odom_items[j]
    return best[1] if best else None


def _replay_session(module: ObjectSceneRegistrationModule) -> ReplayStats:
    """Feed the whole recorded session through the module's lidar path."""
    get_data(DATA_DIR)
    lidar_store = LegacyPickleStore(f"{DATA_DIR}/lidar")
    video_store = LegacyPickleStore(f"{DATA_DIR}/video")
    odom_items = _load_odom_restamped()

    tf = TF()
    module._tf = tf  # local tf fed from recorded odom, no live transport
    module._camera_info = connection._camera_info_static()

    stats = ReplayStats()
    module.detections_2d.subscribe(
        lambda msg: setattr(
            stats, "detections_2d_total", stats.detections_2d_total + msg.detections_length
        )
    )
    module.objects.subscribe(
        lambda objs: setattr(stats, "published_object_batches", stats.published_object_batches + 1)
    )

    try:
        for i, (_ts, lidar_frame) in enumerate(lidar_store.iterate_items()):
            if i % LIDAR_STRIDE:
                continue

            image_frame = video_store.find_closest(lidar_frame.ts)
            if image_frame is None or abs(image_frame.ts - lidar_frame.ts) > MATCH_TOLERANCE_S:
                stats.frames_skipped += 1
                continue
            image_frame.frame_id = "camera_optical"

            odom_frame = _find_odom(odom_items, lidar_frame.ts)
            if odom_frame is None:
                stats.frames_skipped += 1
                continue
            tf.receive_transform(*connection.GO2Connection._odom_to_tf(odom_frame))

            module._process_lidar(image_frame, lidar_frame)
            stats.frames_processed += 1
    finally:
        tf.stop()

    return stats


@pytest.fixture(scope="module")
def replayed_module():
    from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode

    # Mirror the unitree-go2-scene blueprint's tuning (see that blueprint for
    # the distance_threshold rationale).
    module = ObjectSceneRegistrationModule(
        target_frame="world",
        localization="lidar",
        camera_optical_frame="camera_optical",
        distance_threshold=0.35,
    )
    # Same detector module.start() creates for LRPC mode, minus the LCM wiring.
    module._detector = Yoloe2DDetector(
        model_name="yoloe-11l-seg-pf.pt",
        prompt_mode=YoloePromptMode.LRPC,
    )

    stats = _replay_session(module)

    yield module, stats

    if module._detector is not None:
        module._detector.stop()
        module._detector = None
    module.stop()


def test_replay_processes_most_of_the_session(replayed_module) -> None:
    _, stats = replayed_module
    assert stats.frames_processed >= 100, f"too few frames processed: {vars(stats)}"
    # Camera/lidar alignment should succeed for the vast majority of frames.
    assert stats.frames_skipped <= stats.frames_processed * 0.2, vars(stats)
    assert stats.detections_2d_total > 0


def test_replay_accumulates_permanent_objects(replayed_module) -> None:
    module, stats = replayed_module
    permanent = module._object_db.get_objects()
    assert len(permanent) >= 3, (
        f"expected a populated scene, got {len(permanent)} permanent objects "
        f"(stats={vars(stats)}, db={module._object_db!r})"
    )
    # Dedup must be doing real work: far fewer objects than raw detections.
    total = len(module._object_db.get_all_objects())
    assert total < stats.detections_2d_total * 0.5, (
        f"{total} objects from {stats.detections_2d_total} detections — dedup not effective"
    )
    # Stability: each permanent object was corroborated across frames.
    for obj in permanent:
        assert obj.detections_count >= module._object_db._min_detections


def test_replay_objects_are_localized_in_world(replayed_module) -> None:
    module, _ = replayed_module
    # Bounds of everything the lidar saw over the whole session.
    lidar_store = LegacyPickleStore(f"{DATA_DIR}/lidar")
    lo = np.array([np.inf] * 3)
    hi = -np.array([np.inf] * 3)
    for _ts, frame in lidar_store.iterate_items():
        pts, _ = frame.as_numpy()
        if len(pts):
            lo = np.minimum(lo, pts.min(axis=0))
            hi = np.maximum(hi, pts.max(axis=0))

    for obj in module._object_db.get_objects():
        assert obj.frame_id == "world"
        c = np.array([obj.center.x, obj.center.y, obj.center.z])
        assert (c >= lo - 1.0).all() and (c <= hi + 1.0).all(), (
            f"{obj.name} localized at {c} outside session bounds [{lo}, {hi}]"
        )
        assert obj.size.length() > 0.0
        assert obj.confidence > 0.0
        assert len(obj.pointcloud.pointcloud.points) >= 3


def test_replay_catalog_lists_observed_items(replayed_module) -> None:
    module, _ = replayed_module
    catalog = module.list_observed_items()

    permanent = module._object_db.get_objects()
    assert f"Observing {len(permanent)} item(s):" in catalog
    for obj in permanent:
        assert obj.object_id in catalog
    assert "pos=(" in catalog
    assert "size=(" in catalog

    print("\n" + catalog)


def test_replay_catalog_with_vlm_descriptions(replayed_module) -> None:
    """Refine every catalogued object with the local VLM (item 5's path) and
    print the final catalog: names, descriptions, and world locations."""
    module, _ = replayed_module

    # Detection is done; free the detector's GPU memory so the VLM fits
    # (YOLO-E + moondream together exceed an 8GB card). stop() keeps the
    # model referenced, so drop it explicitly before collecting.
    if module._detector is not None:
        module._detector.stop()
        module._detector.model = None
        module._detector = None
    import gc

    import torch

    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    permanent = module._object_db.get_objects()

    refined = module.refine_object_labels(object_ids=[o.object_id for o in permanent])
    assert refined, "VLM refinement produced no labels"

    described = [o for o in module._object_db.get_objects() if o.refined_description]
    assert described, "no object ended up with a description"

    catalog = module.list_observed_items()
    for obj in described:
        assert obj.refined_description in catalog

    print("\n" + catalog)


def test_replay_module_exposes_catalog_scene_skill(replayed_module) -> None:
    """catalog_scene is the bundled find-and-catalog entry point (refine +
    list in one call); confirm it's a registered agent-callable skill on the
    production module. Functional correctness (refine-then-list) is covered
    by the fast stub-VLM test in test_contextual_labeling.py, so this only
    checks wiring — no need to re-run the real VLM here."""
    module, _ = replayed_module
    names = [s.func_name for s in module.get_skills()]
    assert "catalog_scene" in names
