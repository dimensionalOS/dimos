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

"""Wishlist item 2 — identify, list, and locate observed items.

Headless. Two parts:
  * the lidar localizer (Object.from_2d_to_list_lidar) on a synthetic scene,
    with a numeric localization-error check + a camera/top-down overlay;
  * the list_observed_items() catalog over an ObjectDB of known objects.

See manifest.yaml (02-identify-locate).
"""

from __future__ import annotations

import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.object import Object
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule

SKILL_ID = "02-identify-locate"

# Ground-truth blob centers (optical frame == world via identity transform).
_TRUTH = {"cup": np.array([-0.55, 0.25, 1.8]), "chair": np.array([0.62, 0.30, 2.6])}
# Per-object tolerance: the chair blob is broad + partly occluded by the wall.
_TOL = {"cup": 0.12, "chair": 0.30}


def _synthetic_scene() -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """cup + chair blobs plus a back wall and a floor, in optical convention."""
    rng = np.random.default_rng(7)
    cup = _TRUTH["cup"] + rng.normal(0, 0.035, (160, 3))
    chair = _TRUTH["chair"] + rng.normal(0, 0.10, (260, 3)) * [1.0, 1.6, 1.0]
    wall = np.column_stack(
        [rng.uniform(-2.2, 2.2, 900), rng.uniform(-1.2, 0.9, 900), rng.normal(3.6, 0.03, 900)]
    )
    floor = np.column_stack(
        [rng.uniform(-2.2, 2.2, 500), rng.normal(0.95, 0.02, 500), rng.uniform(0.8, 3.6, 500)]
    )
    scene = np.vstack([cup, chair, wall, floor])
    return scene, {"cup": cup, "chair": chair}


def test_lidar_localization_places_objects_at_world_centers(skill_output) -> None:
    """from_2d_to_list_lidar recovers each object's world center within tolerance."""
    import matplotlib.pyplot as plt

    scene, blobs = _synthetic_scene()
    world_pc = PointCloud2.from_numpy(scene.astype(float), frame_id="world", timestamp=1.0)
    camera_info = CameraInfo.from_intrinsics(
        fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480, frame_id="camera_optical"
    )
    img = Image.from_numpy(
        np.zeros((480, 640, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=1.0,
    )

    def project(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        z = pts[:, 2]
        return 500.0 * pts[:, 0] / z + 320.0, 500.0 * pts[:, 1] / z + 240.0, z

    def bbox_of(pts: np.ndarray, pad: float = 8) -> tuple[float, float, float, float]:
        u, v, _ = project(pts)
        return (u.min() - pad, v.min() - pad, u.max() + pad, v.max() + pad)

    dets = [
        Detection2DBBox(bbox=bbox_of(blobs["cup"]), track_id=1, class_id=0, confidence=0.91,
                        name="cup", ts=1.0, image=img),
        Detection2DBBox(bbox=bbox_of(blobs["chair"]), track_id=2, class_id=1, confidence=0.84,
                        name="chair", ts=1.0, image=img),
    ]
    objects = Object.from_2d_to_list_lidar(
        detections_2d=ImageDetections2D(img, dets),
        world_pointcloud=world_pc,
        camera_info=camera_info,
        world_to_optical_transform=Transform.identity(),
        filters=None,  # default stack: raycast + radius_outlier + statistical
    )

    assert objects, "localizer returned no objects"
    errs = {}
    for o in objects:
        assert o.name in _TRUTH
        err = float(np.linalg.norm(np.array([o.center.x, o.center.y, o.center.z]) - _TRUTH[o.name]))
        errs[o.name] = err
        assert err <= _TOL[o.name], f"{o.name} localized {err:.3f} m off (> {_TOL[o.name]} m)"
    assert set(errs) == set(_TRUTH), f"missed an object: got {set(errs)}"

    # --- review artifact --------------------------------------------------
    colors = {"cup": "#2a78d6", "chair": "#1baf7a"}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.6, 4.4))
    u, v, z = project(scene)
    inside = (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
    ax1.scatter(u[inside], v[inside], s=2.2, c=z[inside], cmap="Blues_r", alpha=0.75, linewidths=0)
    for det in dets:
        x1, y1, x2, y2 = det.bbox
        c = colors[det.name]
        ax1.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, edgecolor=c, linewidth=1.8))
        ax1.text(x1, y1 - 7, f"{det.name} {det.confidence:.2f}", color=c, fontsize=9,
                 fontweight="bold", family="monospace")
    ax1.set_xlim(0, 640)
    ax1.set_ylim(480, 0)
    ax1.set_title("camera view — YOLO bboxes + projected lidar")
    ax1.set_xlabel("u (px)")
    ax1.set_ylabel("v (px)")

    ax2.scatter(scene[:, 0], scene[:, 2], s=2.2, color="#b9c4d6", alpha=0.7, linewidths=0)
    for o in objects:
        c = colors[o.name]
        pts, _ = o.pointcloud.as_numpy()
        ax2.scatter(pts[:, 0], pts[:, 2], s=3.5, color=c, linewidths=0)
        ax2.scatter([o.center.x], [o.center.z], marker="x", s=90, color=c, linewidths=2.4)
        ax2.annotate(f"{o.name}\n±{errs[o.name]:.2f} m", (o.center.x, o.center.z),
                     xytext=(10, 8), textcoords="offset points", color=c, fontsize=8.5,
                     fontweight="bold", family="monospace")
    ax2.scatter([0], [0], marker="^", s=70, color="#131a24", zorder=5)
    ax2.set_xlim(-2.4, 2.4)
    ax2.set_ylim(-0.25, 4.0)
    ax2.set_title("world frame (top-down) — localized objects")
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("z, depth (m)")
    ax2.set_aspect("equal", adjustable="box")

    fig.tight_layout(pad=1.4)
    fig.savefig(skill_output.produced("lidar_localization.png"), dpi=115)
    plt.close(fig)


def test_list_observed_items_catalogs_the_scene(skill_output) -> None:
    """list_observed_items() reports every permanent object with pose/size/conf."""
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    try:
        # Distinct track_ids: the ObjectDB dedups same-track detections together,
        # so three real items need three tracks.
        specs = [
            ("cup", Vector3(1.0, 2.0, 0.8), 0.87, 12, 1),
            ("chair", Vector3(0.66, 2.76, 0.29), 0.66, 47, 2),
            ("luggage", Vector3(-2.99, 0.15, 0.05), 0.76, 29, 3),
        ]
        for name, center, conf, count, track_id in specs:
            obj = _make_catalog_object(name, center, conf, count, track_id)
            inserted = module._object_db.add_objects([obj])
            module._object_db.promote(inserted[0].object_id)

        catalog = module.list_observed_items()

        assert "Observing 3 item(s):" in catalog
        for name, center, _conf, _count, _tid in specs:
            assert name in catalog
            assert f"pos=({center.x:.2f}, {center.y:.2f}, {center.z:.2f})m" in catalog

        # An empty database answers cleanly too.
        empty = ObjectSceneRegistrationModule(localization="lidar")
        try:
            assert empty.list_observed_items() == "No objects observed yet."
        finally:
            empty.stop()

        skill_output.path("catalog.txt").write_text(
            "list_observed_items() over a 3-object ObjectDB:\n\n" + catalog + "\n"
        )
        skill_output.produced("catalog.txt")
    finally:
        module.stop()


def _make_catalog_object(
    name: str, center: Vector3, confidence: float, count: int, track_id: int
) -> Object:
    """A minimal valid world-frame Object for catalog tests (mirrors the unit-test helper)."""
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion

    image = Image.from_numpy(
        np.zeros((4, 4, 3), dtype=np.uint8), format=ImageFormat.BGR, frame_id="camera_optical", ts=1.0
    )
    pointcloud = PointCloud2.from_numpy(
        np.array([[center.x, center.y, center.z]], dtype=float), frame_id="world", timestamp=1.0
    )
    return Object(
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=track_id,
        class_id=0,
        confidence=confidence,
        name=name,
        ts=1.0,
        image=image,
        frame_id="world",
        center=center,
        size=Vector3(0.3, 0.3, 0.3),
        pose=PoseStamped(
            ts=1.0, frame_id="world", position=center, orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
        ),
        pointcloud=pointcloud,
        detections_count=count,
    )
