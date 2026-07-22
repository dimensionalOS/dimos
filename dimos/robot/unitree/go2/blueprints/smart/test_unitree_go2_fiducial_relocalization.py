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

"""The combined fiducial->judge stack: composition, no-op defaults, and the wired path.

MarkerDetectionStreamModule publishes marker sightings on its ``detections`` Out
stream; autoconnect wires that (by name+type) into RelocalizationModule's
``detections`` In, where the FiducialPrior Huber-fuses each tag's sightings into
ONE world->map candidate. The last test drives a mapped tag's sightings through
the reloc handler and asserts the prior ends up holding the fused
``map_T_world = map_T_marker @ inv(world_T_marker)`` (the boundary frame rule).
"""

from __future__ import annotations

import json
from pathlib import Path
import threading
from uuid import uuid4

import numpy as np

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.transport import LCMTransport
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.mapping.relocalization.priors import FiducialPriorConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.imageDetections3D import ImageDetections3D
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.marker_detection_stream_module import MarkerDetectionStreamModule
from dimos.perception.fiducial.test_helpers import blank_image
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import (
    unitree_go2_relocalization_lidar_fiducial,
)
from dimos.utils.threadpool import get_max_workers, get_scheduler

_MARKER_ID = 7


def _prewarm_global_scheduler() -> None:
    """Spawn every worker of the process-global ThreadPoolScheduler at import so
    conftest's per-test thread-leak check does not blame the first test that
    starts a module whose In streams deliver through that scheduler."""
    n = get_max_workers()
    barrier = threading.Barrier(n + 1)
    for _ in range(n):
        get_scheduler().executor.submit(barrier.wait)
    barrier.wait(timeout=10.0)


_prewarm_global_scheduler()


def _detections_array(marker_id: int, world_T_marker: np.ndarray, ts: float) -> Detection3DArray:
    """Wire Detection3DArray carrying one marker at ``world_T_marker`` -- exactly
    the fields the detector serializes (marker_id + bbox.center pose)."""
    tf = Transform.from_matrix(world_T_marker, frame_id="world", child_frame_id=f"marker_{marker_id}")
    image = blank_image(ts)
    marker = Detection3DMarker(
        bbox=(10.0, 20.0, 40.0, 50.0),
        track_id=-1,
        class_id=marker_id,
        confidence=1.0,
        name="",
        ts=ts,
        image=image,
        center=tf.translation,
        size=Vector3(0.1, 0.1, 0.0),
        transform=None,
        frame_id="world",
        orientation=tf.rotation,
        marker_id=marker_id,
        corners_px=np.zeros((4, 2), dtype=np.float32),
        dictionary="DICT_APRILTAG_36h11",
        reprojection_error=0.0,
    )
    return ImageDetections3D(image, [marker]).to_ros_detection3d_array(frame_id="world")


def _lcm(topic_ns: str, name: str, type_: type) -> LCMTransport:  # type: ignore[type-arg]
    return LCMTransport(f"/{topic_ns}/{name}", type_)


def test_unitree_go2_fiducial_relocalization_composes_full_path() -> None:
    bp = unitree_go2_relocalization_lidar_fiducial
    assert isinstance(bp, Blueprint)

    modules = [b.module for b in bp.blueprints]
    assert RelocalizationModule in modules
    assert MarkerDetectionStreamModule in modules

    reloc = next(b for b in bp.blueprints if b.module is RelocalizationModule)
    by_type = {type(p).__name__: p for p in reloc.kwargs["priors"]}
    # lidar + fiducial: both priors present and enabled.
    assert set(by_type) == {"RansacPriorConfig", "FiducialPriorConfig"}
    assert by_type["RansacPriorConfig"].enabled is True
    fid = by_type["FiducialPriorConfig"]
    assert fid.enabled is True
    assert fid.marker_length_m == 0.10
    assert fid.camera_info.frame_id == "camera_optical"
    assert fid.marker_map_file is None  # stays unset -> prior no-ops until surveyed

    # aruco_dictionary is wired from the fiducial prior config into the detector,
    # so detector and prior decode/fuse the same tag family from one source.
    detector = next(b for b in bp.blueprints if b.module is MarkerDetectionStreamModule)
    assert detector.kwargs["aruco_dictionary"] == fid.aruco_dictionary

    # ModuleCoordinator._connect_streams' grouping key: one shared (name, type)
    # entry means the detector Out and the reloc In land on the same transport --
    # the autoconnect wiring the whole blueprint exists for.
    members: dict[tuple[str, type], set[tuple[str, str]]] = {}
    for atom in bp.active_blueprints:
        for stream in atom.streams:
            name = bp.remapping_map.get((atom.name, stream.name), stream.name)
            if isinstance(name, str):
                members.setdefault((name, stream.type), set()).add((atom.name, stream.direction))
    assert members[("detections", Detection3DArray)] == {
        ("markerdetectionstreammodule", "out"),
        ("relocalizationmodule", "in"),
    }


def test_default_config_is_noop_until_maps_configured() -> None:
    """Blueprint defaults (no map_file, no marker_map_file) must start and idle;
    a marker frame arriving on the detections handler stays a no-op."""
    ns = f"test_fidreloc_{uuid4().hex[:8]}"
    reloc = RelocalizationModule(priors=[FiducialPriorConfig()])
    reloc.set_transport("global_map", _lcm(ns, "global_map", PointCloud2))
    reloc.set_transport("detections", _lcm(ns, "detections", Detection3DArray))
    try:
        reloc.start()
        assert reloc._premap is None  # disabled before any map decode
        assert reloc._fiducial_prior is None  # no marker_map_file -> prior off
        # A detection arriving with the prior off is a harmless no-op.
        reloc._on_detections(_detections_array(_MARKER_ID, np.eye(4), ts=10.0))
    finally:
        reloc.stop()


def test_detections_reach_fiducial_prior_as_fused_fix(tmp_path: Path) -> None:
    """Repeated sightings of a mapped tag -> the FiducialPrior holds the fused
    map_T_world = map_T_marker @ inv(world_T_marker) (marker map = identity here,
    so the stored fix is inv(world_T_marker))."""
    ns = f"test_fidreloc_{uuid4().hex[:8]}"
    premap_file = tmp_path / "premap.pc2.lcm"
    rng = np.random.default_rng(7)  # deterministic; pinned for reruns
    premap_file.write_bytes(
        PointCloud2.from_numpy(rng.uniform(-1, 1, (64, 3)), timestamp=10.0).lcm_encode()
    )
    marker_map_file = tmp_path / "markers.json"
    marker_map_file.write_text(
        json.dumps(
            {"markers": {str(_MARKER_ID): {"translation": [0.0, 0.0, 0.0], "rotation": [0, 0, 0, 1]}}}
        )
    )

    reloc = RelocalizationModule(
        priors=[FiducialPriorConfig(marker_map_file=str(marker_map_file))],
        map_file=str(premap_file),
    )
    reloc.set_transport("global_map", _lcm(ns, "global_map", PointCloud2))
    reloc.set_transport("detections", _lcm(ns, "detections", Detection3DArray))
    try:
        reloc.start()
        assert reloc._premap is not None  # map_file override flips it out of no-op
        assert reloc._fiducial_prior is not None

        world_T_marker = np.eye(4)
        world_T_marker[:3, :3] = _rot_z_35()
        world_T_marker[:3, 3] = (1.2, -0.7, 0.3)
        for i in range(3):  # >= min_observations -> a fused fix is stamped
            reloc._on_detections(_detections_array(_MARKER_ID, world_T_marker, ts=10.0 + 0.1 * i))

        assert _MARKER_ID in reloc._fiducial_prior._fixes
        fix_T, _ts = reloc._fiducial_prior._fixes[_MARKER_ID]
        np.testing.assert_allclose(fix_T, np.linalg.inv(world_T_marker), atol=1e-6)
    finally:
        reloc.stop()


def _rot_z_35() -> np.ndarray:
    c, s = np.cos(np.radians(35.0)), np.sin(np.radians(35.0))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
