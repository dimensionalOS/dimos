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

from __future__ import annotations

import functools
import itertools
import json
import math
import struct
from typing import TYPE_CHECKING, Any

# Import LCM types
from dimos_lcm.sensor_msgs.PointCloud2 import (
    PointCloud2 as LCMPointCloud2,
)
from dimos_lcm.sensor_msgs.PointField import PointField
from dimos_lcm.std_msgs.Header import Header
import numpy as np
from scipy import ndimage

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    import open3d as o3d  # type: ignore[import-untyped]
    from rerun._baseclasses import Archetype

    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image


@functools.lru_cache(maxsize=16)
def _get_matplotlib_cmap(name: str):  # type: ignore[no-untyped-def]
    """Get a matplotlib colormap by name (cached for performance)."""
    import matplotlib.pyplot as plt

    return plt.get_cmap(name)


@functools.lru_cache(maxsize=16)
def _get_colormap_lut(name: str) -> np.ndarray:
    """Build a 256-entry uint8 LUT from a matplotlib colormap (one-time cost)."""
    cmap = _get_matplotlib_cmap(name)
    t = np.linspace(0, 1, 256)
    return (cmap(t)[:, :3] * 255).astype(np.uint8)  # type: ignore[no-any-return]


def register_colormap_annotation(name: str = "turbo") -> None:
    """Register a colormap as AnnotationContext so Rerun resolves colors viewer-side."""
    import rerun as rr

    lut = _get_colormap_lut(name)
    rr.log(
        "/",
        rr.AnnotationContext(
            [
                rr.datatypes.ClassDescription(
                    info=rr.datatypes.AnnotationInfo(id=i, color=lut[i].tolist())
                )
                for i in range(256)
            ]
        ),
        static=True,
    )


# TODO: encode/decode need to be updated to work with full spectrum of pointcloud2 fields
class PointCloud2(Timestamped):
    msg_name = "sensor_msgs.PointCloud2"

    def __init__(
        self,
        pointcloud: o3d.geometry.PointCloud | o3d.t.geometry.PointCloud | None = None,
        frame_id: str = "world",
        ts: float | None = None,
    ) -> None:
        import open3d as o3d  # type: ignore[import-untyped]

        self.ts = ts  # type: ignore[assignment]
        self.frame_id = frame_id

        # Store internally as tensor pointcloud for speed
        if pointcloud is None:
            self._pcd_tensor: o3d.t.geometry.PointCloud = o3d.t.geometry.PointCloud()
        elif isinstance(pointcloud, o3d.t.geometry.PointCloud):
            self._pcd_tensor = pointcloud
        elif len(pointcloud.points) == 0:
            # from_legacy() warns on empty legacy clouds; build an empty tensor instead
            self._pcd_tensor = o3d.t.geometry.PointCloud()
        else:
            self._pcd_tensor = o3d.t.geometry.PointCloud.from_legacy(pointcloud)
        self._pcd_legacy_cache: o3d.geometry.PointCloud | None = None

    def _ensure_tensor_initialized(self) -> None:
        """Ensure _pcd_tensor and _pcd_legacy_cache exist (handles unpickled old objects)."""
        import open3d as o3d  # type: ignore[import-untyped]

        # Always ensure _pcd_legacy_cache exists
        if not hasattr(self, "_pcd_legacy_cache"):
            self._pcd_legacy_cache = None

        # Check for old pickled format: 'pointcloud' directly in __dict__
        # This takes priority even if _pcd_tensor exists (it might be empty)
        old_pcd = self.__dict__.get("pointcloud")
        if old_pcd is not None and isinstance(old_pcd, o3d.geometry.PointCloud):
            self._pcd_tensor = o3d.t.geometry.PointCloud.from_legacy(old_pcd)
            self._pcd_legacy_cache = old_pcd  # reuse it
            del self.__dict__["pointcloud"]
            return

        if not hasattr(self, "_pcd_tensor"):
            self._pcd_tensor = o3d.t.geometry.PointCloud()

    def __getstate__(self) -> dict[str, object]:
        """Serialize to numpy for pickling (tensors don't pickle well)."""
        self._ensure_tensor_initialized()
        state = self.__dict__.copy()
        # Convert tensor to numpy for serialization
        if "positions" in self._pcd_tensor.point:
            state["_pcd_numpy"] = self._pcd_tensor.point["positions"].numpy()
        else:
            state["_pcd_numpy"] = np.zeros((0, 3), dtype=np.float32)
        # Remove non-picklable objects
        del state["_pcd_tensor"]
        state["_pcd_legacy_cache"] = None
        # Remove all cached_property entries
        for key in list(state):
            if isinstance(getattr(type(self), key, None), functools.cached_property):
                del state[key]
        return state

    def __setstate__(self, state: dict[str, object]) -> None:
        """Restore from pickled state."""
        import open3d as o3d  # type: ignore[import-untyped]
        import open3d.core as o3c  # type: ignore[import-untyped]

        points_obj = state.pop("_pcd_numpy", None)
        points: np.ndarray[tuple[int, int], np.dtype[np.float32]] = (
            points_obj if isinstance(points_obj, np.ndarray) else np.zeros((0, 3), dtype=np.float32)
        )
        self.__dict__.update(state)
        # Recreate tensor from numpy
        self._pcd_tensor = o3d.t.geometry.PointCloud()
        if len(points) > 0:
            self._pcd_tensor.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)

    @property
    def pointcloud(self) -> o3d.geometry.PointCloud:
        """Legacy pointcloud property for backwards compatibility. Cached."""
        self._ensure_tensor_initialized()
        if self._pcd_legacy_cache is None:
            self._pcd_legacy_cache = self._pcd_tensor.to_legacy()
        return self._pcd_legacy_cache

    @pointcloud.setter
    def pointcloud(self, value: o3d.geometry.PointCloud | o3d.t.geometry.PointCloud) -> None:
        import open3d as o3d  # type: ignore[import-untyped]

        if isinstance(value, o3d.t.geometry.PointCloud):
            self._pcd_tensor = value
        elif len(value.points) == 0:
            self._pcd_tensor = o3d.t.geometry.PointCloud()
        else:
            self._pcd_tensor = o3d.t.geometry.PointCloud.from_legacy(value)
        self._pcd_legacy_cache = None

    @property
    def pointcloud_tensor(self) -> o3d.t.geometry.PointCloud:
        """Direct access to tensor pointcloud (faster, no conversion)."""
        self._ensure_tensor_initialized()
        return self._pcd_tensor

    @classmethod
    def from_numpy(
        cls,
        points: np.ndarray,
        frame_id: str = "world",
        timestamp: float | None = None,
        intensities: np.ndarray | None = None,
    ) -> PointCloud2:
        """Create PointCloud2 from numpy array of shape (N, 3).

        Args:
            points: Nx3 numpy array of 3D points
            frame_id: Frame ID for the point cloud
            timestamp: Timestamp for the point cloud (defaults to current time)
            intensities: Optional Nx1 or (N,) float array of per-point intensity values

        Returns:
            PointCloud2 instance
        """
        import open3d as o3d  # type: ignore[import-untyped]
        import open3d.core as o3c  # type: ignore[import-untyped]

        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32)
        if intensities is not None:
            arr = intensities.astype(np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(-1, 1)
            pcd_t.point["intensities"] = o3c.Tensor(arr, dtype=o3c.float32)
        return cls(pointcloud=pcd_t, ts=timestamp, frame_id=frame_id)

    @classmethod
    def from_rgbd(
        cls,
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
        depth_scale: float = 1.0,
        depth_trunc: float = 5.0,
    ) -> PointCloud2:
        """Create PointCloud2 from RGB and depth Image messages.

        Uses frame_id and timestamp from the depth image.

        Args:
            color_image: RGB/BGR color Image message
            depth_image: Depth Image message (float32 meters or uint16 mm)
            camera_info: CameraInfo message with intrinsics
            depth_scale: Scale factor to convert depth to meters (default 1.0 for float32)
            depth_trunc: Maximum depth in meters to include

        Returns:
            PointCloud2 instance with colored points
        """
        import open3d as o3d  # type: ignore[import-untyped]

        # Get color as RGB numpy array
        color_data = color_image.to_rgb().data
        if hasattr(color_data, "get"):  # CuPy array
            color_data = color_data.get()
        color_data = np.ascontiguousarray(color_data)

        # Get depth numpy array
        depth_data = depth_image.data
        if hasattr(depth_data, "get"):  # CuPy array
            depth_data = depth_data.get()

        # Convert depth to float32 meters if needed
        if depth_data.dtype == np.uint16:
            depth_data = depth_data.astype(np.float32) * depth_scale
        elif depth_data.dtype != np.float32:
            depth_data = depth_data.astype(np.float32)
        depth_data = np.ascontiguousarray(depth_data)

        # Verify dimensions match
        color_h, color_w = color_data.shape[:2]
        depth_h, depth_w = depth_data.shape[:2]
        if (color_h, color_w) != (depth_h, depth_w):
            raise ValueError(
                f"Color {color_w}x{color_h} and depth {depth_w}x{depth_h} dimensions don't match"
            )

        # Get intrinsics from camera_info
        intrinsic = camera_info.get_K_matrix()
        fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        cx, cy = intrinsic[0, 2], intrinsic[1, 2]

        # Verify intrinsics match image dimensions
        if camera_info.width != color_w or camera_info.height != color_h:
            # Scale intrinsics if resolution differs
            scale_x = color_w / camera_info.width
            scale_y = color_h / camera_info.height
            fx *= scale_x
            fy *= scale_y
            cx *= scale_x
            cy *= scale_y

        # Create Open3D images
        color_o3d = o3d.geometry.Image(color_data.astype(np.uint8))

        # Filter invalid depth values
        depth_filtered = depth_data.copy()
        valid_mask = np.isfinite(depth_filtered) & (depth_filtered > 0)
        depth_filtered[~valid_mask] = 0.0
        depth_o3d = o3d.geometry.Image(depth_filtered.astype(np.float32))

        o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=color_w,
            height=color_h,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )

        # Create RGBD image and point cloud
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d,
            depth_o3d,
            depth_scale=1.0,  # Already scaled
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False,
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic)

        return cls(
            pointcloud=pcd,
            frame_id=depth_image.frame_id,
            ts=depth_image.ts,
        )

    def __str__(self) -> str:
        return f"PointCloud2(frame_id='{self.frame_id}', num_points={len(self)})"

    def agent_encode(self) -> dict[str, object]:
        """Compact, spatially structured encoding for LLM consumption.

        World-frame meters throughout. Carries the cloud's horizontal centroid
        (so consecutive frames reveal motion of the mapped region) and exact
        world-meter bounding boxes of body-height obstacle clusters (so
        clearance around a given world position is closed-form point-to-box).
        """
        pts = self.points_f32()
        n = int(pts.shape[0])
        out: dict[str, object] = {"frame_id": self.frame_id, "num_points": n}
        if n == 0:
            return out
        xy = pts[:, :2]
        cx, cy = xy.mean(axis=0)
        out["centroid_xy_m"] = [round(float(cx), 2), round(float(cy), 2)]
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        # Distinct occupied 0.2 m x-y cells of THIS frame's cloud alone.
        floor_cells = np.unique(np.floor(xy / 0.2).astype(np.int64), axis=0)
        out["exact_stats"] = {
            "note": "exact full-cloud values in meters; for numeric extent/span/area "
            "answers use these, not the body-height interval map below",
            "x_range": [round(float(mins[0]), 2), round(float(maxs[0]), 2)],
            "y_range": [round(float(mins[1]), 2), round(float(maxs[1]), 2)],
            "z_range": [round(float(mins[2]), 2), round(float(maxs[2]), 2)],
            "horizontal_extent_m": round(float(max(maxs[0] - mins[0], maxs[1] - mins[1])), 2),
            "vertical_span_m": round(float(maxs[2] - mins[2]), 2),
            "occupied_floor_footprint_m2": round(float(floor_cells.shape[0]) * 0.2 * 0.2, 1),
            "footprint_note": "mapped floor area of this frame's cloud only (distinct "
            "occupied 0.2 m x-y cells); use it, not bbox area, for floor coverage. For "
            "area trends compare each frame's own footprint value across frames -- it "
            "can decrease as well as increase; do not accumulate coverage over frames",
        }
        out["compass"] = (
            "8-way direction of motion (dx,dy = last minus first): if |dx|>2.41*|dy| "
            "then east (dx>0) or west (dx<0); if |dy|>2.41*|dx| then north (dy>0) or "
            "south (dy<0); otherwise diagonal by signs (northeast, northwest, "
            "southeast, southwest). For any question about which direction the map "
            "moved or gained coverage, take dx,dy from centroid_xy_m of the last "
            "minus the first frame; range edges are too noisy for direction"
        )
        z = pts[:, 2]
        body = (z >= 0.15) & (z <= 1.0)
        topo = self._enclosure_topology(xy, body)
        if topo is not None:
            out["enclosure_topology"] = topo
        grid = self._body_height_occupancy(xy[body])
        if grid is not None:
            out["body_height_occupancy"] = grid
        relief = self._ground_relief(pts)
        if relief is not None:
            out["ground_relief"] = relief
        gaps = self._doorway_gaps(pts)
        if gaps is not None:
            out["wall_openings"] = gaps
        # The rectangle list is the one channel here that can be shortened
        # without losing a distinction, because its patches are emitted
        # largest first: hand it whatever of the frame's byte budget the
        # other channels left, so a dense frame trims its own tail instead
        # of pushing the whole encoding over the cap.
        pockets = self._unmeasured_pockets(xy, self.ENCODE_SOFT_CAP - len(json.dumps(out)))
        if pockets is not None:
            out["unmeasured_pockets"] = pockets
        return out

    ENCODE_SOFT_CAP = 5940
    """Bytes one frame's encoding aims to stay under. The pocket rectangles
    are trimmed to fit it; every other channel is emitted in full."""

    POCKET_CELL = 0.25
    """Side of the x-y cell a patch of missing measurement is resolved on."""

    POCKET_MIN_PTS = 3
    """Returns in a cell for it to count as measured at all."""

    POCKET_MIN_M2 = 1.0
    """Smallest patch worth naming -- under this it is sensor speckle."""

    POCKET_MAX_ROWS = 26
    """Bands the frame is cut into before the cell is doubled; keeps a fused
    map from being resolved at the same 0.25 m as a single sweep."""

    POCKET_MAX_BYTES = 340
    """Ceiling on the emitted rectangle string. Exact extents cost several
    rectangles where a bounding box costs one, and the encoding as a whole has
    a hard byte cap; patches are emitted largest first and the list is cut
    where it reaches this many bytes. Truncation only shortens the list -- the
    rectangles that are emitted still cover unmeasured cells only."""

    POCKET_MIN_BYTES = 24
    """Below this the list cannot hold one useful rectangle, so the channel is
    dropped rather than emitted with a description of nothing."""

    POCKET_DESC = (
        f"Patches >={POCKET_MIN_M2:g} m2 in the mapped range with no "
        "returns: never measured, so unknown floor, not known-clear. Exact extents, "
        "body_height_occupancy's format (xmin:xmax@ymin:ymax, world m), covering "
        "only unmeasured cells. Beyond exact_stats.x_range/y_range is unmeasured "
        "too. Route answers only, at the goal point: a goal inside a rectangle or "
        "beyond those ranges is unknown, not reachable; one outside them all is on "
        "measured floor -- not unknown merely for passing near a rectangle. Not "
        "obstacles, walls, doorways or rooms."
    )

    @staticmethod
    def _pocket_rects(comp: np.ndarray, cell: float) -> list[str]:
        """One patch as exact rectangles: maximal x-runs per y band, merged
        down the y axis wherever consecutive bands share the same runs.

        The union of the rectangles is exactly the patch's cells -- unlike a
        bounding box, which claims the measured floor around a shadow wedge
        as well as the wedge.
        """
        bands: dict[int, list[tuple[int, int]]] = {}
        for j in np.unique(comp[:, 0]):
            idx = np.sort(comp[comp[:, 0] == j][:, 1])
            breaks = np.flatnonzero(np.diff(idx) > 1)
            starts = np.concatenate(([0], breaks + 1))
            ends = np.concatenate((breaks, [idx.size - 1]))
            bands[int(j)] = [(int(idx[s]), int(idx[e])) for s, e in zip(starts, ends, strict=False)]
        groups: list[tuple[list[int], list[tuple[int, int]]]] = []
        for j in sorted(bands, reverse=True):
            if groups and groups[-1][1] == bands[j] and groups[-1][0][-1] - 1 == j:
                groups[-1][0].append(j)
            else:
                groups.append(([j], bands[j]))

        def m(v: float) -> str:
            return f"{v:.2f}".rstrip("0").rstrip(".")

        out: list[str] = []
        for js, runs in groups:
            y0, y1 = m(min(js) * cell), m((max(js) + 1) * cell)
            for a, b in runs:
                out.append(f"{m(a * cell)}:{m((b + 1) * cell)}@{y0}:{y1}")
        return out

    @classmethod
    def _unmeasured_pockets(cls, xy: np.ndarray, budget: int) -> dict[str, object] | None:
        """Patches inside the mapped area that the sensor never measured.

        The obstacle boxes say where returns *are*; nothing in them separates
        floor the sensor swept and found empty from space it never reached --
        past its range, or in the shadow of something. This is that second
        thing, and it is emitted as the complement: the patches with no
        returns, at the same exact extent the obstacle boxes use, rather than
        as a listing of where the sensor did reach.
        """
        if xy.shape[0] == 0:
            return None
        fixed = len(json.dumps({"unmeasured_pockets": {"desc": cls.POCKET_DESC, "patches": ""}}))
        allowance = min(cls.POCKET_MAX_BYTES, budget - fixed)
        if allowance < cls.POCKET_MIN_BYTES:
            return None
        cell = cls.POCKET_CELL
        lo = xy.min(axis=0)
        hi = xy.max(axis=0)
        span = float(hi[1] - lo[1])
        if not np.isfinite(span) or not np.isfinite(lo).all():
            return None
        while span / cell > cls.POCKET_MAX_ROWS:
            cell *= 2.0
        # Snap to a world 0.25 m lattice so every emitted edge is a round
        # multiple of the cell -- shorter to print and stable across frames.
        i0 = int(np.floor(lo[0] / cell))
        j0 = int(np.floor(lo[1] / cell))
        ij = np.floor(xy / cell).astype(np.int64)
        ij[:, 0] -= i0
        ij[:, 1] -= j0
        w = int(ij[:, 0].max()) + 1
        h = int(ij[:, 1].max()) + 1
        counts = np.zeros(h * w, dtype=np.int32)
        np.add.at(counts, ij[:, 1] * w + ij[:, 0], 1)
        blank = (counts < cls.POCKET_MIN_PTS).reshape(h, w)
        seen = np.zeros((h, w), dtype=bool)
        min_cells = max(1, round(cls.POCKET_MIN_M2 / (cell * cell)))
        found: list[np.ndarray] = []
        for sj in range(h):
            for si in range(w):
                if not blank[sj, si] or seen[sj, si]:
                    continue
                seen[sj, si] = True
                stack = [(sj, si)]
                members: list[tuple[int, int]] = []
                while stack:
                    j, i = stack.pop()
                    members.append((j, i))
                    for nj, ni in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)):
                        if 0 <= nj < h and 0 <= ni < w and blank[nj, ni] and not seen[nj, ni]:
                            seen[nj, ni] = True
                            stack.append((nj, ni))
                if len(members) >= min_cells:
                    found.append(np.array(members))
        if not found:
            return None
        found.sort(key=len, reverse=True)
        parts: list[str] = []
        used = 0
        for comp in found:
            for rect in cls._pocket_rects(comp + np.array([j0, i0]), cell):
                used += len(rect) + 1
                if used > allowance:
                    break
                parts.append(rect)
            if used > allowance:
                break
        if not parts:
            return None
        return {"desc": cls.POCKET_DESC, "patches": ",".join(parts)}

    DOOR_SCALES = (0.10, 0.125, 0.15, 0.20)
    """Cell sizes the gap scan is repeated at. One quantization can straddle a
    door frame and miss it, or split one wall into two; agreement across
    several is what separates a door from a lucky pair of chair legs."""

    DOOR_WIDTH = (0.7, 1.0)
    """Free span of a passable opening, meters -- an interior door leaf."""

    DOOR_FLANK = 0.4
    """How far the structure either side must continue for the gap to be a
    break in something, rather than the end of it."""

    DOOR_THROUGH = 0.6
    """How far mapped floor must run clear straight through the gap, both
    faces, for it to be a passage rather than a slot between two objects."""

    DOOR_MERGE = 0.6
    """Two hits this close are the same opening seen at two cell sizes."""

    @classmethod
    def _gap_scan(cls, band: np.ndarray, xy: np.ndarray, cell: float) -> list[tuple[float, float]]:
        """Door-width breaks in body-height structure at one cell size.

        Two rasters: ``occ`` is where this frame has body-height returns,
        ``mapped`` is where it has any return at all. A doorway is a short run
        of unoccupied-but-mapped cells along a line, walled at both ends, with
        clear mapped floor running through it perpendicular to that line.
        """
        lo = xy.min(axis=0)
        ij = np.floor((xy - lo) / cell).astype(np.int64)
        w = int(ij[:, 0].max()) + 1
        h = int(ij[:, 1].max()) + 1
        if w * h > 400_000:  # a fused map at 10 cm; not worth the scan
            return []
        tot = np.zeros((h, w), np.int32)
        np.add.at(tot, (ij[:, 1], ij[:, 0]), 1)
        bj = np.floor((band - lo) / cell).astype(np.int64)
        np.clip(bj[:, 0], 0, w - 1, out=bj[:, 0])
        np.clip(bj[:, 1], 0, h - 1, out=bj[:, 1])
        hit = np.zeros((h, w), np.int32)
        np.add.at(hit, (bj[:, 1], bj[:, 0]), 1)
        occ = hit >= 3
        mapped = tot >= 3
        lo_w = int(np.ceil(cls.DOOR_WIDTH[0] / cell))
        hi_w = int(np.floor(cls.DOOR_WIDTH[1] / cell))
        flank = int(np.ceil(cls.DOOR_FLANK / cell))
        thru = int(np.ceil(cls.DOOR_THROUGH / cell))
        out: list[tuple[float, float]] = []
        for axis in (0, 1):
            grid_o = occ if axis == 0 else occ.T
            grid_m = mapped if axis == 0 else mapped.T
            rows, cols = grid_o.shape
            found: list[tuple[int, float]] = []
            for j in range(rows):
                line = grid_o[j]
                idx = np.flatnonzero(line)
                if idx.size < 2:
                    continue
                for a, b in itertools.pairwise(idx):
                    span = b - a - 1
                    if span < lo_w or span > hi_w:
                        continue
                    left = line[a - flank + 1 : a + 1]
                    right = line[b : b + flank]
                    if left.size < flank or right.size < flank:
                        continue  # the wall runs off the mapped edge; nothing to break
                    if a - flank + 1 < 0 or not left.all() or not right.all():
                        continue
                    if grid_m[j, a + 1 : b].mean() < 0.5:
                        continue
                    c = (a + b) // 2
                    ends = (
                        grid_o[max(0, j - thru) : j, c],
                        grid_o[j + 1 : j + 1 + thru, c],
                        grid_m[max(0, j - thru) : j, c],
                        grid_m[j + 1 : j + 1 + thru, c],
                    )
                    if any(e.size < thru for e in ends):
                        continue
                    if ends[0].any() or ends[1].any() or not ends[2].all() or not ends[3].all():
                        continue
                    found.append((j, (a + b) / 2.0))
            # One opening shows up on the two or three lines the wall is thick.
            runs: list[list[tuple[int, float]]] = []
            for f in found:
                for run in runs:
                    if abs(run[-1][0] - f[0]) <= 1 and abs(run[-1][1] - f[1]) * cell <= 0.3:
                        run.append(f)
                        break
                else:
                    runs.append([f])
            for run in runs:
                if len(run) < 2:  # a single line is a coincidence, not a frame
                    continue
                along = sum(r[1] for r in run) / len(run)
                across = sum(r[0] for r in run) / len(run)
                p = (
                    (lo[0] + (along + 0.5) * cell, lo[1] + (across + 0.5) * cell)
                    if axis == 0
                    else (lo[0] + (across + 0.5) * cell, lo[1] + (along + 0.5) * cell)
                )
                out.append((float(p[0]), float(p[1])))
        return out

    @classmethod
    def _doorway_gaps(cls, pts: np.ndarray) -> dict[str, object] | None:
        """Passable door-width openings, agreed across cell sizes.

        The obstacle boxes carry every gap in the body-height returns, and most
        of them are furniture edges or the far side of the sensor's reach. This
        picks out the ones shaped like a door: narrow, walled both sides, and
        with floor running through.
        """
        if pts.shape[0] == 0:
            return None
        xy = pts[:, :2]
        z = pts[:, 2]
        band = xy[(z >= 0.15) & (z <= 1.0)]
        if band.shape[0] < 50:
            return None
        hits: list[tuple[float, float]] = []
        for cell in cls.DOOR_SCALES:
            hits += cls._gap_scan(band, xy, cell)
        clusters: list[list[tuple[float, float]]] = []
        for hx, hy in hits:
            for group in clusters:
                gx = sum(p[0] for p in group) / len(group)
                gy = sum(p[1] for p in group) / len(group)
                if math.hypot(gx - hx, gy - hy) <= cls.DOOR_MERGE:
                    group.append((hx, hy))
                    break
            else:
                clusters.append([(hx, hy)])
        clusters.sort(key=len, reverse=True)
        # One opening when one is clearly best. Ties are named together rather
        # than broken arbitrarily: two cell sizes that each found one wall gap
        # and disagree about which is a real ambiguity in this frame, not a
        # ranking the encoder is entitled to resolve.
        keep = [g for g in clusters if len(g) == len(clusters[0])][:3] if clusters else []
        openings = [
            {
                "x": round(sum(p[0] for p in g) / len(g), 2),
                "y": round(sum(p[1] for p in g) / len(g), 2),
                "votes": len(g),
            }
            for g in keep
        ]
        if not openings:
            # The negative carries the whole doorway answer on frames with no
            # door in them, so it is stated outright -- but the three tests
            # that were not passed need no restating when nothing passed them,
            # and the read-me-as guards below apply to a list that is empty.
            return {
                "desc": "No door-width opening was found in this frame: no gap between "
                "two obstacle faces is 0.7-1.0 m wide, walled on both sides, with mapped "
                "floor running clear through it. Nothing here is a doorway.",
                "openings": [],
            }
        if len(openings) == 1:
            # A single opening has no tie to explain and no ranking to justify,
            # so the vote-tie clause is replaced -- not deleted -- by the same
            # three tests stated once. The "these are openings, not obstacles"
            # negative is kept verbatim: it is what stops the gap being read
            # back as a return by the clearance, nearest and extent questions.
            return {
                "desc": "One door-width opening in the body-height structure, world "
                "meters: a 0.7-1.0 m free span between two obstacle faces, each face "
                "continuing as solid structure for at least 0.4 m to its own side, "
                "with mapped floor running clear 0.6 m straight through on both faces "
                "-- a passage the robot could walk through, not a slot between two "
                "objects or the end of a wall. votes is how many of the four cell "
                "sizes agreed. These are openings, not obstacles: never read them as "
                "returns, and never use them for clearance, nearest-obstacle, extent, "
                "area, footprint or direction answers.",
                "openings": openings,
            }
        return {
            "desc": "Door-width openings in the body-height structure, world meters. A "
            "gap is listed only when all three hold: the free span between two obstacle "
            "faces is 0.7-1.0 m; each face continues as solid structure for at least "
            "0.4 m to its own side, so the gap is a break in something rather than the "
            "end of it; and mapped floor runs clear for 0.6 m straight through the gap "
            "on both faces, so it is a passage and not a slot between two objects. The "
            "scan is repeated at four cell sizes and votes is how many agreed; only "
            "the openings with the top vote count are listed, so more than one entry "
            "means the frame genuinely does not settle which of them is the door and "
            "each is a place the robot could walk through. These are openings, "
            "not obstacles: never read them as returns, and never use them for "
            "clearance, nearest-obstacle, extent, area, footprint or direction "
            "answers.",
            "openings": openings,
        }

    GROUND_CELL = 0.25
    """Side of the x-y cell the floor surface is sampled on, meters."""

    GROUND_STEP = 0.15
    """How far a cell's floor must sit off the main floor to count as off-level."""

    @classmethod
    def _ground_cells(cls, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """Per-cell floor height: (ix, iy, z) with z the cell's 10th-percentile.

        The low percentile is the floor under whatever else the cell holds --
        a chair leg or a wall face sits above its own floor return, so it does
        not drag the surface up.
        """
        cell = cls.GROUND_CELL
        ix = np.floor(pts[:, 0] / cell).astype(np.int64)
        iy = np.floor(pts[:, 1] / cell).astype(np.int64)
        ox, oy = int(ix.min()), int(iy.min())
        key = (ix - ox) * (1 << 21) + (iy - oy)
        order = np.lexsort((pts[:, 2], key))
        key_s, z_s = key[order], pts[order, 2]
        starts = np.flatnonzero(np.concatenate(([True], key_s[1:] != key_s[:-1])))
        counts = np.diff(np.append(starts, key_s.size))
        keep = counts >= 4  # a cell needs a few returns before its floor means anything
        starts, counts = starts[keep], counts[keep]
        if starts.size < 8:
            return None
        gz = z_s[starts + np.minimum(counts - 1, (0.10 * counts).astype(np.int64))]
        k = key_s[starts]
        return (k >> 21) + ox, (k & ((1 << 21) - 1)) + oy, gz

    @classmethod
    def _ground_relief(cls, pts: np.ndarray, max_areas: int = 4) -> dict[str, object] | None:
        """Where the floor itself is not at the main level, and by how much.

        The body-height band above says where returns are; it says nothing
        about the surface underneath them, so a platform, a landing or a run of
        steps reads as ordinary floor. This raster carries that: one floor
        height per cell, the main floor as their median, and each connected
        patch that departs from it.
        """
        got = cls._ground_cells(pts)
        if got is None:
            return None
        cix, ciy, gz = got
        cell = cls.GROUND_CELL
        floor = float(np.median(gz))
        dz = gz - floor
        off = np.abs(dz) > cls.GROUND_STEP
        areas: list[dict[str, object]] = []
        if off.any():
            index = {(int(a), int(b)): i for i, (a, b) in enumerate(zip(cix, ciy, strict=False))}
            # One empty cell of slack, so a sensor gap does not split one patch in two.
            steps = [(dx, dy) for dx in (-2, -1, 0, 1, 2) for dy in (-2, -1, 0, 1, 2)]
            label = np.full(cix.size, -1, dtype=np.int64)
            groups: list[list[int]] = []
            for seed in range(cix.size):
                if not off[seed] or label[seed] >= 0:
                    continue
                label[seed] = len(groups)
                stack, members = [seed], [seed]
                while stack:
                    j = stack.pop()
                    for dx, dy in steps:
                        i = index.get((int(cix[j]) + dx, int(ciy[j]) + dy))
                        if i is None or label[i] >= 0 or not off[i]:
                            continue
                        if (dz[i] > 0) != (dz[j] > 0):
                            continue  # a rise and a drop are not the same feature
                        label[i] = label[seed]
                        stack.append(i)
                        members.append(i)
                groups.append(members)
            min_cells = int(round(1.0 / (cell * cell)))  # ignore anything under 1 m2
            for members in sorted(groups, key=len, reverse=True):
                if len(members) < min_cells:
                    break
                m = np.array(members)
                d = dz[m]
                up = float(d.mean()) > 0
                areas.append(
                    {
                        "x": round(float((cix[m] * cell + cell / 2).mean()), 1),
                        "y": round(float((ciy[m] * cell + cell / 2).mean()), 1),
                        "area_m2": round(len(members) * cell * cell, 1),
                        "dz_m": round(float(np.median(d)), 2),
                        "far_dz_m": round(float(np.percentile(d, 90 if up else 10)), 2),
                    }
                )
                if len(areas) >= max_areas:
                    break
        return {
            "desc": "The floor surface itself, world meters -- the body-height map above "
            "cannot see it. floor_z is the main floor level; areas lists each connected "
            "patch of floor of at least 1 m2 whose own level is more than 0.15 m off it "
            "(a raised platform, a landing, a run of steps, a sunken area), with x,y its "
            "centre, dz_m its typical level relative to the main floor (+ above, - below) "
            "and far_dz_m the level its floor reaches at its far end. dz_m and far_dz_m "
            "are equal across a flat platform; far_dz_m is the larger where the floor "
            "keeps climbing across the patch, so it is the full bottom-to-top rise of a "
            "flight of steps and dz_m is the height of the surface it lands on. An empty "
            "areas means every cell of mapped floor sits within 0.15 m of floor_z -- the "
            "floor is level, with no step, ramp, platform or drop anywhere the sensor saw "
            "it. These are floor heights only; for whole-cloud z use exact_stats.",
            "floor_z": round(floor, 2),
            "areas": areas,
        }

    # Enclosure topology. The obstacle list below says where returns are; it
    # says nothing about which floor those returns enclose. Eroding the mapped
    # free floor by half a doorway severs every doorway-width passage, so what
    # survives is one blob per enclosed area and the cuts are the openings
    # between them -- topology, computed, instead of left to be inferred from a
    # flat list of extents.
    _TOPO_CELL = 0.15  # base grid pitch, meters
    _TOPO_MAX_DIM = 240  # coarsen rather than exceed this many cells per axis
    _TOPO_ERODE = 0.6  # half the widest passage that still counts as a doorway
    _TOPO_MIN_AREA = 2.5  # m^2 of surviving core below which it is not an area

    @staticmethod
    def _disk(radius_cells: float) -> np.ndarray:
        """Boolean disk structuring element of the given radius, in cells."""
        n = int(np.ceil(radius_cells))
        yy, xx = np.mgrid[-n : n + 1, -n : n + 1]
        disk: np.ndarray = (xx * xx + yy * yy) <= radius_cells * radius_cells + 1e-9
        return disk

    @classmethod
    def _enclosure_topology(cls, xy: np.ndarray, body: np.ndarray) -> dict[str, object] | None:
        """How many separately enclosed areas this cloud covers, and their pinches.

        ponytail: the room count is a connected-component count of eroded free
        floor, not a count of obstacle runs -- which is what a flat extent list
        invites the reader to count instead.
        """
        if xy.shape[0] == 0:
            return None
        lo = xy.min(axis=0)
        span = float((xy.max(axis=0) - lo).max())
        cell = max(cls._TOPO_CELL, span / cls._TOPO_MAX_DIM)
        ij = np.floor((xy - lo) / cell).astype(np.int64)
        h = int(ij[:, 1].max()) + 1
        w = int(ij[:, 0].max()) + 1
        if h < 8 or w < 8:  # too small a footprint for topology to mean anything
            return None
        mapped = np.zeros((h, w), bool)
        mapped[ij[:, 1], ij[:, 0]] = True
        occupied = np.zeros((h, w), bool)
        obstacles = ij[body]
        occupied[obstacles[:, 1], obstacles[:, 0]] = True
        interior = ndimage.binary_fill_holes(ndimage.binary_closing(mapped, cls._disk(1.5)))
        free = ndimage.binary_opening(interior & ~occupied, cls._disk(1.0))
        core = ndimage.binary_erosion(free, cls._disk(cls._TOPO_ERODE / cell))
        labels, count = ndimage.label(core)
        if count == 0:
            return None
        px = cell * cell
        sizes = ndimage.sum(np.ones_like(labels, dtype=np.float64), labels, range(1, count + 1))
        keep = [i + 1 for i in range(count) if sizes[i] * px >= cls._TOPO_MIN_AREA]
        if not keep:
            return None
        remap = np.zeros(count + 1, np.int32)
        for new, old in enumerate(keep, start=1):
            remap[old] = new
        marks = remap[labels]
        found = []
        for k in range(1, len(keep) + 1):
            ys, xs = np.nonzero(marks == k)
            found.append(
                (
                    float(xs.mean() * cell + lo[0]),
                    float(ys.mean() * cell + lo[1]),
                    float(ys.size * px),
                )
            )
        found.sort(key=lambda a: -a[2])
        openings: list[tuple[float, float, float]] = []
        if len(keep) > 1:
            # Every free cell claims the nearest surviving core; where two
            # claims meet is exactly the passage the erosion cut.
            _, nearest = ndimage.distance_transform_edt(marks == 0, return_indices=True)
            claim = marks[nearest[0], nearest[1]] * free
            edge = np.zeros_like(free)
            for axis in (0, 1):
                a, b = (claim[:-1, :], claim[1:, :]) if axis == 0 else (claim[:, :-1], claim[:, 1:])
                split = (a > 0) & (b > 0) & (a != b)
                if axis == 0:
                    edge[:-1, :] |= split
                    edge[1:, :] |= split
                else:
                    edge[:, :-1] |= split
                    edge[:, 1:] |= split
            elabels, ecount = ndimage.label(edge, structure=np.ones((3, 3)))
            for k in range(1, ecount + 1):
                ys, xs = np.nonzero(elabels == k)
                width = float(max(ys.max() - ys.min(), xs.max() - xs.min()) * cell + cell)
                openings.append(
                    (float(xs.mean() * cell + lo[0]), float(ys.mean() * cell + lo[1]), width)
                )
            openings.sort(key=lambda a: a[2])
        areas_s = ";".join(f"{a:.2f},{b:.2f}@{c:.1f}" for a, b, c in found)
        if len(keep) == 1:
            # One surviving blob and nothing cut it. The openings key would be
            # empty and the pinch format has nothing to describe, so both are
            # replaced by an explicit negative rather than dropped: the room
            # count, the not-floor-area guard and the not-a-position-track
            # guard all still have to be said, because they are what stop the
            # reader counting wall runs as rooms or reading cx,cy as motion.
            return {
                "desc": "separate_areas 1: free floor (mapped floor minus body-height "
                "obstacles) eroded by 0.60 m, severing every passage under 1.2 m, is "
                "one undivided enclosed area -- no interior doorway-width pinch. "
                "separate_areas is already the room or area count; obstacle boxes, "
                "wall runs and y-bands in body_height_occupancy are wall and furniture "
                "pieces and counting those overcounts rooms several-fold. areas is "
                "cx,cy@m2 of post-erosion free floor, NOT mapped floor area (use "
                "occupied_floor_footprint_m2 above for floor area, footprint or an "
                "area-trend) and NOT a position track: for how far the mapped region's "
                "centre moved, or which direction the map moved or gained coverage, "
                "take dx,dy from centroid_xy_m exactly as the compass note says.",
                "separate_areas": 1,
                "areas": areas_s,
            }
        return {
            "desc": "Enclosure topology of this cloud's floor. Mapped floor minus "
            "body-height obstacles was eroded by 0.60 m, which severs every passage "
            "narrower than 1.2 m; each blob that survives is one separately enclosed "
            "area, and each cut is a doorway-width opening between two of them. "
            "separate_areas is therefore how many enclosed rooms this cloud covers, "
            "under the rule that two areas are separate exactly when the only way "
            "between them is a doorway-width opening. It is already the answer to a "
            "room or area count; the obstacle boxes, wall runs and y-bands in "
            "body_height_occupancy are pieces of wall and furniture, and counting "
            "those instead overcounts rooms several-fold. openings are the pinches "
            "between the areas as x,y@width_m. areas are "
            "cx,cy@m2 of post-erosion free floor -- that m2 is NOT mapped floor "
            "area, so for floor area, footprint or an area-trend (how the mapped "
            "floor area changed) use "
            "occupied_floor_footprint_m2 above instead. These cx,cy are not a "
            "position track: for how far the mapped region's centre moved, or which "
            "direction the map moved or gained coverage, take dx,dy from "
            "centroid_xy_m exactly as the compass note says.",
            "separate_areas": len(keep),
            "areas": areas_s,
            "openings": ";".join(f"{a:.2f},{b:.2f}@{c:.2f}" for a, b, c in openings[:3]),
        }

    @staticmethod
    def _body_height_occupancy(xy: np.ndarray, max_cells: int = 28) -> dict[str, object] | None:
        """Exact bounding boxes of body-height obstacle clusters, world meters.

        ponytail: y binned into bands only to segment clusters; the emitted
        extents are exact point min/max, so clearance is closed-form
        point-to-box in both axes.
        """
        if xy.shape[0] == 0:
            return None
        lo = xy.min(axis=0)
        hi = xy.max(axis=0)
        span = float(max(hi[0] - lo[0], hi[1] - lo[1]))
        cell = next((c for c in (0.25, 0.4, 0.8, 1.6, 3.2) if span / c < max_cells), 6.4)
        iy = np.floor((xy[:, 1] - lo[1]) / cell).astype(int)
        parts = []
        for r in range(int(iy.max()), -1, -1):
            sel = xy[iy == r]
            if sel.shape[0] == 0:
                continue
            sel = sel[np.argsort(sel[:, 0])]
            rx = sel[:, 0]
            breaks = np.flatnonzero(np.diff(rx) > cell)
            starts = np.concatenate(([0], breaks + 1))
            ends = np.concatenate((breaks, [rx.size - 1]))
            for s, e in zip(starts, ends, strict=False):
                a, b = f"{rx[s]:.2f}", f"{rx[e]:.2f}"
                run = a if a == b else f"{a}:{b}"
                ry = sel[s : e + 1, 1]
                ya, yb = f"{ry.min():.2f}", f"{ry.max():.2f}"
                run += f"@{ya}" if ya == yb else f"@{ya}:{yb}"
                parts.append(run)
        return {
            "desc": "Exact bounding boxes (world meters) of obstacle points at "
            "body height (z 0.15..1.0 m), listed north to south. Each box "
            "xmin:xmax@ymin:ymax is the exact extent of its points (+x east, "
            "+y north; a lone value = zero-width, a thin obstacle). "
            "Horizontal clearance from a query point (qx,qy) = min over all "
            "boxes of hypot(dx,dy), where dx = max(0, xmin-qx, qx-xmax) and "
            "dy = max(0, ymin-qy, qy-ymax) (each term is zero only when the "
            "query lies inside that extent).",
            "boxes": ",".join(parts),
        }

    @functools.cached_property
    def center(self) -> Vector3:
        """Calculate the center of the pointcloud in world frame."""
        center = np.asarray(self.pointcloud.points).mean(axis=0)
        return Vector3(*center)

    def points(self):  # type: ignore[no-untyped-def]
        """Get points (returns tensor positions, use as_numpy() for numpy array)."""
        import open3d.core as o3c  # type: ignore[import-untyped]

        self._ensure_tensor_initialized()
        if "positions" not in self._pcd_tensor.point:
            return o3c.Tensor(np.zeros((0, 3), dtype=np.float32))
        return self._pcd_tensor.point["positions"]

    def __add__(self, other: PointCloud2) -> PointCloud2:
        """Combine two PointCloud2 instances into one.

        The resulting point cloud contains points from both inputs.
        The frame_id and timestamp are taken from the first point cloud.

        Args:
            other: Another PointCloud2 instance to combine with

        Returns:
            New PointCloud2 instance containing combined points
        """
        if not isinstance(other, PointCloud2):
            raise ValueError("Can only add PointCloud2 to another PointCloud2")

        return PointCloud2(
            pointcloud=self.pointcloud + other.pointcloud,
            frame_id=self.frame_id,
            ts=max(self.ts, other.ts),
        )

    def transform(self, tf: Transform) -> PointCloud2:
        """Transform the pointcloud using a Transform object.

        Applies the rotation and translation from the transform to all points,
        converting them into the transform's frame_id.

        Args:
            tf: Transform object containing rotation and translation

        Returns:
            New PointCloud2 instance with transformed points in the new frame
        """
        import open3d as o3d  # type: ignore[import-untyped]

        points, _ = self.as_numpy()

        if len(points) == 0:
            return PointCloud2(
                pointcloud=o3d.geometry.PointCloud(),
                frame_id=tf.frame_id,
                ts=self.ts,
            )

        # Build 4x4 transformation matrix from Transform
        transform_matrix = tf.to_matrix()

        # Convert points to homogeneous coordinates (N, 4)
        ones = np.ones((len(points), 1))
        points_homogeneous = np.hstack([points, ones])

        # Apply transformation: (4, 4) @ (4, N) -> (4, N) -> transpose to (N, 4)
        transformed_points = (transform_matrix @ points_homogeneous.T).T

        # Extract xyz coordinates (drop homogeneous coordinate)
        transformed_xyz = transformed_points[:, :3].astype(np.float64)

        # Create new Open3D point cloud
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(transformed_xyz)

        # Colors are frame-independent, carry them through.
        if self.pointcloud.has_colors():
            new_pcd.colors = self.pointcloud.colors

        return PointCloud2(
            pointcloud=new_pcd,
            frame_id=tf.frame_id,
            ts=self.ts,
        )

    def voxel_downsample(self, voxel_size: float = 0.025) -> PointCloud2:
        """Downsample the pointcloud with a voxel grid."""
        if voxel_size <= 0:
            return self
        if len(self.pointcloud.points) < 20:
            return self
        downsampled = self._pcd_tensor.voxel_down_sample(voxel_size)
        return PointCloud2(pointcloud=downsampled, frame_id=self.frame_id, ts=self.ts)

    def as_numpy(
        self,
    ) -> tuple[np.ndarray[Any, Any], np.ndarray[Any, Any] | None]:
        """Get points and colors as numpy arrays.

        Returns:
            Tuple of (points, colors) where:
            - points: Nx3 numpy array of 3D points
            - colors: Nx3 array in [0, 1] range, or None if no colors
        """
        points = np.asarray(self.pointcloud.points)
        colors = np.asarray(self.pointcloud.colors) if self.pointcloud.has_colors() else None
        return points, colors

    def points_f32(self) -> np.ndarray:
        """Get positions as float32 numpy array, bypassing legacy float64 conversion."""
        self._ensure_tensor_initialized()
        if "positions" in self._pcd_tensor.point:
            arr = self._pcd_tensor.point["positions"].numpy()
            return arr.astype(np.float32) if arr.dtype != np.float32 else arr  # type: ignore[no-any-return]
        return np.zeros((0, 3), dtype=np.float32)

    def intensities_f32(self) -> np.ndarray | None:
        """Get per-point intensity values as a flat float32 array, or None if absent."""
        self._ensure_tensor_initialized()
        if "intensities" in self._pcd_tensor.point:
            arr = self._pcd_tensor.point["intensities"].numpy().flatten()
            return arr.astype(np.float32) if arr.dtype != np.float32 else arr  # type: ignore[no-any-return]
        return None

    @functools.cached_property
    def axis_aligned_bounding_box(self) -> o3d.geometry.AxisAlignedBoundingBox:
        """Get axis-aligned bounding box of the point cloud."""
        return self.pointcloud.get_axis_aligned_bounding_box()

    @functools.cached_property
    def oriented_bounding_box(self) -> o3d.geometry.OrientedBoundingBox:
        """Get oriented bounding box of the point cloud."""
        return self.pointcloud.get_oriented_bounding_box()

    @functools.cached_property
    def bounding_box_dimensions(self) -> tuple[float, float, float]:
        """Get dimensions (width, height, depth) of axis-aligned bounding box."""
        bbox = self.axis_aligned_bounding_box
        extent = bbox.get_extent()
        return tuple(extent)

    def bounding_box_intersects(self, other: PointCloud2) -> bool:
        # Get axis-aligned bounding boxes
        bbox1 = self.axis_aligned_bounding_box
        bbox2 = other.axis_aligned_bounding_box

        # Get min and max bounds
        min1 = bbox1.get_min_bound()
        max1 = bbox1.get_max_bound()
        min2 = bbox2.get_min_bound()
        max2 = bbox2.get_max_bound()

        # Check overlap in all three dimensions
        # Boxes intersect if they overlap in ALL dimensions
        return (  # type: ignore[no-any-return]
            min1[0] <= max2[0]
            and max1[0] >= min2[0]
            and min1[1] <= max2[1]
            and max1[1] >= min2[1]
            and min1[2] <= max2[2]
            and max1[2] >= min2[2]
        )

    def lcm_encode(self, frame_id: str | None = None) -> bytes:
        """Convert to LCM PointCloud2 message with optional RGB colors."""
        msg = LCMPointCloud2()

        # Header
        msg.header = Header()
        msg.header.seq = 0
        msg.header.frame_id = frame_id or self.frame_id

        msg.header.stamp.sec = int(self.ts)
        msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)

        points, _ = self.as_numpy()

        # Check if pointcloud has colors
        self._ensure_tensor_initialized()
        has_colors = "colors" in self._pcd_tensor.point

        if len(points) == 0:
            msg.height = 0
            msg.width = 0
            msg.point_step = 16
            msg.row_step = 0
            msg.data_length = 0
            msg.data = b""
            msg.is_dense = True
            msg.is_bigendian = False
            msg.fields_length = 4
            msg.fields = self._create_xyzrgb_fields() if has_colors else self._create_xyz_fields()
            return msg.lcm_encode()  # type: ignore[no-any-return]

        msg.height = 1
        msg.width = len(points)

        if has_colors:
            # Get colors (0-1 range) and convert to uint8
            colors = self._pcd_tensor.point["colors"].numpy()
            if colors.max() <= 1.0:
                colors = (colors * 255).astype(np.uint8)
            else:
                colors = colors.astype(np.uint8)

            # Pack RGB into float32 (ROS convention: bytes are [padding, r, g, b])
            rgb_packed = np.zeros(len(points), dtype=np.float32)
            rgb_uint32 = (
                (colors[:, 0].astype(np.uint32) << 16)
                | (colors[:, 1].astype(np.uint32) << 8)
                | colors[:, 2].astype(np.uint32)
            )
            rgb_packed = rgb_uint32.view(np.float32)

            msg.fields = self._create_xyzrgb_fields()
            msg.fields_length = 4
            msg.point_step = 16  # x, y, z, rgb (4 floats)

            point_data = np.column_stack([points, rgb_packed]).astype(np.float32)
        else:
            msg.fields = self._create_xyz_fields()
            msg.fields_length = 4
            msg.point_step = 16  # x, y, z, intensity

            if "intensities" in self._pcd_tensor.point:
                intensities = (
                    self._pcd_tensor.point["intensities"].numpy().flatten().astype(np.float32)
                )
            else:
                intensities = np.zeros(len(points), dtype=np.float32)

            point_data = np.column_stack([points, intensities]).astype(np.float32)

        msg.row_step = msg.point_step * msg.width
        data_bytes = point_data.tobytes()
        msg.data_length = len(data_bytes)
        msg.data = data_bytes

        msg.is_dense = True
        msg.is_bigendian = False

        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_warmup(cls) -> None:
        """Preload the heavy imports lcm_decode needs.

        Called at subscribe time (see LCMEncoderMixin.subscribe) so the first
        decode doesn't stall the LCM handler thread on the open3d import.
        """
        import open3d.core  # type: ignore[import-untyped] # noqa: F401

    @classmethod
    def lcm_decode(cls, data: bytes) -> PointCloud2:
        import open3d as o3d  # type: ignore[import-untyped]
        import open3d.core as o3c  # type: ignore[import-untyped]

        msg = LCMPointCloud2.lcm_decode(data)

        if msg.width == 0 or msg.height == 0:
            pc = o3d.geometry.PointCloud()
            return cls(
                pointcloud=pc,
                frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
                ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
                if hasattr(msg, "header") and msg.header.stamp.sec > 0
                else None,
            )

        # Parse field offsets
        x_offset = y_offset = z_offset = rgb_offset = intensity_offset = None
        for msgfield in msg.fields:
            if msgfield.name == "x":
                x_offset = msgfield.offset
            elif msgfield.name == "y":
                y_offset = msgfield.offset
            elif msgfield.name == "z":
                z_offset = msgfield.offset
            elif msgfield.name == "rgb":
                rgb_offset = msgfield.offset
            elif msgfield.name == "intensity":
                intensity_offset = msgfield.offset

        if any(offset is None for offset in [x_offset, y_offset, z_offset]):
            raise ValueError("PointCloud2 message missing X, Y, or Z msgfields")

        num_points = msg.width * msg.height
        raw_data = msg.data
        point_step = msg.point_step

        # Fast path for standard layout
        if x_offset == 0 and y_offset == 4 and z_offset == 8 and point_step >= 12:
            if point_step == 12:
                points = np.frombuffer(raw_data, dtype=np.float32).reshape(-1, 3)
            else:
                dt = np.dtype(
                    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("_pad", f"V{point_step - 12}")]
                )
                structured = np.frombuffer(raw_data, dtype=dt, count=num_points)
                points = np.column_stack((structured["x"], structured["y"], structured["z"]))
        else:
            points = np.zeros((num_points, 3), dtype=np.float32)
            for i in range(num_points):
                base_offset = i * point_step
                points[i, 0] = struct.unpack(
                    "<f", raw_data[base_offset + x_offset : base_offset + x_offset + 4]
                )[0]
                points[i, 1] = struct.unpack(
                    "<f", raw_data[base_offset + y_offset : base_offset + y_offset + 4]
                )[0]
                points[i, 2] = struct.unpack(
                    "<f", raw_data[base_offset + z_offset : base_offset + z_offset + 4]
                )[0]

        # Create tensor pointcloud
        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)

        # Extract intensity if present
        if intensity_offset is not None and rgb_offset is None:
            dt_i = np.dtype(
                [
                    ("_pre", f"V{intensity_offset}"),
                    ("intensity", "<f4"),
                    ("_post", f"V{point_step - intensity_offset - 4}"),
                ]
            )
            structured_i = np.frombuffer(raw_data, dtype=dt_i, count=num_points)
            intensities = structured_i["intensity"].astype(np.float32)
            if np.any(intensities != 0):
                pcd_t.point["intensities"] = o3c.Tensor(
                    intensities.reshape(-1, 1), dtype=o3c.float32
                )

        # Extract RGB colors if present
        if rgb_offset is not None:
            dt = np.dtype(
                [
                    ("_pre", f"V{rgb_offset}"),
                    ("rgb", "<f4"),
                    ("_post", f"V{point_step - rgb_offset - 4}"),
                ]
            )
            structured = np.frombuffer(raw_data, dtype=dt, count=num_points)
            rgb_packed = structured["rgb"].view(np.uint32)
            r = ((rgb_packed >> 16) & 0xFF).astype(np.float32) / 255.0
            g = ((rgb_packed >> 8) & 0xFF).astype(np.float32) / 255.0
            b = (rgb_packed & 0xFF).astype(np.float32) / 255.0
            colors = np.column_stack([r, g, b])
            pcd_t.point["colors"] = o3c.Tensor(colors, dtype=o3c.float32)

        return cls(
            pointcloud=pcd_t,
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
            if hasattr(msg, "header") and msg.header.stamp.sec > 0
            else None,
        )

    def _create_xyz_fields(self) -> list:  # type: ignore[type-arg]
        """Create X, Y, Z, intensity field definitions."""
        fields = []
        for i, name in enumerate(["x", "y", "z", "intensity"]):
            field = PointField()
            field.name = name
            field.offset = i * 4
            field.datatype = 7  # FLOAT32
            field.count = 1
            fields.append(field)
        return fields

    def _create_xyzrgb_fields(self) -> list:  # type: ignore[type-arg]
        """Create X, Y, Z, RGB field definitions for colored pointclouds."""
        fields = []
        for i, name in enumerate(["x", "y", "z"]):
            field = PointField()
            field.name = name
            field.offset = i * 4
            field.datatype = 7  # FLOAT32
            field.count = 1
            fields.append(field)

        # RGB field (packed as float32, ROS convention)
        rgb_field = PointField()
        rgb_field.name = "rgb"
        rgb_field.offset = 12
        rgb_field.datatype = 7  # FLOAT32 (contains packed RGB)
        rgb_field.count = 1
        fields.append(rgb_field)

        return fields

    def __len__(self) -> int:
        """Return number of points."""
        self._ensure_tensor_initialized()
        if "positions" not in self._pcd_tensor.point:
            return 0
        return int(self._pcd_tensor.point["positions"].shape[0])

    def to_rerun(
        self,
        voxel_size: float = 0.05,
        colors: list[int] | None = None,
        mode: str = "spheres",
        fill_mode: str = "solid",
        bottom_cutoff: float | None = None,
        **kwargs: object,
    ) -> Archetype:
        """Convert to Rerun archetype for visualization.

        Args:
            voxel_size: size for visualization
            colors: Optional RGB color [r, g, b] for all points (0-255).
                If None, uses height-based turbo colormap via class_ids
                (requires register_colormap_annotation() called once).
            mode: "points" for raw points, "boxes" for cubes (default), or "spheres" for sized spheres
            fill_mode: Fill mode for boxes - "solid", "majorwireframe", or "densewireframe"
            **kwargs: Additional args (ignored for compatibility)

        Returns:
            rr.Points3D or rr.Boxes3D archetype for logging to Rerun
        """
        import rerun as rr

        points = self.points_f32()
        if len(points) == 0:
            return rr.Points3D([]) if mode != "boxes" else rr.Boxes3D(centers=[])

        if bottom_cutoff is not None:
            points = points[points[:, 2] >= bottom_cutoff]
            if len(points) == 0:
                return rr.Points3D([]) if mode != "boxes" else rr.Boxes3D(centers=[])

        # Use class_ids for height-based colormap (viewer resolves colors via AnnotationContext)
        # Fall back to explicit colors when provided
        class_ids = None
        point_colors = None
        if colors is not None:
            point_colors = colors
        else:
            z = points[:, 2]
            class_ids = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint8)

        if mode == "points":
            return rr.Points3D(
                positions=points, colors=point_colors, class_ids=class_ids, radii=voxel_size / 2
            )
        elif mode == "boxes":
            half = voxel_size / 2
            return rr.Boxes3D(
                centers=points,
                half_sizes=[half, half, half],
                colors=point_colors,
                class_ids=class_ids,
                fill_mode=fill_mode,  # type: ignore[arg-type]
            )
        else:
            return rr.Points3D(
                positions=points,
                radii=voxel_size / 2,
                colors=point_colors,
                class_ids=class_ids,
            )

    def filter_by_height(
        self,
        min_height: float | None = None,
        max_height: float | None = None,
    ) -> PointCloud2:
        """Filter points based on their height (z-coordinate).

        This method creates a new PointCloud2 containing only points within the specified
        height range. All metadata (frame_id, timestamp) is preserved.

        Args:
            min_height: Optional minimum height threshold. Points with z < min_height are filtered out.
                       If None, no lower limit is applied.
            max_height: Optional maximum height threshold. Points with z > max_height are filtered out.
                       If None, no upper limit is applied.

        Returns:
            New PointCloud2 instance containing only the filtered points.

        Raises:
            ValueError: If both min_height and max_height are None (no filtering would occur).

        Example:
            # Remove ground points below 0.1m height
            filtered_pc = pointcloud.filter_by_height(min_height=0.1)

            # Keep only points between ground level and 2m height
            filtered_pc = pointcloud.filter_by_height(min_height=0.0, max_height=2.0)

            # Remove points above 1.5m (e.g., ceiling)
            filtered_pc = pointcloud.filter_by_height(max_height=1.5)
        """
        import open3d as o3d  # type: ignore[import-untyped]

        # Validate that at least one threshold is provided
        if min_height is None and max_height is None:
            raise ValueError("At least one of min_height or max_height must be specified")

        # Get points as numpy array
        points, _ = self.as_numpy()

        if len(points) == 0:
            # Empty pointcloud - return a copy
            return PointCloud2(
                pointcloud=o3d.geometry.PointCloud(),
                frame_id=self.frame_id,
                ts=self.ts,
            )

        # Extract z-coordinates (height values) - column index 2
        heights = points[:, 2]

        # Create boolean mask for filtering based on height thresholds
        # Start with all True values
        mask = np.ones(len(points), dtype=bool)

        # Apply minimum height filter if specified
        if min_height is not None:
            mask &= heights >= min_height

        # Apply maximum height filter if specified
        if max_height is not None:
            mask &= heights <= max_height

        # Apply mask to filter points
        filtered_points = points[mask]

        # Create new PointCloud2 with filtered points
        return PointCloud2.from_numpy(
            points=filtered_points,
            frame_id=self.frame_id,
            timestamp=self.ts,
        )

    def __repr__(self) -> str:
        """String representation."""
        return f"PointCloud(points={len(self)}, frame_id='{self.frame_id}', ts={self.ts})"
