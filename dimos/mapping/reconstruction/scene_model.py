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

"""Full 3D model of a scanned space, sliceable into cross-sections.

`SceneModel` fuses everything a mapping session produced — the lidar point
cloud and robot trajectory from a Rerun recording, and optionally the camera
stream from a recording session (`mem2.db` + `gtsam_odom.tum`) to colorize the
geometry — into one queryable model:

    model = SceneModel.from_rrd("building.rrd")
    model.colorize_from_session(Path("~/session_dir"))       # optional, RGB
    plan = model.horizontal_section(z=1.5)                   # plan cut
    elev = model.vertical_section((0, -5), (20, -5))         # section cut
    plan.save_png("plan.png")
    model.save_rerun("model.rrd")                            # dimos-viewer model.rrd
    mesh = model.to_mesh()                                   # o3d TriangleMesh

Downstream consumers (e.g. the floorplan generator, wishlist item 1) can build
on sections instead of re-implementing their own point-cloud slicing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Rotation matrix from a (x, y, z, w) quaternion.

    Thin wrapper over the shared Quaternion type so the repo keeps ONE
    quaternion->matrix implementation (a zero-norm quaternion yields identity).
    """
    if not (qx or qy or qz or qw):
        return np.eye(3)
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion

    return Quaternion(qx, qy, qz, qw).to_rotation_matrix()


def load_rrd_points(path: Path | str) -> tuple[np.ndarray, np.ndarray]:
    """Extract the point cloud and trajectory vertices from a .rrd recording.

    Collects every `Points3D` batch (the map) and every `LineStrips3D` strip
    (the robot path) in the recording. Returns `(points (N,3), trajectory
    (M,3))`, both float32, unclipped.
    """
    import rerun_bindings as rb

    rec = rb.load_recording(str(path))
    cloud: list[np.ndarray] = []
    path_pts: list[list[float]] = []
    for chunk in rec.chunks():
        table = chunk.to_record_batch()
        for name, col in zip(table.schema.names, table.columns):
            if "Points3D:positions" in name:
                # Fast path: flatten list<fixed_size_list<f32>[3]> straight from
                # the arrow buffers. to_pylist() materializes every coordinate
                # as a Python float — minutes instead of seconds on map-scale
                # recordings (measured in rrd_feed.py on the same schema).
                try:
                    flat = col.values.values.to_numpy(zero_copy_only=False)
                    if len(flat):
                        cloud.append(np.asarray(flat, dtype=np.float32).reshape(-1, 3))
                except Exception:
                    for row in col.to_pylist():
                        if row:
                            cloud.append(np.asarray(row, dtype=np.float32))
            elif "LineStrips3D:strips" in name:
                for row in col.to_pylist():
                    if row:
                        for strip in row:
                            path_pts.extend(strip)

    points = np.vstack(cloud) if cloud else np.zeros((0, 3), dtype=np.float32)
    trajectory = (
        np.asarray(path_pts, dtype=np.float32) if path_pts else np.zeros((0, 3), np.float32)
    )
    return points, trajectory


@dataclass
class Section:
    """A planar cut through the model: density raster + optional mean color."""

    density: np.ndarray  # (H, W) int32 — lidar hits per cell
    color: np.ndarray | None  # (H, W, 3) uint8 — mean RGB where colorized
    extent: tuple[float, float, float, float]  # u_lo, v_lo, u_hi, v_hi (m)
    axes: tuple[str, str]  # axis labels, e.g. ("x [m]", "y [m]")

    def occupancy(self, min_hits: int = 1) -> np.ndarray:
        """Boolean occupancy mask at the given hit threshold."""
        return self.density >= min_hits

    def save_png(self, path: Path | str, title: str | None = None) -> None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        u_lo, v_lo, u_hi, v_hi = self.extent
        mpl_extent = (u_lo, u_hi, v_lo, v_hi)  # imshow wants (left, right, bottom, top)
        fig, ax = plt.subplots(figsize=(12, max(12 * (v_hi - v_lo) / max(u_hi - u_lo, 1e-6), 3)))
        if self.color is not None and self.color.any():
            shown = self.color.astype(np.float32) / 255.0
            empty = ~self.occupancy()
            shown[empty] = 1.0  # white background
            ax.imshow(shown, origin="lower", extent=mpl_extent, interpolation="nearest")
        else:
            with np.errstate(divide="ignore"):
                img = np.log1p(self.density.astype(np.float32))
            ax.imshow(img, origin="lower", extent=mpl_extent, cmap="Greys",
                      interpolation="nearest")
        ax.set_xlabel(self.axes[0])
        ax.set_ylabel(self.axes[1])
        ax.set_aspect("equal")
        if title:
            ax.set_title(title)
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)


def _ellipse(px: int) -> np.ndarray:
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (px, px))


@dataclass
class ClosedRegion:
    """A sealed connected area: interior free space + the closed shell around it.

    `bridged` marks shell cells with no raw lidar evidence — the gaps
    (glass, doorways, unscanned spans) that morphological closing sealed.
    """

    interior: np.ndarray  # (ny, nx) bool — flood-filled interior, leak-free
    shell: np.ndarray  # (ny, nx) bool — occupied cells adjacent to interior
    bridged: np.ndarray  # (ny, nx) bool — shell cells the closing inferred
    loops: list[np.ndarray] = field(default_factory=list)  # (K, 2) world m, first == last
    origin: tuple[float, float] = (0.0, 0.0)  # xy of cell (0, 0) min corner
    resolution: float = 0.1
    z_range: tuple[float, float] = (0.0, 0.0)
    closure_m: float = 0.0  # smallest closing kernel (m) that sealed the region


@dataclass
class VoxelOccupancy:
    """Dense 3D lidar-hit-count grid: the analytic voxel model of the scan.

    Dense on purpose: a building-scale scan (e.g. the china office, ~50x40x10 m)
    is 20M voxels at 0.1 m — 20 MB as uint8 — and per-z-layer slices need to be
    dense 2D arrays for cv2 anyway. `counts[k]` is exactly the z-layer raster
    (row = y, col = x, origin at the min corner), so `slab()` output shares the
    cell convention of `Section` rasters built with the same bounds/resolution.
    """

    counts: np.ndarray  # (nz, ny, nx) uint8 — saturating hit counts
    origin: tuple[float, float, float]  # world coords of the min corner of voxel [0,0,0]
    voxel: float  # cube edge, m

    @property
    def shape(self) -> tuple[int, int, int]:
        return self.counts.shape  # type: ignore[return-value]

    def occupancy(self, min_hits: int = 1) -> np.ndarray:
        return self.counts >= min_hits

    def z_of(self, k: int) -> float:
        """World z of the center of layer k."""
        return self.origin[2] + (k + 0.5) * self.voxel

    def layer_index(self, z: float) -> int:
        return int(np.clip((z - self.origin[2]) / self.voxel, 0, self.shape[0] - 1))

    def slab(self, z_lo: float, z_hi: float, min_hits: int = 1) -> np.ndarray:
        """(ny, nx) bool — cells whose column has >= min_hits in [z_lo, z_hi]."""
        k_lo = self.layer_index(z_lo)
        k_hi = self.layer_index(z_hi)
        return self.counts[k_lo : k_hi + 1].sum(axis=0, dtype=np.int32) >= min_hits

    def column_counts(self, z_lo: float, z_hi: float) -> np.ndarray:
        """(ny, nx) int32 — summed hit counts per column over [z_lo, z_hi]."""
        k_lo = self.layer_index(z_lo)
        k_hi = self.layer_index(z_hi)
        return self.counts[k_lo : k_hi + 1].sum(axis=0, dtype=np.int32)

    def xy_extent(self) -> tuple[float, float, float, float]:
        """(x_lo, y_lo, x_hi, y_hi) of the grid in world meters."""
        _, ny, nx = self.shape
        return (
            self.origin[0],
            self.origin[1],
            self.origin[0] + nx * self.voxel,
            self.origin[1] + ny * self.voxel,
        )

    def closed_region(
        self,
        z_lo: float,
        z_hi: float,
        seeds_xy: np.ndarray,
        min_hits: int = 2,
        gap_steps: tuple[float, ...] = (0.3, 0.6, 1.2, 2.5),
        barrier: np.ndarray | None = None,
    ) -> ClosedRegion | None:
        """Seal the connected area around `seeds_xy` between z_lo and z_hi.

        Watertightness by progressive morphological closing: try increasingly
        large closing kernels (`gap_steps`, meters) on the wall-evidence slab
        until the free space flooded from the seeds no longer leaks to the
        raster border. The smallest kernel that seals wins, so a 30 cm scan gap
        is bridged with a 30 cm kernel — an open patio is not welded shut just
        because a doorway elsewhere needed 1.2 m.

        2D per level, not 3D closing: walls are vertical, and a meter-scale 3D
        kernel would weld tabletops to ceilings (and cost minutes on a 20M-cell
        grid). 3D watertightness for display comes from extruding `bridged`
        over the level's z range.

        `barrier` is an optional (ny, nx) bool mask of cells the flood may not
        cross even though the lidar saw nothing there — e.g. the not-indoors
        region from ceiling evidence, which keeps a ground floor's flood from
        escaping through open entrance doors. Barrier cells seal the flood but
        are never reported as shell/bridged evidence.

        Returns None when there are no seeds. When even the largest kernel
        cannot seal (open-air scan), returns the best effort with the border
        leak component removed.
        """
        if seeds_xy is None or len(seeds_xy) == 0:
            return None
        res = self.voxel
        occ = self.slab(z_lo, z_hi, min_hits)
        ny, nx = occ.shape

        # 1-cell guaranteed-free border so "leaks to the border" is well defined
        pad = 1
        raw_u8 = np.zeros((ny + 2 * pad, nx + 2 * pad), np.uint8)
        raw_u8[pad:-pad, pad:-pad] = occ.astype(np.uint8) * 255
        occ_u8 = raw_u8
        barrier_full = np.zeros_like(raw_u8, dtype=bool)
        if barrier is not None:
            barrier_full[pad:-pad, pad:-pad] = barrier
            occ_u8 = cv2.bitwise_or(raw_u8, barrier_full.astype(np.uint8) * 255)

        # Seeds: robot trajectory cells, dilated by body clearance so a vertex
        # grazing a wall cell still lands in free space.
        seed = np.zeros_like(occ_u8)
        ci = ((seeds_xy[:, 0] - self.origin[0]) / res).astype(np.int32).clip(0, nx - 1) + pad
        ri = ((seeds_xy[:, 1] - self.origin[1]) / res).astype(np.int32).clip(0, ny - 1) + pad
        seed[ri, ci] = 255
        seed = cv2.dilate(seed, _ellipse(max(3, int(0.4 / res)) | 1))

        border = np.zeros_like(occ_u8, dtype=bool)
        border[0, :] = border[-1, :] = True
        border[:, 0] = border[:, -1] = True

        interior: np.ndarray | None = None
        closed = occ_u8
        closure_m = gap_steps[-1]
        sealed = False
        for g in gap_steps:
            # closing a gap g in a THIN wall needs a kernel ~1.5x g (measured):
            # the erosion half of MORPH_CLOSE eats marginal bridges back open
            px = max(3, int(round(1.5 * g / res))) | 1
            closed = cv2.morphologyEx(occ_u8, cv2.MORPH_CLOSE, _ellipse(px))
            free = (closed == 0).astype(np.uint8)
            n, labels = cv2.connectedComponents(free)
            seed_labels = set(np.unique(labels[(seed > 0) & (free > 0)])) - {0}
            if not seed_labels:
                continue  # seeds swallowed by this kernel — walls too close; try next
            cand = np.isin(labels, list(seed_labels))
            if not (cand & border).any():
                interior, closure_m, sealed = cand, g, True
                break
            interior = cand  # keep best effort

        if interior is None:
            return None
        if not sealed:
            # open-air best effort: drop whatever touches the border
            n, labels = cv2.connectedComponents(interior.astype(np.uint8))
            leak = set(np.unique(labels[border & interior])) - {0}
            interior = interior & ~np.isin(labels, list(leak))
            logger.warning(
                f"closed_region: could not seal within {gap_steps[-1]} m — "
                "returning border-clipped interior"
            )

        # Fill pinholes (sensor shadows under furniture) so loops stay simple.
        holes = (~interior) & (closed == 0)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(holes.astype(np.uint8))
        min_cells = max(1, int(0.15 / (res * res)))
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < min_cells and not (border & (labels == i)).any():
                interior |= labels == i

        interior_u8 = interior.astype(np.uint8) * 255
        touching = cv2.dilate(interior_u8, _ellipse(3)) > 0
        shell = (closed > 0) & touching & ~barrier_full  # barrier cells are synthetic
        bridged = shell & (raw_u8 == 0)

        # Contours -> closed world-space polygons (outer boundaries + holes).
        contours, _ = cv2.findContours(interior_u8, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        loops: list[np.ndarray] = []
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, epsilon=2.0, closed=True)  # 2 cells ~= 2*res m
            if len(approx) < 3:
                continue
            pts = approx.reshape(-1, 2).astype(np.float64)
            world = np.column_stack(
                [
                    self.origin[0] + (pts[:, 0] - pad + 0.5) * res,
                    self.origin[1] + (pts[:, 1] - pad + 0.5) * res,
                ]
            )
            loops.append(np.vstack([world, world[:1]]))  # explicitly closed

        unpad = (slice(pad, -pad), slice(pad, -pad))
        return ClosedRegion(
            interior=interior[unpad],
            shell=shell[unpad],
            bridged=bridged[unpad],
            loops=loops,
            origin=(self.origin[0], self.origin[1]),
            resolution=res,
            z_range=(z_lo, z_hi),
            closure_m=closure_m,
        )


class SceneModel:
    """A 3D model of the scanned space built from lidar + trajectory + imagery."""

    def __init__(
        self,
        points: np.ndarray,
        trajectory: np.ndarray | None = None,
        colors: np.ndarray | None = None,
    ) -> None:
        self.points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        self.trajectory = (
            np.asarray(trajectory, dtype=np.float32).reshape(-1, 3)
            if trajectory is not None
            else np.zeros((0, 3), dtype=np.float32)
        )
        if colors is not None:
            self.colors = np.asarray(colors, dtype=np.uint8).reshape(-1, 3)
            self.colored = np.ones(len(self.points), dtype=bool)
        else:
            self.colors = np.zeros((len(self.points), 3), dtype=np.uint8)
            self.colored = np.zeros(len(self.points), dtype=bool)

    @classmethod
    def from_rrd(
        cls,
        path: Path | str,
        clip_percentiles: tuple[float, float] = (0.5, 99.5),
    ) -> SceneModel:
        """Build from a Rerun recording, clipping z outliers."""
        points, trajectory = load_rrd_points(path)
        if len(points):
            z_lo, z_hi = np.percentile(points[:, 2], list(clip_percentiles))
            points = points[(points[:, 2] >= z_lo) & (points[:, 2] <= z_hi)]
        logger.info(
            f"SceneModel: {len(points)} points, {len(trajectory)} trajectory vertices "
            f"from {path}"
        )
        return cls(points, trajectory)

    # ------------------------------------------------------------------ #
    # Imagery
    # ------------------------------------------------------------------ #

    def colorize_from_session(
        self,
        session_dir: Path | str,
        max_frames: int = 120,
        max_range: float = 8.0,
        min_range: float = 0.4,
    ) -> float:
        """Project recorded camera frames onto the points (nearest-view wins).

        Expects `gtsam_odom.tum` (stamped base poses), `camera_intrinsics.json`
        and `mem2.db` (a `color_image` stream) in `session_dir`. Returns the
        fraction of points that received a color.
        """
        from dimos.memory2.store.sqlite import SqliteStore

        session = Path(session_dir).expanduser()
        poses = np.loadtxt(str(session / "gtsam_odom.tum"))
        intr: dict[str, Any] = json.loads((session / "camera_intrinsics.json").read_text())
        K = np.asarray(intr["intrinsics"], dtype=np.float64)
        res_w, res_h = intr["resolution"]
        opt = intr["optical_in_base"]  # [x y z qx qy qz qw]
        base_r_opt = quat_to_matrix(*opt[3:])
        base_t_opt = np.asarray(opt[:3], dtype=np.float64)

        store = SqliteStore(path=str(session / "mem2.db"), must_exist=True)
        store.start()
        stream = store.replay().stream("color_image")

        best_dist = np.full(len(self.points), np.inf, dtype=np.float32)
        pts64 = self.points.astype(np.float64)
        sample = np.unique(np.linspace(0, len(poses) - 1, max_frames).astype(int))
        used = 0
        for i in sample:
            t, x, y, z, qx, qy, qz, qw = poses[i]
            img = stream.find_closest(float(t), tolerance=0.4)
            if img is None:
                continue
            rgb = np.asarray(img.to_rgb().data)
            world_r_base = quat_to_matrix(qx, qy, qz, qw)
            world_r_opt = world_r_base @ base_r_opt
            world_t_opt = world_r_base @ base_t_opt + np.array([x, y, z])
            cam = (pts64 - world_t_opt) @ world_r_opt  # == R^T (p - t)
            depth = cam[:, 2]
            vis = (depth > min_range) & (depth < max_range)
            if not vis.any():
                continue
            u = (K[0, 0] * cam[vis, 0] / depth[vis] + K[0, 2]).astype(np.int32)
            v = (K[1, 1] * cam[vis, 1] / depth[vis] + K[1, 2]).astype(np.int32)
            in_img = (u >= 0) & (u < res_w) & (v >= 0) & (v < res_h)
            idx = np.nonzero(vis)[0][in_img]
            if not len(idx):
                continue
            closer = depth[idx].astype(np.float32) < best_dist[idx]
            idx = idx[closer]
            self.colors[idx] = rgb[v[in_img][closer], u[in_img][closer]]
            self.colored[idx] = True
            best_dist[idx] = depth[idx].astype(np.float32)
            used += 1
        store.stop()
        frac = float(self.colored.mean()) if len(self.points) else 0.0
        logger.info(f"Colorized {frac:.0%} of points from {used} camera frames")
        return frac

    # ------------------------------------------------------------------ #
    # Cross-sections
    # ------------------------------------------------------------------ #

    def _rasterize(
        self,
        u: np.ndarray,
        v: np.ndarray,
        sel: np.ndarray,
        resolution: float,
        axes: tuple[str, str],
        bounds: tuple[float, float, float, float] | None = None,
    ) -> Section:
        if bounds is None and not sel.any():
            return Section(np.zeros((1, 1), np.int32), None, (0, 0, 1, 1), axes)
        if bounds is not None:
            # fixed window: multiple sections share one grid (aligned cells)
            u_lo, v_lo, u_hi, v_hi = bounds
            sel = sel & (u >= u_lo) & (u <= u_hi) & (v >= v_lo) & (v <= v_hi)
        else:
            us_all, vs_all = u[sel], v[sel]
            u_lo, u_hi = float(us_all.min()), float(us_all.max())
            v_lo, v_hi = float(vs_all.min()), float(vs_all.max())
        us, vs = u[sel], v[sel]
        w = max(int(math.ceil((u_hi - u_lo) / resolution)) + 1, 1)
        h = max(int(math.ceil((v_hi - v_lo) / resolution)) + 1, 1)
        ci = ((us - u_lo) / resolution).astype(np.int32).clip(0, w - 1)
        ri = ((vs - v_lo) / resolution).astype(np.int32).clip(0, h - 1)
        density = np.zeros((h, w), dtype=np.int32)
        np.add.at(density, (ri, ci), 1)

        color: np.ndarray | None = None
        colored_sel = sel & self.colored
        if colored_sel.any():
            ucs, vcs = u[colored_sel], v[colored_sel]
            cci = ((ucs - u_lo) / resolution).astype(np.int32).clip(0, w - 1)
            cri = ((vcs - v_lo) / resolution).astype(np.int32).clip(0, h - 1)
            sums = np.zeros((h, w, 3), dtype=np.float64)
            counts = np.zeros((h, w), dtype=np.int64)
            np.add.at(sums, (cri, cci), self.colors[colored_sel].astype(np.float64))
            np.add.at(counts, (cri, cci), 1)
            color = np.zeros((h, w, 3), dtype=np.uint8)
            hit = counts > 0
            color[hit] = (sums[hit] / counts[hit, None]).astype(np.uint8)

        return Section(density, color, (u_lo, v_lo, u_hi, v_hi), axes)

    def horizontal_section(
        self,
        z: float,
        thickness: float = 0.2,
        resolution: float = 0.05,
        bounds: tuple[float, float, float, float] | None = None,
    ) -> Section:
        """Plan cut: all points with |pz - z| <= thickness/2, rasterized in xy.

        Pass `bounds` (x_lo, y_lo, x_hi, y_hi) to rasterize into a fixed
        window — sections requested with the same bounds and resolution are
        cell-aligned and can be compared/combined directly.
        """
        sel = np.abs(self.points[:, 2] - z) <= thickness / 2
        return self._rasterize(
            self.points[:, 0], self.points[:, 1], sel, resolution, ("x [m]", "y [m]"),
            bounds=bounds,
        )

    def vertical_section(
        self,
        p1: tuple[float, float],
        p2: tuple[float, float],
        thickness: float = 0.5,
        resolution: float = 0.05,
        z_range: tuple[float, float] | None = None,
    ) -> Section:
        """Section/elevation cut along the line p1→p2 (distance-along vs z)."""
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            raise ValueError("vertical section needs two distinct points")
        ux, uy = dx / length, dy / length
        rel_x = self.points[:, 0] - p1[0]
        rel_y = self.points[:, 1] - p1[1]
        along = rel_x * ux + rel_y * uy
        lateral = -rel_x * uy + rel_y * ux
        sel = (np.abs(lateral) <= thickness / 2) & (along >= 0) & (along <= length)
        if z_range is not None:
            sel &= (self.points[:, 2] >= z_range[0]) & (self.points[:, 2] <= z_range[1])
        return self._rasterize(
            along, self.points[:, 2], sel, resolution, ("along cut [m]", "z [m]")
        )

    # ------------------------------------------------------------------ #
    # 3D exports
    # ------------------------------------------------------------------ #

    def to_pointcloud(self, voxel: float | None = 0.05) -> o3d.geometry.PointCloud:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points.astype(np.float64))
        if self.colored.any():
            cols = self.colors.astype(np.float64) / 255.0
            # uncolored points get a height colormap so the model stays readable
            z = self.points[:, 2]
            t = (z - z.min()) / (z.max() - z.min() + 1e-9)
            fallback = np.column_stack([t, 0.4 + 0.2 * t, 1.0 - t])
            cols[~self.colored] = fallback[~self.colored]
            pcd.colors = o3d.utility.Vector3dVector(cols)
        if voxel:
            pcd = pcd.voxel_down_sample(voxel)
        return pcd

    def to_voxel_grid(self, voxel: float = 0.08) -> o3d.geometry.VoxelGrid:
        return o3d.geometry.VoxelGrid.create_from_point_cloud(self.to_pointcloud(None), voxel)

    def voxel_occupancy(
        self,
        voxel: float = 0.1,
        bounds: tuple[float, float, float, float, float, float] | None = None,
        max_cells: int = 400_000_000,
    ) -> VoxelOccupancy:
        """Bin the cloud into a dense (nz, ny, nx) hit-count grid.

        `bounds` is (x0, y0, z0, x1, y1, z1); defaults to the cloud extents
        plus a 0.3 m margin. Raises ValueError when the grid would exceed
        `max_cells` — usually outlier points; clip the cloud or coarsen `voxel`.
        """
        if not len(self.points):
            raise ValueError("voxel_occupancy: empty model")
        if bounds is None:
            lo = self.points.min(axis=0) - 0.3
            hi = self.points.max(axis=0) + 0.3
        else:
            lo = np.asarray(bounds[:3], dtype=np.float64)
            hi = np.asarray(bounds[3:], dtype=np.float64)
        dims = np.maximum(np.ceil((hi - lo) / voxel).astype(np.int64), 1)
        nx, ny, nz = int(dims[0]), int(dims[1]), int(dims[2])
        if nx * ny * nz > max_cells:
            raise ValueError(
                f"voxel_occupancy: {nx}x{ny}x{nz} = {nx * ny * nz:,} cells exceeds "
                f"max_cells={max_cells:,} — outlier points? Try a coarser voxel."
            )
        pts = self.points
        inside = np.all((pts >= lo) & (pts < hi), axis=1)
        idx = ((pts[inside] - lo) / voxel).astype(np.int64)
        ci = idx[:, 0].clip(0, nx - 1)
        ri = idx[:, 1].clip(0, ny - 1)
        ki = idx[:, 2].clip(0, nz - 1)
        flat = np.ravel_multi_index((ki, ri, ci), (nz, ny, nx))
        counts = np.bincount(flat, minlength=nz * ny * nx).reshape(nz, ny, nx)
        counts = counts.clip(0, 255).astype(np.uint8)
        logger.info(
            f"voxel_occupancy: {nx}x{ny}x{nz} grid at {voxel} m, "
            f"{int((counts > 0).sum()):,} occupied voxels"
        )
        return VoxelOccupancy(counts, (float(lo[0]), float(lo[1]), float(lo[2])), voxel)

    def to_mesh(self, voxel: float = 0.08, poisson_depth: int = 10) -> o3d.geometry.TriangleMesh:
        """Watertight-ish surface via Poisson reconstruction (slow on big clouds)."""
        pcd = self.to_pointcloud(voxel)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4, max_nn=30)
        )
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=poisson_depth
        )
        # trim the low-support blobs Poisson invents in empty space
        keep = np.asarray(densities) > np.quantile(np.asarray(densities), 0.05)
        mesh.remove_vertices_by_mask(~keep)
        return mesh

    def save_mesh(
        self,
        path: Path | str,
        voxel: float = 0.08,
        poisson_depth: int = 10,
        mesh: o3d.geometry.TriangleMesh | None = None,
    ) -> Path:
        """Export a surface mesh; format from the suffix (.glb/.gltf/.obj/.ply/.stl).

        GLB/glTF carries vertex colors (from `colorize_from_session` or the
        height fallback) and normals, so the model drops straight into standard
        3D tooling (Blender, three.js, CAD viewers). OBJ gets vertex colors as
        the common non-standard `v x y z r g b` extension. Pass `mesh` to reuse
        an already-built mesh instead of re-running Poisson.
        """
        path = Path(path)
        if mesh is None:
            mesh = self.to_mesh(voxel=voxel, poisson_depth=poisson_depth)
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        if path.suffix.lower() == ".stl" and mesh.has_vertex_colors():
            # STL cannot carry colors; drop them so the writer doesn't warn
            mesh = o3d.geometry.TriangleMesh(mesh)
            mesh.vertex_colors = o3d.utility.Vector3dVector()
        if not o3d.io.write_triangle_mesh(str(path), mesh):
            raise RuntimeError(f"failed to write mesh to {path}")
        logger.info(
            f"Wrote {path} ({len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles)"
        )
        return path

    def _display_points(self, max_points: int) -> tuple[np.ndarray, np.ndarray | None]:
        """Colored, size-capped point set shared by the rerun exporters."""
        pcd = self.to_pointcloud(None)
        pts = np.asarray(pcd.points, dtype=np.float32)
        cols = (np.asarray(pcd.colors) * 255).astype(np.uint8) if pcd.has_colors() else None
        if len(pts) > max_points:
            pick = np.random.default_rng(0).choice(len(pts), max_points, replace=False)
            pts = pts[pick]
            cols = cols[pick] if cols is not None else None
        return pts, cols

    def save_rerun(
        self, path: Path | str, point_radius: float = 0.02, max_points: int = 2_000_000
    ) -> None:
        """Write an interactive 3D model viewable with `dimos-viewer <path>`."""
        import rerun as rr

        pts, cols = self._display_points(max_points)

        rr.init("dimos_scene_model")
        rr.save(str(path))
        rr.log(
            "world/model",
            rr.Points3D(positions=pts, colors=cols, radii=point_radius),
            static=True,
        )
        if len(self.trajectory):
            rr.log(
                "world/trajectory",
                rr.LineStrips3D([self.trajectory], colors=[(255, 80, 60)]),
                static=True,
            )
        logger.info(f"Wrote {path} ({len(pts)} points) — view with: dimos-viewer {path}")

    def save_rerun_sliceable(
        self,
        path: Path | str,
        vox: VoxelOccupancy,
        regions: list[ClosedRegion] | None = None,
        z_step: float | None = None,
        min_hits: int = 2,
        point_radius: float = 0.02,
        max_points: int = 2_000_000,
    ) -> Path:
        """Write a sliceable 3D model: `dimos-viewer <path>`, time slider = cut height.

        The recording's only timeline is `cut_z` (meters, encoded as the
        duration axis), with one horizontal section per step — so scrubbing the
        viewer's time slider sweeps a cut plane through the property. Statics:
        the point model, the voxel model, the trajectory, each region's closed
        loops, and the `bridged` cells (gaps the loop closing inferred — glass,
        doorways, unscanned spans) extruded over their level's z range in orange.
        """
        import rerun as rr
        import rerun.blueprint as rrb

        path = Path(path)
        z_step = z_step or vox.voxel
        regions = regions or []

        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(origin="world", name="Model"),
                rrb.Spatial2DView(origin="section", name="Cross-section"),
            ),
            rrb.TimePanel(state="expanded"),
        )
        rr.init("dimos_scene_model_sliceable")
        rr.save(str(path), default_blueprint=blueprint)

        pts, cols = self._display_points(max_points)
        rr.log(
            "world/model",
            rr.Points3D(positions=pts, colors=cols, radii=point_radius),
            static=True,
        )
        if len(self.trajectory):
            rr.log(
                "world/trajectory",
                rr.LineStrips3D([self.trajectory], colors=[(255, 80, 60)]),
                static=True,
            )

        # Voxel model as centers with voxel-half radii (Boxes3D at ~1M instances
        # chokes the renderer; points with these radii read as voxels).
        occ = vox.occupancy(min_hits)
        kk, rr_i, cc = np.nonzero(occ)
        if len(kk):
            centers = np.column_stack(
                [
                    vox.origin[0] + (cc + 0.5) * vox.voxel,
                    vox.origin[1] + (rr_i + 0.5) * vox.voxel,
                    vox.origin[2] + (kk + 0.5) * vox.voxel,
                ]
            ).astype(np.float32)
            z = centers[:, 2]
            t = (z - z.min()) / (z.max() - z.min() + 1e-9)
            vox_cols = np.column_stack([40 + 180 * t, 80 + 120 * t, 220 - 160 * t]).astype(
                np.uint8
            )
            rr.log(
                "world/voxels",
                rr.Points3D(positions=centers, colors=vox_cols, radii=vox.voxel * 0.45),
                static=True,
            )
        else:
            # A sparse single-pass scan can leave no voxel above min_hits;
            # z.min() on the empty array would crash the save after the rest
            # of the pipeline already succeeded.
            logger.warning(f"No voxels with >= {min_hits} hits — skipping voxel layer")

        for i, region in enumerate(regions):
            if region.loops:
                z_loop = region.z_range[0] + 1.0
                strips = [
                    np.column_stack([loop, np.full(len(loop), z_loop)]) for loop in region.loops
                ]
                rr.log(
                    f"world/loops/L{i}",
                    rr.LineStrips3D(strips, colors=[(0, 220, 220)]),
                    static=True,
                )
            br_r, br_c = np.nonzero(region.bridged)
            if len(br_r):
                z_lo, z_hi = region.z_range
                n_layers = max(1, int((z_hi - z_lo) / vox.voxel))
                bx = region.origin[0] + (br_c + 0.5) * region.resolution
                by = region.origin[1] + (br_r + 0.5) * region.resolution
                col = np.repeat(np.column_stack([bx, by]), n_layers, axis=0)
                bz = np.tile(z_lo + (np.arange(n_layers) + 0.5) * vox.voxel, len(br_r))
                rr.log(
                    f"world/inferred/L{i}",
                    rr.Points3D(
                        positions=np.column_stack([col, bz]).astype(np.float32),
                        colors=(255, 150, 30),
                        radii=vox.voxel * 0.45,
                    ),
                    static=True,
                )

        # --- the cut_z timeline: one section per step, slider = cut height ---
        x_lo, y_lo, x_hi, y_hi = vox.xy_extent()
        cx, cy = (x_lo + x_hi) / 2, (y_lo + y_hi) / 2
        sx, sy = x_hi - x_lo, y_hi - y_lo
        nz = vox.shape[0]
        for k in range(nz):
            z_cut = vox.z_of(k)
            rr.set_time("cut_z", duration=float(z_cut))

            layer = vox.counts[k]
            img = (np.log1p(layer.astype(np.float32)) * 80).clip(0, 255).astype(np.uint8)
            rr.log("section/occupancy", rr.Image(img[::-1]))  # flip: image rows go down

            strips_2d = []
            for region in regions:
                if not (region.z_range[0] <= z_cut <= region.z_range[1]):
                    continue
                for loop in region.loops:
                    px = (loop[:, 0] - x_lo) / vox.voxel
                    py = (y_hi - loop[:, 1]) / vox.voxel  # match image row order
                    strips_2d.append(np.column_stack([px, py]).astype(np.float32))
            if strips_2d:
                rr.log("section/loops", rr.LineStrips2D(strips_2d, colors=[(0, 220, 220)]))

            rr.log(
                "world/cut",
                rr.Boxes3D(
                    centers=[(cx, cy, z_cut)],
                    half_sizes=[(sx / 2, sy / 2, vox.voxel / 2)],
                    colors=[(80, 180, 255, 60)],
                    fill_mode="solid",
                ),
            )
            rr_k, cc_k = np.nonzero(layer >= min_hits)
            if len(rr_k):
                slice_pts = np.column_stack(
                    [
                        vox.origin[0] + (cc_k + 0.5) * vox.voxel,
                        vox.origin[1] + (rr_k + 0.5) * vox.voxel,
                        np.full(len(rr_k), z_cut),
                    ]
                ).astype(np.float32)
                rr.log(
                    "world/cut_slice",
                    rr.Points3D(positions=slice_pts, colors=(255, 60, 60), radii=vox.voxel * 0.5),
                )
        logger.info(
            f"Wrote {path} ({nz} cut steps) — view with: dimos-viewer {path} "
            "(time slider sweeps the cut plane)"
        )
        return path
