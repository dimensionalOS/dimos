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

"""Multi-level surface path planner."""

from __future__ import annotations

import time
from typing import Any

from dimos_lcm.geometry_msgs import (
    Point as LCMPoint,
    Pose as LCMPose,
    PoseStamped as LCMPoseStamped,
    Quaternion as LCMQuaternion,
)
from dimos_lcm.nav_msgs import Path as LCMPath
from dimos_lcm.std_msgs import Header as LCMHeader, Time as LCMTime
import networkx as nx
import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path, sec_nsec
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SURFACE_DILATION_PASSES = 3
SURFACE_EROSION_PASSES = 3

NODE_SPACING_M = 2.0
NODE_WALL_BUFFER_M = 0.3
NODE_STEP_THRESHOLD_M = 0.25


class MLSPlannerConfig(ModuleConfig):
    world_frame: str = "map"
    voxel_size: float = 0.1
    robot_height: float = 1.5


def _extract_surfaces(points: np.ndarray, voxel_size: float, robot_height: float) -> np.ndarray:
    """For each XY column, mark cells with at least robot_height of free space above as surfaces."""
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    indices = np.floor(points / voxel_size).astype(np.int64)
    ix, iy, iz = indices[:, 0], indices[:, 1], indices[:, 2]

    order = np.lexsort((iz, iy, ix))
    sx, sy, sz = ix[order], iy[order], iz[order]

    height_cells = int(np.ceil(robot_height / voxel_size))

    next_same_col = np.zeros(len(sx), dtype=bool)
    next_same_col[:-1] = (sx[:-1] == sx[1:]) & (sy[:-1] == sy[1:])

    gap = np.empty(len(sx), dtype=np.int64)
    gap[:-1] = sz[1:] - sz[:-1]
    gap[-1] = 0

    is_surface = (~next_same_col) | (gap > height_cells)

    surf_ix = sx[is_surface]
    surf_iy = sy[is_surface]
    surf_iz = sz[is_surface]

    surf_ix, surf_iy, surf_iz = _close_surface_holes(
        surf_ix, surf_iy, surf_iz, SURFACE_DILATION_PASSES, SURFACE_EROSION_PASSES
    )

    x = (surf_ix.astype(np.float32) + 0.5) * voxel_size
    y = (surf_iy.astype(np.float32) + 0.5) * voxel_size
    z = (surf_iz.astype(np.float32) + 1.0) * voxel_size
    return np.column_stack([x, y, z]).astype(np.float32)


def _close_surface_holes(
    surf_ix: np.ndarray,
    surf_iy: np.ndarray,
    surf_iz: np.ndarray,
    dilation_passes: int,
    erosion_passes: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Do dilation then erosion on the surface map at each z level.

    Closes a lot of small holes that are artifacts missing lidar points.
    """
    if len(surf_ix) == 0 or (dilation_passes <= 0 and erosion_passes <= 0):
        return surf_ix, surf_iy, surf_iz

    pad = max(dilation_passes, 0)
    new_ix: list[np.ndarray] = []
    new_iy: list[np.ndarray] = []
    new_iz: list[np.ndarray] = []
    for level_iz in np.unique(surf_iz):
        sel = surf_iz == level_iz
        lx = surf_ix[sel]
        ly = surf_iy[sel]
        x0, x1 = int(lx.min()), int(lx.max())
        y0, y1 = int(ly.min()), int(ly.max())
        w = x1 - x0 + 1 + 2 * pad
        h = y1 - y0 + 1 + 2 * pad
        mask = np.zeros((h, w), dtype=bool)
        mask[ly - y0 + pad, lx - x0 + pad] = True
        if dilation_passes > 0:
            mask = ndimage.binary_dilation(mask, iterations=dilation_passes)
        if erosion_passes > 0:
            mask = ndimage.binary_erosion(mask, iterations=erosion_passes)
        ys, xs = np.where(mask)
        new_ix.append(xs.astype(np.int64) + x0 - pad)
        new_iy.append(ys.astype(np.int64) + y0 - pad)
        new_iz.append(np.full(len(xs), level_iz, dtype=np.int64))

    return (
        np.concatenate(new_ix),
        np.concatenate(new_iy),
        np.concatenate(new_iz),
    )


def _build_surface_lookup(
    sx: np.ndarray, sy: np.ndarray, sz: np.ndarray
) -> dict[tuple[int, int], np.ndarray]:
    """Group surface cells by XY column."""
    by_column: dict[tuple[int, int], list[int]] = {}
    for ix_, iy_, iz_ in zip(sx.tolist(), sy.tolist(), sz.tolist(), strict=True):
        by_column.setdefault((ix_, iy_), []).append(iz_)
    return {key: np.array(sorted(vs), dtype=np.int64) for key, vs in by_column.items()}


def _build_surface_adjacency(
    surface_lookup: dict[tuple[int, int], np.ndarray],
    voxel_size: float,
    step_threshold_cells: int,
) -> tuple[csr_matrix, dict[tuple[int, int, int], int], list[tuple[int, int, int]]]:
    """Sparse 8-connected adjacency over surface cells, with a per-step dz cap."""
    n = sum(len(zs) for zs in surface_lookup.values())
    if n == 0:
        return csr_matrix((0, 0), dtype=np.float64), {}, []

    ix = np.empty(n, dtype=np.int64)
    iy = np.empty(n, dtype=np.int64)
    iz = np.empty(n, dtype=np.int64)
    cursor = 0
    for (ix_col, iy_col), zs in surface_lookup.items():
        k = len(zs)
        ix[cursor : cursor + k] = int(ix_col)
        iy[cursor : cursor + k] = int(iy_col)
        iz[cursor : cursor + k] = zs
        cursor += k

    idx_to_cell: list[tuple[int, int, int]] = list(
        zip(ix.tolist(), iy.tolist(), iz.tolist(), strict=True)
    )
    cell_to_idx: dict[tuple[int, int, int], int] = {cell: i for i, cell in enumerate(idx_to_cell)}

    # Pack (ix, iy) into one int64 key. Padding leaves room for dx, dy in -1, 0, 1.
    ix_pos = ix - ix.min() + 1
    iy_pos = iy - iy.min() + 1
    y_range = int(iy_pos.max()) + 2
    col_key = ix_pos * y_range + iy_pos

    sort_order = np.lexsort((iz, col_key))
    sorted_col_key = col_key[sort_order]
    sorted_iz = iz[sort_order]

    row_chunks: list[np.ndarray] = []
    col_chunks: list[np.ndarray] = []
    data_chunks: list[np.ndarray] = []
    for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
        neighbor_key = (ix_pos + dx) * y_range + (iy_pos + dy)
        lo = np.searchsorted(sorted_col_key, neighbor_key, side="left")
        hi = np.searchsorted(sorted_col_key, neighbor_key, side="right")
        n_per_src = hi - lo
        total = int(n_per_src.sum())
        if total == 0:
            continue
        src_flat = np.repeat(np.arange(n), n_per_src)
        starts = np.zeros(n, dtype=np.int64)
        starts[1:] = np.cumsum(n_per_src[:-1])
        candidate_sorted_idx = lo[src_flat] + (np.arange(total) - starts[src_flat])
        dz = sorted_iz[candidate_sorted_idx] - iz[src_flat]
        valid = np.abs(dz) <= step_threshold_cells
        if not valid.any():
            continue
        src_valid = src_flat[valid]
        dst_valid = sort_order[candidate_sorted_idx[valid]]
        dz_valid = dz[valid]
        step_cost = np.sqrt(dx * dx + dy * dy + dz_valid * dz_valid) * voxel_size
        row_chunks.append(src_valid)
        col_chunks.append(dst_valid)
        data_chunks.append(step_cost.astype(np.float64))

    if not row_chunks:
        return csr_matrix((n, n), dtype=np.float64), cell_to_idx, idx_to_cell

    rows = np.concatenate(row_chunks)
    cols = np.concatenate(col_chunks)
    data = np.concatenate(data_chunks)
    return csr_matrix((data, (rows, cols)), shape=(n, n)), cell_to_idx, idx_to_cell


def _walk_predecessors(
    predecessors: np.ndarray, end_idx: int, idx_to_cell: list[tuple[int, int, int]]
) -> list[tuple[int, int, int]]:
    """Walk predecessors from end_idx back to its scipy Dijkstra source (neg predecessor)."""
    cells: list[tuple[int, int, int]] = [idx_to_cell[end_idx]]
    cur = end_idx
    while True:
        nxt = int(predecessors[cur])
        if nxt < 0:
            break
        cells.append(idx_to_cell[nxt])
        cur = nxt
    return cells


def place_nodes(
    surface_points: np.ndarray,
    voxel_size: float,
    *,
    node_spacing: float,
    wall_buffer: float,
    step_threshold: float,
) -> tuple[
    nx.Graph,
    csr_matrix,
    dict[tuple[int, int, int], int],
    list[tuple[int, int, int]],
    dict[tuple[int, int], np.ndarray],
]:
    """Place nodes by greedy NMS on the dist-to-wall field from one Dijkstra.

    Run multisource Dijkstra with the surface edges as sources to find distance from wall.
    Then place nodes spaced throughout based on that distance."""
    graph = nx.Graph()
    if len(surface_points) == 0:
        empty_adj = csr_matrix((0, 0), dtype=np.float64)
        return graph, empty_adj, {}, [], {}

    sx = np.floor(surface_points[:, 0] / voxel_size).astype(np.int64)
    sy = np.floor(surface_points[:, 1] / voxel_size).astype(np.int64)
    sz = np.floor(surface_points[:, 2] / voxel_size).astype(np.int64)
    surface_lookup = _build_surface_lookup(sx, sy, sz)

    step_cells = max(0, int(step_threshold / voxel_size))
    adj, cell_to_idx, idx_to_cell = _build_surface_adjacency(surface_lookup, voxel_size, step_cells)

    n_cells = adj.shape[0]
    if n_cells == 0:
        return graph, adj, cell_to_idx, idx_to_cell, surface_lookup

    neighbor_counts = np.diff(adj.indptr)
    boundary_indices = np.where(neighbor_counts < 8)[0]
    if len(boundary_indices) == 0:
        boundary_indices = np.array([0], dtype=np.int64)

    dist = dijkstra(adj, indices=boundary_indices, min_only=True)

    cells_arr = np.array(idx_to_cell, dtype=np.float64)
    cell_positions = cells_arr * voxel_size + np.array([0.5 * voxel_size, 0.5 * voxel_size, 0.0])

    candidate_mask = np.isfinite(dist) & (dist >= wall_buffer)
    candidate_indices = np.where(candidate_mask)[0]
    if len(candidate_indices) == 0:
        return graph, adj, cell_to_idx, idx_to_cell, surface_lookup
    order = candidate_indices[np.argsort(-dist[candidate_indices])]
    positions = cell_positions[order]

    # kill points in batches with kd tree
    tree = cKDTree(positions)
    killed = np.zeros(len(order), dtype=bool)

    for i in range(len(order)):
        if killed[i]:
            continue
        cix, ciy, ciz = idx_to_cell[int(order[i])]
        nid = graph.number_of_nodes()
        graph.add_node(
            nid,
            pos=(
                (cix + 0.5) * voxel_size,
                (ciy + 0.5) * voxel_size,
                ciz * voxel_size,
            ),
            cell=(cix, ciy, ciz),
        )
        nearby = tree.query_ball_point(positions[i], r=node_spacing)
        killed[np.asarray(nearby, dtype=np.int64)] = True

    return graph, adj, cell_to_idx, idx_to_cell, surface_lookup


def add_node_edges(
    graph: nx.Graph,
    adj: csr_matrix,
    cell_to_idx: dict[tuple[int, int, int], int],
    idx_to_cell: list[tuple[int, int, int]],
) -> tuple[np.ndarray, dict[int, int]]:
    """Add Voronoi-adjacency edges between placed nodes.

    One multi-source Dijkstra labels each cell with its nearest node. The cheapest
    boundary crossing per node-pair becomes an edge with the cell path on data["path"].
    Returns source_cells (per-cell owner) and cell_idx_to_nid for pose-snapping.
    """
    if graph.number_of_nodes() == 0:
        return np.full(adj.shape[0], -9999, dtype=np.int64), {}

    node_ids = list(graph.nodes())
    source_cell_indices = np.empty(len(node_ids), dtype=np.int64)
    cell_idx_to_nid: dict[int, int] = {}
    for nid in node_ids:
        cell_idx = cell_to_idx[graph.nodes[nid]["cell"]]
        source_cell_indices[nid] = cell_idx
        cell_idx_to_nid[cell_idx] = nid

    dist, predecessors, source_cells = dijkstra(
        adj,
        indices=source_cell_indices,
        min_only=True,
        return_predecessors=True,
    )

    rows = np.repeat(np.arange(adj.shape[0]), np.diff(adj.indptr))
    cols = adj.indices
    weights = adj.data
    src_u = source_cells[rows]
    src_v = source_cells[cols]
    boundary = (src_u != src_v) & (src_u >= 0) & (src_v >= 0)
    if not boundary.any():
        return source_cells, cell_idx_to_nid

    b_rows = rows[boundary]
    b_cols = cols[boundary]
    b_costs = dist[b_rows] + weights[boundary] + dist[b_cols]
    b_src_u = src_u[boundary]
    b_src_v = src_v[boundary]

    # Keep the min-cost boundary crossing per (node_a, node_b) pair.
    best: dict[tuple[int, int], tuple[float, int, int]] = {}
    for i in range(len(b_costs)):
        nid_a = cell_idx_to_nid[int(b_src_u[i])]
        nid_b = cell_idx_to_nid[int(b_src_v[i])]
        u_a = int(b_rows[i])
        u_b = int(b_cols[i])
        if nid_a > nid_b:
            nid_a, nid_b = nid_b, nid_a
            u_a, u_b = u_b, u_a
        cost = float(b_costs[i])
        existing = best.get((nid_a, nid_b))
        if existing is None or existing[0] > cost:
            best[(nid_a, nid_b)] = (cost, u_a, u_b)

    for (nid_a, nid_b), (cost, u_a, u_b) in best.items():
        path_a = _walk_predecessors(predecessors, u_a, idx_to_cell)
        path_b = _walk_predecessors(predecessors, u_b, idx_to_cell)
        path_a.reverse()
        full_path = np.array(path_a + path_b, dtype=np.int64)
        graph.add_edge(nid_a, nid_b, weight=cost, path=full_path)

    return source_cells, cell_idx_to_nid


class _PublishableLineSegments3D(LineSegments3D):
    """LineSegments3D with a Python lcm_encode. Upstream only implements decode."""

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMPath()
        sec, nsec = sec_nsec(self.ts)
        lcm_poses = []
        for (p1, p2), trav in zip(self._segments, self._traversability, strict=False):
            for pt in (p1, p2):
                lp = LCMPoseStamped()
                lp.pose = LCMPose()
                lp.pose.position = LCMPoint()
                lp.pose.orientation = LCMQuaternion()
                lp.pose.position.x = pt[0]
                lp.pose.position.y = pt[1]
                lp.pose.position.z = pt[2]
                lp.pose.orientation.w = trav
                lp.header = LCMHeader()
                lp.header.stamp = LCMTime()
                lp.header.stamp.sec = sec
                lp.header.stamp.nsec = nsec
                lp.header.frame_id = self.frame_id
                lcm_poses.append(lp)
        lcm_msg.poses_length = len(lcm_poses)
        lcm_msg.poses = lcm_poses
        lcm_msg.header.stamp.sec = sec
        lcm_msg.header.stamp.nsec = nsec
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]


def _nodes_to_cloud(graph: nx.Graph) -> np.ndarray:
    if graph.number_of_nodes() == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return np.array([graph.nodes[n]["pos"] for n in graph.nodes()], dtype=np.float32)


def _edges_to_segments(
    graph: nx.Graph, voxel_size: float
) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """Emit one segment per consecutive cell pair along each edge's cached path."""
    segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    for _, _, data in graph.edges(data=True):
        path_cells: np.ndarray = data["path"]
        for i in range(len(path_cells) - 1):
            a = path_cells[i]
            b = path_cells[i + 1]
            ax = (float(a[0]) + 0.5) * voxel_size
            ay = (float(a[1]) + 0.5) * voxel_size
            az = float(a[2]) * voxel_size
            bx = (float(b[0]) + 0.5) * voxel_size
            by = (float(b[1]) + 0.5) * voxel_size
            bz = float(b[2]) * voxel_size
            segments.append(((ax, ay, az), (bx, by, bz)))
    return segments


class MLSPlanner(Module):
    config: MLSPlannerConfig

    global_map: In[PointCloud2]
    start_pose: In[Odometry]
    goal_pose: In[Odometry]
    path: Out[Path]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_start: Odometry | None = None
        self._graph: nx.Graph | None = None
        self._cell_to_idx: dict[tuple[int, int, int], int] | None = None
        self._surface_lookup: dict[tuple[int, int], np.ndarray] | None = None
        self._source_cells: np.ndarray | None = None
        self._cell_idx_to_nid: dict[int, int] | None = None

    async def handle_global_map(self, msg: PointCloud2) -> None:
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            return

        t0 = time.perf_counter()
        surface_points = _extract_surfaces(points, self.config.voxel_size, self.config.robot_height)
        surfaces_ms = (time.perf_counter() - t0) * 1000
        self.surface_map.publish(
            PointCloud2.from_numpy(
                surface_points, frame_id=self.config.world_frame, timestamp=time.time()
            )
        )
        logger.info(
            "Surfaces ready",
            surfaces=len(surface_points),
            surface_ms=round(surfaces_ms, 1),
        )

        logger.info("Placing nodes", spacing_m=NODE_SPACING_M)
        t1 = time.perf_counter()
        graph, adj, cell_to_idx, idx_to_cell, surface_lookup = place_nodes(
            surface_points,
            self.config.voxel_size,
            node_spacing=NODE_SPACING_M,
            wall_buffer=NODE_WALL_BUFFER_M,
            step_threshold=NODE_STEP_THRESHOLD_M,
        )
        place_ms = (time.perf_counter() - t1) * 1000
        self.nodes.publish(
            PointCloud2.from_numpy(
                _nodes_to_cloud(graph),
                frame_id=self.config.world_frame,
                timestamp=time.time(),
            )
        )
        logger.info(
            "Nodes placed",
            nodes=graph.number_of_nodes(),
            place_ms=round(place_ms, 1),
        )

        logger.info("Building edges")
        t2 = time.perf_counter()
        source_cells, cell_idx_to_nid = add_node_edges(graph, adj, cell_to_idx, idx_to_cell)
        edges_ms = (time.perf_counter() - t2) * 1000
        logger.info(
            "Edges built",
            edges=graph.number_of_edges(),
            edges_ms=round(edges_ms, 1),
        )

        self._graph = graph
        self._cell_to_idx = cell_to_idx
        self._surface_lookup = surface_lookup
        self._source_cells = source_cells
        self._cell_idx_to_nid = cell_idx_to_nid

        self.node_edges.publish(
            _PublishableLineSegments3D(
                ts=time.time(),
                frame_id=self.config.world_frame,
                segments=_edges_to_segments(graph, self.config.voxel_size),
            )
        )

    async def handle_start_pose(self, msg: Odometry) -> None:
        self._latest_start = msg

    async def handle_goal_pose(self, msg: Odometry) -> None:
        if self._latest_start is None:
            logger.warning("MLSPlanner received goal before start; skipping")
            return
        if self._graph is None or self._graph.number_of_nodes() == 0:
            logger.warning("MLSPlanner received goal before graph was built; skipping")
            return

        t0 = time.perf_counter()
        start = (self._latest_start.x, self._latest_start.y, self._latest_start.z)
        goal = (msg.x, msg.y, msg.z)

        start_node = self._snap_pose_to_node(start)
        goal_node = self._snap_pose_to_node(goal)
        if start_node is None or goal_node is None:
            logger.warning(
                "Could not snap pose to graph",
                start=start,
                goal=goal,
                start_node=start_node,
                goal_node=goal_node,
            )
            return
        assert self._graph is not None
        logger.info(
            "Snapped poses to graph nodes",
            start_pose=start,
            start_node=start_node,
            start_node_pos=self._graph.nodes[start_node]["pos"],
            goal_pose=goal,
            goal_node=goal_node,
            goal_node_pos=self._graph.nodes[goal_node]["pos"],
        )

        try:
            node_seq = nx.shortest_path(
                self._graph, source=start_node, target=goal_node, weight="weight"
            )
        except nx.NetworkXNoPath:
            logger.warning(
                "No path between start and goal nodes",
                start_node=start_node,
                goal_node=goal_node,
            )
            return

        waypoints = self._assemble_waypoints(node_seq, start, goal)
        plan_ms = (time.perf_counter() - t0) * 1000

        now = time.time()
        path_msg = Path(
            ts=now,
            frame_id=self.config.world_frame,
            poses=[
                PoseStamped(
                    ts=now,
                    frame_id=self.config.world_frame,
                    position=[float(x), float(y), float(z)],
                    orientation=[0.0, 0.0, 0.0, 1.0],
                )
                for x, y, z in waypoints
            ],
        )
        self.path.publish(path_msg)
        logger.info(
            "Path planned",
            waypoints=len(waypoints),
            nodes_traversed=len(node_seq),
            plan_ms=round(plan_ms, 1),
        )

    def _snap_pose_to_node(self, pose_xyz: tuple[float, float, float]) -> int | None:
        """Snap pose to its owning node via the precomputed Voronoi labels.

        Finds the surface cell in the pose's column, looks up its owner in source_cells.
        Falls back to nearby columns if the pose's own column has no surface.
        """
        if (
            self._graph is None
            or self._cell_to_idx is None
            or self._surface_lookup is None
            or self._source_cells is None
            or self._cell_idx_to_nid is None
        ):
            return None
        if self._graph.number_of_nodes() == 0:
            return None

        voxel = self.config.voxel_size
        x, y, z = pose_xyz
        ix = int(np.floor(x / voxel))
        iy = int(np.floor(y / voxel))
        target_iz = int(np.floor(z / voxel)) - 1
        tolerance_cells = int(np.ceil(self.config.robot_height / voxel))

        cell = self._best_iz_in_column(ix, iy, target_iz, tolerance_cells)
        if cell is None:
            # Pose's column has no surface. Try nearby columns.
            search_radius = 5
            best_cell = None
            best_d2 = -1
            for dix in range(-search_radius, search_radius + 1):
                for diy in range(-search_radius, search_radius + 1):
                    if dix == 0 and diy == 0:
                        continue
                    c = self._best_iz_in_column(ix + dix, iy + diy, target_iz, tolerance_cells)
                    if c is None:
                        continue
                    d2 = dix * dix + diy * diy
                    if best_d2 < 0 or d2 < best_d2:
                        best_d2 = d2
                        best_cell = c
            cell = best_cell
            if cell is None:
                return None

        cell_idx = self._cell_to_idx.get(cell)
        if cell_idx is None:
            return None
        owner_cell_idx = int(self._source_cells[cell_idx])
        if owner_cell_idx < 0:
            return None
        return self._cell_idx_to_nid.get(owner_cell_idx)

    def _best_iz_in_column(
        self, ix: int, iy: int, target_iz: int, tolerance_cells: int
    ) -> tuple[int, int, int] | None:
        """Surface iz in column (ix, iy) closest to target_iz, within tolerance."""
        if self._surface_lookup is None:
            return None
        zs = self._surface_lookup.get((ix, iy))
        if zs is None or len(zs) == 0:
            return None
        distances = np.abs(zs - target_iz)
        best = int(distances.argmin())
        if int(distances[best]) > tolerance_cells:
            return None
        return (ix, iy, int(zs[best]))

    def _assemble_waypoints(
        self,
        node_seq: list[int],
        start_pose: tuple[float, float, float],
        goal_pose: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        """Chain cached edge paths into a continuous waypoint list, bracketed by the actual poses."""
        assert self._graph is not None
        voxel = self.config.voxel_size
        cells: list[tuple[int, int, int]] = []
        for i in range(len(node_seq) - 1):
            a, b = node_seq[i], node_seq[i + 1]
            edge_path: np.ndarray = self._graph[a][b]["path"]
            # Stored path runs from min(a, b) to max(a, b). Reverse if traversing the other way.
            if a > b:
                edge_path = edge_path[::-1]
            tail = edge_path if i == 0 else edge_path[1:]
            for c in tail:
                cells.append((int(c[0]), int(c[1]), int(c[2])))

        waypoints: list[tuple[float, float, float]] = [start_pose]
        for cix, ciy, ciz in cells:
            waypoints.append(((cix + 0.5) * voxel, (ciy + 0.5) * voxel, (ciz + 1.0) * voxel))
        waypoints.append(goal_pose)
        return waypoints
