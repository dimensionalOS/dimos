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

from dataclasses import dataclass, field
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
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path, sec_nsec
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SURFACE_DILATION_PASSES = 3
SURFACE_EROSION_PASSES = 3

NODE_SPACING_M = 1.0
NODE_WALL_BUFFER_M = 0.3
NODE_STEP_THRESHOLD_M = 0.25


class MLSPlannerConfig(ModuleConfig):
    world_frame: str = "map"
    voxel_size: float = 0.1
    robot_height: float = 1.5


def surface_point_xyz(ix: int, iy: int, iz: int, voxel_size: float) -> tuple[float, float, float]:
    """Standing-surface coord for one cell. XY centered in the cell, Z at the cell's top face."""
    return ((ix + 0.5) * voxel_size, (iy + 0.5) * voxel_size, (iz + 1.0) * voxel_size)


def surface_points_xyz(cells: np.ndarray, voxel_size: float) -> np.ndarray:
    """Vectorized surface_point_xyz. (N, 3) int cell indices to (N, 3) world XYZ."""
    offset = np.array([0.5 * voxel_size, 0.5 * voxel_size, voxel_size])
    return cells * voxel_size + offset


@dataclass
class SurfaceGraph:
    """Surface-cell grid plus its waypoint-node overlay.

    place_nodes populates the first five fields. add_node_edges fills source_cells
    and cell_idx_to_nid and adds edges to graph.
    """

    graph: nx.Graph
    adj: csr_matrix
    cell_to_idx: dict[tuple[int, int, int], int]
    idx_to_cell: list[tuple[int, int, int]]
    surface_lookup: dict[tuple[int, int], np.ndarray]
    source_cells: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.int64))
    cell_idx_to_nid: dict[int, int] = field(default_factory=dict)


def _extract_surfaces(points: np.ndarray, voxel_size: float, robot_height: float) -> np.ndarray:
    """For each XY column, mark cells with at least robot_height of free space above as surfaces."""
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    indices = np.floor(points / voxel_size).astype(np.int64)
    ix, iy, iz = indices[:, 0], indices[:, 1], indices[:, 2]

    order = np.lexsort((iz, iy, ix))
    ix_sorted, iy_sorted, iz_sorted = ix[order], iy[order], iz[order]

    height_cells = int(np.ceil(robot_height / voxel_size))

    next_same_col = np.zeros(len(ix_sorted), dtype=bool)
    next_same_col[:-1] = (ix_sorted[:-1] == ix_sorted[1:]) & (iy_sorted[:-1] == iy_sorted[1:])

    gap = np.empty(len(ix_sorted), dtype=np.int64)
    gap[:-1] = iz_sorted[1:] - iz_sorted[:-1]
    gap[-1] = 0

    is_surface = (~next_same_col) | (gap > height_cells)

    surface_ix = ix_sorted[is_surface]
    surface_iy = iy_sorted[is_surface]
    surface_iz = iz_sorted[is_surface]

    surface_ix, surface_iy, surface_iz = _close_surface_holes(
        surface_ix,
        surface_iy,
        surface_iz,
        SURFACE_DILATION_PASSES,
        SURFACE_EROSION_PASSES,
        ix,
        iy,
        iz,
        height_cells,
    )

    cells = np.column_stack([surface_ix, surface_iy, surface_iz])
    return surface_points_xyz(cells, voxel_size).astype(np.float32)


def _close_surface_holes(
    surface_ix: np.ndarray,
    surface_iy: np.ndarray,
    surface_iz: np.ndarray,
    dilation_passes: int,
    erosion_passes: int,
    obstacle_ix: np.ndarray,
    obstacle_iy: np.ndarray,
    obstacle_iz: np.ndarray,
    height_cells: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Dilate then erode the surface map at each z level, without bridging walls.

    Fills small holes from missing lidar points, then rejects dilated cells whose
    column has an obstacle point at z' in (level_iz, level_iz + height_cells].
    That's the same clearance check the surface extractor uses, applied to the
    dilated cells so morphology can't bridge across a wall column.
    """
    if len(surface_ix) == 0 or (dilation_passes <= 0 and erosion_passes <= 0):
        return surface_ix, surface_iy, surface_iz

    pad = max(dilation_passes, 0)
    new_ix: list[np.ndarray] = []
    new_iy: list[np.ndarray] = []
    new_iz: list[np.ndarray] = []
    for level_iz in np.unique(surface_iz):
        sel = surface_iz == level_iz
        lx = surface_ix[sel]
        ly = surface_iy[sel]
        x0, x1 = int(lx.min()), int(lx.max())
        y0, y1 = int(ly.min()), int(ly.max())
        w = x1 - x0 + 1 + 2 * pad
        h = y1 - y0 + 1 + 2 * pad
        # numpy mask is indexed (row, col) = (y, x).
        mask = np.zeros((h, w), dtype=bool)
        mask[ly - y0 + pad, lx - x0 + pad] = True
        if dilation_passes > 0:
            mask = ndimage.binary_dilation(mask, iterations=dilation_passes)
        if erosion_passes > 0:
            mask = ndimage.binary_erosion(mask, iterations=erosion_passes)

        blocking = (
            (obstacle_iz > level_iz)
            & (obstacle_iz <= level_iz + height_cells)
            & (obstacle_ix >= x0 - pad)
            & (obstacle_ix <= x1 + pad)
            & (obstacle_iy >= y0 - pad)
            & (obstacle_iy <= y1 + pad)
        )
        if blocking.any():
            blocked = np.zeros((h, w), dtype=bool)
            blocked[
                obstacle_iy[blocking] - y0 + pad,
                obstacle_ix[blocking] - x0 + pad,
            ] = True
            mask = mask & ~blocked

        ys, xs = np.where(mask)
        new_ix.append(xs.astype(np.int64) + x0 - pad)
        new_iy.append(ys.astype(np.int64) + y0 - pad)
        new_iz.append(np.full(len(xs), level_iz, dtype=np.int64))

    return (
        np.concatenate(new_ix),
        np.concatenate(new_iy),
        np.concatenate(new_iz),
    )


def build_surface_lookup(
    ix: np.ndarray, iy: np.ndarray, iz: np.ndarray
) -> dict[tuple[int, int], np.ndarray]:
    """Group surface cells by XY column, deduping z values per column.

    Public so downstream code can recompute the same lookup the planner uses.
    """
    by_column: dict[tuple[int, int], list[int]] = {}
    for ix_, iy_, iz_ in zip(ix.tolist(), iy.tolist(), iz.tolist(), strict=True):
        by_column.setdefault((ix_, iy_), []).append(iz_)
    return {key: np.unique(np.array(vs, dtype=np.int64)) for key, vs in by_column.items()}


def build_surface_adjacency(
    surface_lookup: dict[tuple[int, int], np.ndarray],
    voxel_size: float,
    step_threshold_cells: int,
) -> tuple[csr_matrix, dict[tuple[int, int, int], int], list[tuple[int, int, int]]]:
    """Sparse 8-connected adjacency over surface cells, with a per-step dz cap.

    Public so downstream code can recompute the same adjacency the planner uses.
    """
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

    # Pack (ix, iy) into one int64 key. Offsets leave room for dx, dy in -1, 0, 1.
    ix_offset = ix - ix.min() + 1
    iy_offset = iy - iy.min() + 1
    y_stride = int(iy_offset.max()) + 2
    col_key = ix_offset * y_stride + iy_offset

    sort_order = np.lexsort((iz, col_key))
    sorted_col_key = col_key[sort_order]
    sorted_iz = iz[sort_order]

    row_chunks: list[np.ndarray] = []
    col_chunks: list[np.ndarray] = []
    data_chunks: list[np.ndarray] = []
    for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
        # For each source cell, range-scan the (dx, dy) neighbor column for its surface cells,
        # then keep only the dz transitions that are within the per-step limit.
        neighbor_key = (ix_offset + dx) * y_stride + (iy_offset + dy)
        lo = np.searchsorted(sorted_col_key, neighbor_key, side="left")
        hi = np.searchsorted(sorted_col_key, neighbor_key, side="right")
        n_per_src = hi - lo
        total = int(n_per_src.sum())
        if total == 0:
            continue
        src_per_candidate = np.repeat(np.arange(n), n_per_src)
        starts = np.zeros(n, dtype=np.int64)
        starts[1:] = np.cumsum(n_per_src[:-1])
        candidate_in_sorted = lo[src_per_candidate] + (np.arange(total) - starts[src_per_candidate])
        dz = sorted_iz[candidate_in_sorted] - iz[src_per_candidate]
        valid = np.abs(dz) <= step_threshold_cells
        if not valid.any():
            continue
        src_valid = src_per_candidate[valid]
        dst_valid = sort_order[candidate_in_sorted[valid]]
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


def _wall_safe_adjacency(
    adj: csr_matrix,
    dist_to_wall: np.ndarray,
    missing_neighbors: np.ndarray,
    buffer_m: float,
    voxel_size: float,
) -> csr_matrix:
    """Scale adjacency edge costs up for cells near walls or with high edge exposure.

    Two multiplicative penalties combine per cell:
    - Distance penalty (buffer_m / dist)^4 clamped to >= 1, with a small dist
      floor so cells exactly on an edge stay distinguishable from cells one
      voxel in.
    - Exposure penalty 1 + missing/2 from the count of missing same-z neighbors,
      gated by proximity to a wall. Far from walls, a missing neighbor is almost
      always just a small hole in the surface map (mesh-sampling noise), not a
      real wall edge -- so the exposure factor fades to 1 as dist exceeds buffer.
      Near walls, the full corner-avoidance behavior kicks in.

    Per-edge cost averages the two endpoints' penalties.
    """
    safe_dist = np.maximum(dist_to_wall, voxel_size / 10.0)
    dist_penalty = np.maximum(1.0, (buffer_m / safe_dist) ** 4)
    near_wall = np.clip(1.0 - dist_to_wall / buffer_m, 0.0, 1.0)
    exposure_penalty = 1.0 + (missing_neighbors / 2.0) * near_wall
    penalty = dist_penalty * exposure_penalty
    src = np.repeat(np.arange(adj.shape[0]), np.diff(adj.indptr))
    dst = adj.indices
    scaled = adj.data * (penalty[src] + penalty[dst]) / 2.0
    return csr_matrix((scaled, adj.indices.copy(), adj.indptr.copy()), shape=adj.shape)


def place_nodes(
    surface_points: np.ndarray,
    voxel_size: float,
    *,
    node_spacing: float,
    wall_buffer: float,
    step_threshold: float,
) -> SurfaceGraph:
    """Place nodes by greedy NMS on the dist-to-wall field from one Dijkstra.

    Run multisource Dijkstra with the surface edges as sources to find distance from wall.
    Then place nodes spaced throughout based on that distance. The returned adjacency
    is wall-safe: edge costs scale up within wall_buffer of any wall so add_node_edges
    routes through corridor centers.
    """
    graph = nx.Graph()
    if len(surface_points) == 0:
        return SurfaceGraph(
            graph=graph,
            adj=csr_matrix((0, 0), dtype=np.float64),
            cell_to_idx={},
            idx_to_cell=[],
            surface_lookup={},
        )

    sx = np.floor(surface_points[:, 0] / voxel_size).astype(np.int64)
    sy = np.floor(surface_points[:, 1] / voxel_size).astype(np.int64)
    sz = np.floor(surface_points[:, 2] / voxel_size).astype(np.int64)
    surface_lookup = build_surface_lookup(sx, sy, sz)

    step_cells = max(0, int(step_threshold / voxel_size))
    adj, cell_to_idx, idx_to_cell = build_surface_adjacency(surface_lookup, voxel_size, step_cells)

    n_cells = adj.shape[0]
    if n_cells == 0:
        return SurfaceGraph(
            graph=graph,
            adj=adj,
            cell_to_idx=cell_to_idx,
            idx_to_cell=idx_to_cell,
            surface_lookup=surface_lookup,
        )

    # Detect walls and cliffs using a same-z-only adjacency. The 3D adj would miss
    # cliff edges by counting cross-z neighbors, letting a landing corner that
    # overlooks a stair below appear "interior".
    adj_xy, _, _ = build_surface_adjacency(surface_lookup, voxel_size, 0)
    xy_neighbor_count = np.diff(adj_xy.indptr)
    wall_adjacent_indices = np.where(xy_neighbor_count < 8)[0]
    if len(wall_adjacent_indices) == 0:
        wall_adjacent_indices = np.array([0], dtype=np.int64)

    dist = dijkstra(adj_xy, indices=wall_adjacent_indices, min_only=True)

    cells_arr = np.array(idx_to_cell, dtype=np.float64)
    cell_positions = surface_points_xyz(cells_arr, voxel_size)

    candidate_mask = np.isfinite(dist) & (dist >= wall_buffer)
    candidate_indices = np.where(candidate_mask)[0]
    if len(candidate_indices) == 0:
        return SurfaceGraph(
            graph=graph,
            adj=adj,
            cell_to_idx=cell_to_idx,
            idx_to_cell=idx_to_cell,
            surface_lookup=surface_lookup,
        )
    order = candidate_indices[np.argsort(-dist[candidate_indices])]
    positions = cell_positions[order]

    tree = cKDTree(positions)
    killed = np.zeros(len(order), dtype=bool)

    for i in range(len(order)):
        if killed[i]:
            continue
        cix, ciy, ciz = idx_to_cell[int(order[i])]
        nid = graph.number_of_nodes()
        graph.add_node(
            nid,
            pos=surface_point_xyz(cix, ciy, ciz, voxel_size),
            cell=(cix, ciy, ciz),
        )
        nearby = tree.query_ball_point(positions[i], r=node_spacing)
        killed[np.asarray(nearby, dtype=np.int64)] = True

    missing_neighbors = 8 - xy_neighbor_count
    adj_safe = _wall_safe_adjacency(adj, dist, missing_neighbors, wall_buffer, voxel_size)
    return SurfaceGraph(
        graph=graph,
        adj=adj_safe,
        cell_to_idx=cell_to_idx,
        idx_to_cell=idx_to_cell,
        surface_lookup=surface_lookup,
    )


def add_node_edges(sg: SurfaceGraph) -> None:
    """Add Voronoi-adjacency edges between placed nodes, mutating sg in place.

    One multi-source Dijkstra labels each cell with its nearest node. The cheapest
    boundary crossing per node-pair becomes an edge with the cell path on data["path"].
    Also fills sg.source_cells (per-cell owner) and sg.cell_idx_to_nid for pose-snapping.
    """
    if sg.graph.number_of_nodes() == 0:
        sg.source_cells = np.full(sg.adj.shape[0], -9999, dtype=np.int64)
        sg.cell_idx_to_nid = {}
        return

    node_ids = list(sg.graph.nodes())
    source_cell_indices = np.empty(len(node_ids), dtype=np.int64)
    cell_idx_to_nid: dict[int, int] = {}
    for nid in node_ids:
        cell_idx = sg.cell_to_idx[sg.graph.nodes[nid]["cell"]]
        source_cell_indices[nid] = cell_idx
        cell_idx_to_nid[cell_idx] = nid

    dist, predecessors, source_cells = dijkstra(
        sg.adj,
        indices=source_cell_indices,
        min_only=True,
        return_predecessors=True,
    )

    rows = np.repeat(np.arange(sg.adj.shape[0]), np.diff(sg.adj.indptr))
    cols = sg.adj.indices
    weights = sg.adj.data
    src_u = source_cells[rows]
    src_v = source_cells[cols]
    boundary = (src_u != src_v) & (src_u >= 0) & (src_v >= 0)
    sg.source_cells = source_cells
    sg.cell_idx_to_nid = cell_idx_to_nid
    if not boundary.any():
        return

    b_rows = rows[boundary]
    b_cols = cols[boundary]
    b_costs = dist[b_rows] + weights[boundary] + dist[b_cols]
    b_src_u = src_u[boundary]
    b_src_v = src_v[boundary]

    # Keep the min-cost boundary crossing per node pair.
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
        path_a = _walk_predecessors(predecessors, u_a, sg.idx_to_cell)
        path_b = _walk_predecessors(predecessors, u_b, sg.idx_to_cell)
        path_a.reverse()
        full_path = np.array(path_a + path_b, dtype=np.int64)
        sg.graph.add_edge(nid_a, nid_b, weight=cost, path=full_path)


class _LineSegmentsAsPath(LineSegments3D):
    """Pack LineSegments3D into nav_msgs/Path until a dedicated message exists.

    Segment endpoints alternate as poses (p1, p2, p1, p2, ...) and traversability rides
    on each pose's orientation.w.
    """

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
    sg: SurfaceGraph, voxel_size: float
) -> tuple[
    list[tuple[tuple[float, float, float], tuple[float, float, float]]],
    list[float],
]:
    """Emit one segment per consecutive cell pair on each edge.

    All segments belonging to the same graph edge share the edge's total weight
    so the renderer can color the whole edge uniformly.
    """
    segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    weights: list[float] = []
    for _, _, edge_data in sg.graph.edges(data=True):
        path_cells: np.ndarray = edge_data["path"]
        edge_weight = float(edge_data.get("weight", 0.0))
        for i in range(len(path_cells) - 1):
            a = path_cells[i]
            b = path_cells[i + 1]
            segments.append(
                (
                    surface_point_xyz(int(a[0]), int(a[1]), int(a[2]), voxel_size),
                    surface_point_xyz(int(b[0]), int(b[1]), int(b[2]), voxel_size),
                )
            )
            weights.append(edge_weight)
    return segments, weights


class MLSPlanner(Module):
    config: MLSPlannerConfig

    global_map: In[PointCloud2]
    start_pose: In[Odometry]
    goal_pose: In[Odometry]
    clicked_point: In[PointStamped]
    path: Out[Path]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_start: tuple[float, float, float] | None = None
        self._surface_graph: SurfaceGraph | None = None
        # Clicks alternate between setting the start and setting the goal+planning.
        self._next_click_sets_start: bool = True

    async def handle_global_map(self, msg: PointCloud2) -> None:
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            return

        # 1. Surface extraction
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

        # 2. Node placement
        logger.info("Placing nodes", spacing_m=NODE_SPACING_M)
        t1 = time.perf_counter()
        sg = place_nodes(
            surface_points,
            self.config.voxel_size,
            node_spacing=NODE_SPACING_M,
            wall_buffer=NODE_WALL_BUFFER_M,
            step_threshold=NODE_STEP_THRESHOLD_M,
        )
        place_ms = (time.perf_counter() - t1) * 1000
        self.nodes.publish(
            PointCloud2.from_numpy(
                _nodes_to_cloud(sg.graph),
                frame_id=self.config.world_frame,
                timestamp=time.time(),
            )
        )
        logger.info(
            "Nodes placed",
            nodes=sg.graph.number_of_nodes(),
            place_ms=round(place_ms, 1),
        )

        # 3. Edge construction
        logger.info("Building edges")
        t2 = time.perf_counter()
        add_node_edges(sg)
        edges_ms = (time.perf_counter() - t2) * 1000
        logger.info(
            "Edges built",
            edges=sg.graph.number_of_edges(),
            edges_ms=round(edges_ms, 1),
        )

        self._surface_graph = sg
        segments, segment_weights = _edges_to_segments(sg, self.config.voxel_size)
        self.node_edges.publish(
            _LineSegmentsAsPath(
                ts=time.time(),
                frame_id=self.config.world_frame,
                segments=segments,
                traversability=segment_weights,
            )
        )

    async def handle_start_pose(self, msg: Odometry) -> None:
        self._latest_start = (msg.x, msg.y, msg.z)

    def _publish_empty_path(self) -> None:
        """Clear any previously published path so the visualizer drops the stale plan."""
        self.path.publish(Path(ts=time.time(), frame_id=self.config.world_frame, poses=[]))

    async def handle_goal_pose(self, msg: Odometry) -> None:
        self._plan_to((msg.x, msg.y, msg.z))

    async def handle_clicked_point(self, msg: PointStamped) -> None:
        pt = (msg.x, msg.y, msg.z)
        if self._next_click_sets_start:
            self._latest_start = pt
            self._next_click_sets_start = False
            self._publish_empty_path()
            logger.info("Click set start; next click will set goal", start=pt)
            return
        self._next_click_sets_start = True
        logger.info("Click set goal", goal=pt)
        self._plan_to(pt)

    def _plan_to(self, goal: tuple[float, float, float]) -> None:
        if self._latest_start is None:
            logger.warning("MLSPlanner received goal before start; skipping")
            return
        sg = self._surface_graph
        if sg is None or sg.graph.number_of_nodes() == 0:
            logger.warning("MLSPlanner received goal before graph was built; skipping")
            return

        t0 = time.perf_counter()
        start = self._latest_start

        start_node = self._snap_pose_to_node(sg, start)
        goal_node = self._snap_pose_to_node(sg, goal)
        if start_node is None or goal_node is None:
            logger.warning(
                "Could not snap pose to graph",
                start=start,
                goal=goal,
                start_node=start_node,
                goal_node=goal_node,
            )
            self._publish_empty_path()
            return
        logger.info(
            "Snapped poses to graph nodes",
            start_pose=start,
            start_node=start_node,
            start_node_pos=sg.graph.nodes[start_node]["pos"],
            goal_pose=goal,
            goal_node=goal_node,
            goal_node_pos=sg.graph.nodes[goal_node]["pos"],
        )

        try:
            node_seq = nx.shortest_path(
                sg.graph, source=start_node, target=goal_node, weight="weight"
            )
        except nx.NetworkXNoPath:
            logger.warning(
                "No path between start and goal nodes",
                start_node=start_node,
                goal_node=goal_node,
            )
            self._publish_empty_path()
            return

        waypoints = self._assemble_waypoints(sg, node_seq, start, goal)
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

    def _snap_pose_to_node(
        self, sg: SurfaceGraph, pose_xyz: tuple[float, float, float]
    ) -> int | None:
        """Snap pose to its owning node via the precomputed Voronoi labels.

        Finds the surface cell in the pose's column, looks up its owner in source_cells.
        Falls back to nearby columns if the pose's own column has no surface.
        """
        if sg.graph.number_of_nodes() == 0:
            return None

        voxel = self.config.voxel_size
        x, y, z = pose_xyz
        ix = int(np.floor(x / voxel))
        iy = int(np.floor(y / voxel))
        target_iz = int(np.floor(z / voxel)) - 1
        tolerance_cells = int(np.ceil(self.config.robot_height / voxel))

        cell = self._best_iz_in_column(sg, ix, iy, target_iz, tolerance_cells)
        if cell is None:
            # Pose's column has no surface. Try nearby columns.
            search_radius = 5
            best_cell = None
            best_d2 = -1
            for dix in range(-search_radius, search_radius + 1):
                for diy in range(-search_radius, search_radius + 1):
                    if dix == 0 and diy == 0:
                        continue
                    c = self._best_iz_in_column(sg, ix + dix, iy + diy, target_iz, tolerance_cells)
                    if c is None:
                        continue
                    d2 = dix * dix + diy * diy
                    if best_d2 < 0 or d2 < best_d2:
                        best_d2 = d2
                        best_cell = c
            cell = best_cell
            if cell is None:
                return None

        cell_idx = sg.cell_to_idx.get(cell)
        if cell_idx is None:
            return None
        owner_cell_idx = int(sg.source_cells[cell_idx])
        if owner_cell_idx < 0:
            return None
        return sg.cell_idx_to_nid.get(owner_cell_idx)

    def _best_iz_in_column(
        self, sg: SurfaceGraph, ix: int, iy: int, target_iz: int, tolerance_cells: int
    ) -> tuple[int, int, int] | None:
        """Surface iz in column (ix, iy) closest to target_iz, within tolerance."""
        zs = sg.surface_lookup.get((ix, iy))
        if zs is None or len(zs) == 0:
            return None
        distances = np.abs(zs - target_iz)
        best = int(distances.argmin())
        if int(distances[best]) > tolerance_cells:
            return None
        return (ix, iy, int(zs[best]))

    def _assemble_waypoints(
        self,
        sg: SurfaceGraph,
        node_seq: list[int],
        start_pose: tuple[float, float, float],
        goal_pose: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        """Chain cached edge paths into a continuous waypoint list, bracketed by the actual poses."""
        voxel = self.config.voxel_size
        cells: list[tuple[int, int, int]] = []
        for i in range(len(node_seq) - 1):
            a, b = node_seq[i], node_seq[i + 1]
            edge_path: np.ndarray = sg.graph[a][b]["path"]
            # Stored path runs from min(a, b) to max(a, b). Reverse if traversing the other way.
            if a > b:
                edge_path = edge_path[::-1]
            tail = edge_path if i == 0 else edge_path[1:]
            for c in tail:
                cells.append((int(c[0]), int(c[1]), int(c[2])))

        waypoints: list[tuple[float, float, float]] = [start_pose]
        for cix, ciy, ciz in cells:
            waypoints.append(surface_point_xyz(cix, ciy, ciz, voxel))
        waypoints.append(goal_pose)
        return waypoints
