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

"""Re-derive a stream's recorded poses as a static URDF offset from a base stream.

Given a URDF and two `stream:urdf_frame` mappings, (destructively) rewrites the
downstream stream's `pose_*` so each message is the base stream's world pose at
that timestamp composed with the fixed base->downstream transform read from the
URDF:

    world_to_downstream(t) = world_to_base(nearest t) . T_urdf(base_frame, downstream_frame)

The base stream is assumed to be an odometry-type stream whose stored pose is the
base frame in the world. Only pose metadata (the `pose_*` columns + the rtree
point index) is touched. Point-pose streams only for now (lidar clouds later).

    uv run python dimos/mapping/recording/scripts/db_reform.py REC.db ROBOT.urdf \
        --base_frame fastlio_odometry:base_link \
        --downstream_frame color_image:camera_optical
"""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
import sqlite3
import xml.etree.ElementTree as ElementTree

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def _identity() -> Transform:
    return Transform(translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))


def parse_urdf_graph(urdf_path: str) -> dict[str, list[tuple[str, Transform]]]:
    """Undirected frame graph from a URDF: each joint origin is the parent->child
    transform (and its inverse the child->parent edge). Joint type is ignored —
    the origin is the static mount, which is what we want."""
    root = ElementTree.parse(urdf_path).getroot()
    graph: dict[str, list[tuple[str, Transform]]] = {}
    for joint in root.findall("joint"):
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        if parent_element is None or child_element is None:
            continue
        parent = parent_element.get("link")
        child = child_element.get("link")
        if parent is None or child is None:
            continue
        origin = joint.find("origin")
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        if origin is not None:
            xyz_attr = origin.get("xyz")
            rpy_attr = origin.get("rpy")
            if xyz_attr:
                xyz = tuple(float(value) for value in xyz_attr.split())  # type: ignore[assignment]
            if rpy_attr:
                rpy = tuple(float(value) for value in rpy_attr.split())  # type: ignore[assignment]
        edge = Transform(translation=Vector3(*xyz), rotation=Quaternion.from_euler(Vector3(*rpy)))
        graph.setdefault(parent, []).append((child, edge))
        graph.setdefault(child, []).append((parent, edge.inverse()))
    return graph


def transform_between(
    graph: dict[str, list[tuple[str, Transform]]], source: str, target: str
) -> Transform:
    """Static transform source->target by composing joint edges along the URDF
    path (BFS). `world_p_target = world_p_source + transform_between(...)`."""
    if source not in graph:
        raise SystemExit(f"frame '{source}' not in URDF")
    if target not in graph:
        raise SystemExit(f"frame '{target}' not in URDF")
    accumulated = {source: _identity()}
    queue = deque([source])
    while queue:
        node = queue.popleft()
        if node == target:
            return accumulated[node]
        for neighbor, edge in graph[node]:
            if neighbor not in accumulated:
                accumulated[neighbor] = accumulated[node] + edge
                queue.append(neighbor)
    raise SystemExit(f"no URDF path from '{source}' to '{target}'")


def _load_world_poses(conn: sqlite3.Connection, stream: str) -> tuple[np.ndarray, list[Transform]]:
    rows = conn.execute(
        f"SELECT ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
        f'FROM "{stream}" WHERE pose_qw IS NOT NULL ORDER BY ts'
    ).fetchall()
    if not rows:
        raise SystemExit(f"base stream '{stream}' has no populated poses")
    timestamps = np.array([row[0] for row in rows])
    poses = [
        Transform(
            translation=Vector3(row[1], row[2], row[3]),
            rotation=Quaternion(row[4], row[5], row[6], row[7]),
        )
        for row in rows
    ]
    return timestamps, poses


def _nearest(timestamps: np.ndarray, value: float) -> int:
    index = int(np.searchsorted(timestamps, value))
    index = min(max(index, 0), len(timestamps) - 1)
    if index > 0 and abs(timestamps[index - 1] - value) < abs(timestamps[index] - value):
        index -= 1
    return index


def _split_mapping(mapping: str) -> tuple[str, str]:
    if ":" not in mapping:
        raise SystemExit(f"expected stream:urdf_frame, got '{mapping}'")
    stream, frame = mapping.split(":", 1)
    return stream, frame


def reform(db_path: str, urdf_path: str, base: str, downstream: str) -> int:
    base_stream, base_frame = _split_mapping(base)
    down_stream, down_frame = _split_mapping(downstream)
    base_to_downstream = transform_between(parse_urdf_graph(urdf_path), base_frame, down_frame)

    conn = sqlite3.connect(db_path)
    try:
        tables = {
            row[0] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")
        }
        if down_stream not in tables:
            raise SystemExit(f"no '{down_stream}' stream in db")
        has_rtree = f"{down_stream}_rtree" in tables
        base_ts, base_poses = _load_world_poses(conn, base_stream)
        rows = conn.execute(f'SELECT id, ts FROM "{down_stream}"').fetchall()

        conn.execute("BEGIN")
        for row_id, ts in rows:
            world_to_down = base_poses[_nearest(base_ts, ts)] + base_to_downstream
            position = world_to_down.translation
            rotation = world_to_down.rotation
            conn.execute(
                f'UPDATE "{down_stream}" SET pose_x=?,pose_y=?,pose_z=?,'
                f"pose_qx=?,pose_qy=?,pose_qz=?,pose_qw=? WHERE id=?",
                (
                    position.x,
                    position.y,
                    position.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                    row_id,
                ),
            )
            if has_rtree:
                conn.execute(
                    f'INSERT OR REPLACE INTO "{down_stream}_rtree"'
                    f"(id,x_min,x_max,y_min,y_max,z_min,z_max) VALUES (?,?,?,?,?,?,?)",
                    (
                        row_id,
                        position.x,
                        position.x,
                        position.y,
                        position.y,
                        position.z,
                        position.z,
                    ),
                )
        conn.execute("COMMIT")
        return len(rows)
    except Exception:
        conn.execute("ROLLBACK")
        raise
    finally:
        conn.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("db", help="recording mem2.db (edited in place)")
    parser.add_argument("urdf", help="URDF describing the frame tree")
    parser.add_argument(
        "--base_frame", required=True, help="stream:urdf_frame for the world-frame odometry base"
    )
    parser.add_argument("--downstream_frame", required=True, help="stream:urdf_frame to re-derive")
    args = parser.parse_args()
    if not Path(args.db).exists():
        raise SystemExit(f"no such db: {args.db}")
    if not Path(args.urdf).exists():
        raise SystemExit(f"no such urdf: {args.urdf}")

    print(f">> {args.downstream_frame}  :=  {args.base_frame} . T_urdf")
    count = reform(args.db, args.urdf, args.base_frame, args.downstream_frame)
    print(f"   rewrote {count} '{args.downstream_frame.split(':', 1)[0]}' poses")
    print("done")


if __name__ == "__main__":
    main()
