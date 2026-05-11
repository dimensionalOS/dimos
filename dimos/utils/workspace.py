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

"""Reachability + Yoshikawa-manipulability workspace analysis via Pinocchio.

Run ``python -m dimos.utils.workspace <urdf> [viz|query|suggest|interactive]``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np


class WorkspaceMap:
    """Sampled reachability + manipulability map. Works on any URDF Pinocchio can load."""

    def __init__(
        self,
        urdf_path: str | Path,
        n_samples: int = 100_000,
        *,
        ee_joint_id: int | None = None,
        seed: int = 42,
    ) -> None:
        import pinocchio

        self._pin = pinocchio
        self.model = pinocchio.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.ee_id = ee_joint_id if ee_joint_id is not None else (self.model.njoints - 1)
        self.q_lo = self.model.lowerPositionLimit.copy()
        self.q_hi = self.model.upperPositionLimit.copy()
        self._sample(n_samples, seed)

    def _sample(self, n: int, seed: int) -> None:
        rng = np.random.default_rng(seed)
        self.positions = np.empty((n, 3))
        self.configs = np.empty((n, self.model.nq))
        self.manipulability = np.empty(n)

        for i in range(n):
            q = rng.uniform(self.q_lo, self.q_hi)
            self._pin.forwardKinematics(self.model, self.data, q)
            self._pin.computeJointJacobians(self.model, self.data, q)

            self.positions[i] = self.data.oMi[self.ee_id].translation
            self.configs[i] = q

            J = self._pin.getJointJacobian(
                self.model,
                self.data,
                self.ee_id,
                self._pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            JJt = J[:3, :] @ J[:3, :].T
            self.manipulability[i] = np.sqrt(max(0.0, np.linalg.det(JJt)))

    def query(
        self,
        target: np.ndarray | tuple[float, float, float],
        radius: float = 0.03,
    ) -> dict[str, Any]:
        """Check reachability at a Cartesian target (x,y,z)."""
        target = np.asarray(target, dtype=np.float64)
        dists = np.linalg.norm(self.positions - target, axis=1)
        mask = dists < radius
        if int(mask.sum()) == 0:
            for r in (0.05, 0.08, 0.12):
                mask = dists < r
                if mask.sum() > 0:
                    break
        n_nearby = int(mask.sum())
        if n_nearby == 0:
            nearest = int(np.argmin(dists))
            return {
                "reachable": False,
                "n_configs": 0,
                "mean_manipulability": 0.0,
                "nearest_distance": float(dists[nearest]),
                "nearest_position": self.positions[nearest].tolist(),
                "nearest_config": self.configs[nearest].tolist(),
            }
        manip_nearby = self.manipulability[mask]
        indices = np.where(mask)[0]
        best_idx = indices[int(np.argmax(manip_nearby))]
        return {
            "reachable": True,
            "n_configs": n_nearby,
            "mean_manipulability": float(manip_nearby.mean()),
            "max_manipulability": float(manip_nearby.max()),
            "best_config": self.configs[best_idx].tolist(),
            "best_position": self.positions[best_idx].tolist(),
            "distance": float(dists[best_idx]),
        }

    def stats(self) -> str:
        """Human-readable workspace summary (bounds, reach, hull volume)."""
        p = self.positions
        lines = [
            "Workspace stats:",
            f"  Samples: {len(p):,}",
            f"  X range: [{p[:, 0].min():.3f}, {p[:, 0].max():.3f}] m",
            f"  Y range: [{p[:, 1].min():.3f}, {p[:, 1].max():.3f}] m",
            f"  Z range: [{p[:, 2].min():.3f}, {p[:, 2].max():.3f}] m",
            f"  Max reach from origin: {np.linalg.norm(p, axis=1).max():.3f} m",
            f"  Manipulability: [{self.manipulability.min():.4f}, {self.manipulability.max():.4f}]",
        ]
        try:
            from scipy.spatial import ConvexHull

            hull = ConvexHull(p)
            lines.append(f"  Convex hull volume: {hull.volume:.4f} m³")
        except Exception:
            pass
        return "\n".join(lines)


# ── Meshcat visualization helpers ──────────────────────────────────────────


def _colormap(values: np.ndarray) -> np.ndarray:
    """Red→green colormap, clipped at 2nd/98th percentile for contrast."""
    v = values.copy()
    lo, hi = np.percentile(v, 2), np.percentile(v, 98)
    v = np.clip((v - lo) / (hi - lo + 1e-12), 0.0, 1.0)
    colors = np.zeros((len(v), 3), dtype=np.uint8)
    colors[:, 0] = ((1.0 - v) * 255).astype(np.uint8)
    colors[:, 1] = (v * 255).astype(np.uint8)
    colors[:, 2] = 25
    return colors


def render_cloud(meshcat: Any, ws: WorkspaceMap, path: str = "/workspace") -> None:
    """Render a WorkspaceMap's EE positions to Drake's Meshcat, colored by manipulability."""
    from pydrake.perception import BaseField, Fields, PointCloud

    cloud = PointCloud(len(ws.positions), Fields(int(BaseField.kXYZs) | int(BaseField.kRGBs)))
    cloud.mutable_xyzs()[:] = ws.positions.T.astype(np.float32)
    cloud.mutable_rgbs()[:] = _colormap(ws.manipulability).T
    meshcat.SetObject(path, cloud, point_size=0.004)


def render_target(
    meshcat: Any,
    pos: list[float] | tuple[float, float, float],
    name: str = "target",
    color: tuple[float, float, float] = (1.0, 0.0, 0.0),
) -> None:
    from pydrake.geometry import Rgba, Sphere
    from pydrake.math import RigidTransform

    meshcat.SetObject(f"/{name}", Sphere(0.015), Rgba(*color, 0.8))
    meshcat.SetTransform(f"/{name}", RigidTransform(list(pos)))


# ── CLI ────────────────────────────────────────────────────────────────────


def _cmd_viz(args: argparse.Namespace) -> int:
    from pydrake.geometry import Meshcat

    ws = WorkspaceMap(args.urdf, args.samples, ee_joint_id=args.ee_joint_id)
    print(ws.stats())
    meshcat = Meshcat()
    print(f"\nMeshcat: {meshcat.web_url()}")
    print("Green = dexterous, Red = near singularity\n")
    render_cloud(meshcat, ws)
    print("Press Ctrl-C to exit.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    return 0


def _cmd_query(args: argparse.Namespace) -> int:
    ws = WorkspaceMap(args.urdf, args.samples, ee_joint_id=args.ee_joint_id)
    result = ws.query((args.x, args.y, args.z))
    print(f"\nTarget: ({args.x:.3f}, {args.y:.3f}, {args.z:.3f})")
    if result["reachable"]:
        print(f"  REACHABLE — {result['n_configs']} configs within 3cm")
        print(f"  Mean manipulability: {result['mean_manipulability']:.4f}")
        print(f"  Best config: {[f'{q:.3f}' for q in result['best_config']]}")
        p = result["best_position"]
        print(f"  Best EE position: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    else:
        print("  NOT REACHABLE")
        p = result["nearest_position"]
        print(f"  Nearest reachable point: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
        print(f"  Distance to nearest: {result['nearest_distance']:.3f} m")
        print(f"  Nearest config: {[f'{q:.3f}' for q in result['nearest_config']]}")
    return 0


def _cmd_suggest(args: argparse.Namespace) -> int:
    ws = WorkspaceMap(args.urdf, args.samples, ee_joint_id=args.ee_joint_id)
    target = np.array([args.x, args.y, args.z])
    dists = np.linalg.norm(ws.positions - target, axis=1)
    closest = np.argsort(dists)[:20]
    sorted_by_manip = sorted(closest, key=lambda i: -ws.manipulability[i])

    print(f"\nSuggested poses near ({args.x:.3f}, {args.y:.3f}, {args.z:.3f}):")
    print(f"{'#':>3}  {'dist':>6}  {'manip':>7}  {'position':>30}  joint config")
    print("-" * 100)
    for rank, idx in enumerate(sorted_by_manip[:10], 1):
        p, q = ws.positions[idx], ws.configs[idx]
        pos_str = f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})"
        q_str = "[" + ", ".join(f"{v:.2f}" for v in q) + "]"
        print(
            f"{rank:>3}  {dists[idx]:>6.3f}  {ws.manipulability[idx]:>7.4f}  {pos_str:>30}  {q_str}"
        )
    return 0


def _cmd_interactive(args: argparse.Namespace) -> int:
    from pydrake.geometry import Meshcat

    ws = WorkspaceMap(args.urdf, args.samples, ee_joint_id=args.ee_joint_id)
    print(ws.stats())
    meshcat = Meshcat()
    print(f"\nMeshcat: {meshcat.web_url()}")
    render_cloud(meshcat, ws)
    print("\nType target as 'x y z' (meters), or 'q' to quit.\n")

    while True:
        try:
            line = input("target> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in ("q", "quit", "exit"):
            break
        parts = line.split()
        if len(parts) != 3:
            print("  Enter three floats: x y z")
            continue
        try:
            x, y, z = (float(p) for p in parts)
        except ValueError:
            print("  Invalid input")
            continue

        result = ws.query((x, y, z))
        if result["reachable"]:
            render_target(meshcat, (x, y, z), "target", (0.0, 1.0, 0.0))
            render_target(meshcat, result["best_position"], "best_ee", (0.0, 0.5, 1.0))
            print(
                f"  REACHABLE — {result['n_configs']} configs, "
                f"manip={result['mean_manipulability']:.4f}"
            )
            print(f"  Joint config: {[round(q, 3) for q in result['best_config']]}")
        else:
            render_target(meshcat, (x, y, z), "target", (1.0, 0.0, 0.0))
            render_target(meshcat, result["nearest_position"], "nearest", (1.0, 0.5, 0.0))
            print(f"  NOT REACHABLE — nearest is {result['nearest_distance']:.3f}m away")
            print(f"  Nearest config: {[round(q, 3) for q in result['nearest_config']]}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("urdf", type=Path, help="Path to a URDF parseable by Pinocchio")
    ap.add_argument("--samples", type=int, default=100_000)
    ap.add_argument(
        "--ee-joint-id",
        type=int,
        default=None,
        help="Pinocchio joint index for the end-effector (default: last joint)",
    )

    sub = ap.add_subparsers(dest="command")
    sub.add_parser("viz", help="Visualize workspace in Meshcat (default)")
    sub.add_parser("interactive", help="Visualize + interactive target query")
    q = sub.add_parser("query", help="Query if a target is reachable")
    q.add_argument("x", type=float)
    q.add_argument("y", type=float)
    q.add_argument("z", type=float)
    s = sub.add_parser("suggest", help="Suggest reachable poses near a target")
    s.add_argument("x", type=float)
    s.add_argument("y", type=float)
    s.add_argument("z", type=float)

    args = ap.parse_args()
    cmd = args.command or "viz"
    return {
        "viz": _cmd_viz,
        "query": _cmd_query,
        "suggest": _cmd_suggest,
        "interactive": _cmd_interactive,
    }[cmd](args)


if __name__ == "__main__":
    sys.exit(main())
