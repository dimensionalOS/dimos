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

"""One-shot viser viewer for capability maps.

The default view is the **body-frame workspace**: the arm's actual
reachable volume in pelvis coordinates (the asymmetric blob a human
expects — no heading quotient, no symmetry artifacts), rendered as a
point cloud colored red→green by *dexterity* (fraction of approach
angles reachable per cell: red = one way in, green = approach from
anywhere), inside a translucent shell of everything reachable. An
opaque voxel style is available for a solid volume reading.

The IK ghost poses a **rigid** kinematic model. The real G1 arms are
compliant and sag a few cm under gravity at manipulation PD gains —
gravity-feedforward compensation belongs in the mink control task, not
this viewer.

Interactive extras:

- **IK target gizmo** — drag a 6-DOF target around; a mink QP poses the
  G1 URDF's arm to reach it live, and the status line reports the IK
  result next to the map's prediction for the same pose.
- **Slice planes** — a vertical plane (adjustable yaw) and a horizontal
  plane (adjustable height) showing dexterity cross-sections in scene.
- The quotient-map views from the planning side ("canonical": ψ = 0
  gauge; "position": orientation-marginal revolved profile) remain as
  alternate modes.

CLI::

    python -m dimos.manipulation.reachability.viewer \\
        --map ~/Desktop/g1_reachability/g1_left_capability.npz \\
        --map ~/Desktop/g1_reachability/g1_right_capability.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.manipulation.reachability.capability_map import CapabilityMap
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_REPO_ROOT = Path(__file__).parents[3]
_G1_URDF = _REPO_ROOT / "data" / "g1_urdf" / "g1.urdf"
_EMPTY_GRAY = (45, 45, 55)


# ----------------------------------------------------------------------
# Pure geometry builders (unit-tested without a server)


def body_point_cloud(
    cap: CapabilityMap, min_dexterity: float, min_count: int = 1
) -> tuple[np.ndarray, np.ndarray]:
    """(points, dexterity) at occupied body-frame cell centers."""
    params = cap.params
    dexterity = cap.body_dexterity()
    keep = (cap.body_counts >= min_count) & (dexterity >= min_dexterity)
    iz, ix, iy = np.nonzero(keep)
    if len(iz) == 0:
        return np.empty((0, 3)), np.empty(0)
    centers = (np.arange(params.n_xy) + 0.5) * params.cell - params.r_xy
    z_centers = (np.arange(params.n_z) + 0.5) * params.cell + params.z_min
    points = np.stack([centers[ix], centers[iy], z_centers[iz]], axis=1)
    return points, dexterity[iz, ix, iy]


def body_voxel_mesh(
    cap: CapabilityMap, min_dexterity: float, min_count: int = 1
) -> tuple[Any, int]:
    """Trimesh box-mesh of body-frame cells, vertex-colored by dexterity.

    Returns (mesh | None, n_voxels). ``min_dexterity`` in [0, 1] selects the
    dexterous core; 0 shows everything reachable.
    """
    import trimesh

    params = cap.params
    dexterity = cap.body_dexterity()
    keep = (cap.body_counts >= min_count) & (dexterity >= min_dexterity)
    n_voxels = int(keep.sum())
    if n_voxels == 0:
        return None, 0

    # (z, x, y) → (x, y, z) voxel matrix.
    matrix = keep.transpose(1, 2, 0)
    import matplotlib

    colors = np.zeros((*matrix.shape, 4), dtype=np.uint8)
    rgba = matplotlib.colormaps["RdYlGn"](dexterity.transpose(1, 2, 0))
    colors[..., :3] = (rgba[..., :3] * 255).astype(np.uint8)
    colors[..., 3] = 255

    transform = np.eye(4)
    transform[0, 0] = transform[1, 1] = transform[2, 2] = params.cell
    transform[:3, 3] = (
        -params.r_xy + params.cell / 2,
        -params.r_xy + params.cell / 2,
        params.z_min + params.cell / 2,
    )
    grid = trimesh.voxel.VoxelGrid(matrix, transform=transform)
    return grid.as_boxes(colors=colors), n_voxels


def slice_image_yaw(
    cap: CapabilityMap, yaw_deg: float, px_per_cell: int = 6
) -> tuple[np.ndarray, float, float]:
    """Dexterity cross-section along the vertical plane through the pelvis
    axis at the given yaw. Returns (RGB image, width_m, height_m); image x
    spans [-r_xy, r_xy] along the yaw direction, y spans [z_min, z_max]."""
    params = cap.params
    n_s = params.n_xy * px_per_cell
    n_z = params.n_z * px_per_cell
    s = np.linspace(-params.r_xy, params.r_xy, n_s)
    z = np.linspace(params.z_max, params.z_min, n_z)  # row 0 = top
    yaw = np.deg2rad(yaw_deg)
    xs = np.cos(yaw) * s
    ys = np.sin(yaw) * s
    positions = np.stack(
        [
            np.broadcast_to(xs, (n_z, n_s)).reshape(-1),
            np.broadcast_to(ys, (n_z, n_s)).reshape(-1),
            np.broadcast_to(z[:, None], (n_z, n_s)).reshape(-1),
        ],
        axis=1,
    )
    image = _dexterity_image(cap, positions, (n_z, n_s))
    return image, 2 * params.r_xy, params.z_max - params.z_min


def slice_image_height(
    cap: CapabilityMap, z: float, px_per_cell: int = 6
) -> tuple[np.ndarray, float, float]:
    """Dexterity cross-section on the horizontal plane at height z."""
    params = cap.params
    n = params.n_xy * px_per_cell
    axis = np.linspace(-params.r_xy, params.r_xy, n)
    xx, yy = np.meshgrid(axis, -axis)  # row 0 = +y edge so the image reads like a map
    positions = np.stack(
        [xx.reshape(-1), yy.reshape(-1), np.full(n * n, z)],
        axis=1,
    )
    image = _dexterity_image(cap, positions, (n, n))
    return image, 2 * params.r_xy, 2 * params.r_xy


def _dexterity_image(cap: CapabilityMap, positions: np.ndarray, shape: tuple[int, int]):
    import matplotlib

    dexterity = cap.body_dexterity()
    iz, ix, iy, valid = cap.body_indices(positions)
    values = np.zeros(len(positions))
    occupied = np.zeros(len(positions), dtype=bool)
    values[valid] = dexterity[iz[valid], ix[valid], iy[valid]]
    occupied[valid] = cap.body_counts[iz[valid], ix[valid], iy[valid]] > 0

    rgba = matplotlib.colormaps["RdYlGn"](np.clip(values / max(values.max(), 1e-9), 0, 1))
    image = (rgba[:, :3] * 255).astype(np.uint8)
    image[~occupied] = _EMPTY_GRAY
    return image.reshape(*shape, 3)


def canonical_cloud(
    cap: CapabilityMap,
    theta_lo_deg: float,
    theta_hi_deg: float,
    gamma_bin: int | None,
    min_score: int,
) -> tuple[np.ndarray, np.ndarray]:
    """(points, scores) for the ψ=0 gauge representative of every marked cell."""
    params = cap.params
    t_lo = int(np.clip(theta_lo_deg / 180.0 * params.n_theta, 0, params.n_theta - 1))
    t_hi = int(np.clip(theta_hi_deg / 180.0 * params.n_theta + 1, t_lo + 1, params.n_theta))
    block = cap.counts[:, t_lo:t_hi]
    scores = block[..., gamma_bin].max(axis=1) if gamma_bin is not None else block.max(axis=(1, 4))
    iz, ix, iy = np.nonzero(scores >= min_score)
    if len(iz) == 0:
        return np.empty((0, 3)), np.empty(0)
    centers = (np.arange(params.n_xy) + 0.5) * params.cell - params.r_xy
    z_centers = (np.arange(params.n_z) + 0.5) * params.cell + params.z_min
    points = np.stack([-centers[ix], -centers[iy], z_centers[iz]], axis=1)
    return points, scores[iz, ix, iy].astype(float)


def position_cloud(
    cap: CapabilityMap, min_score: int, ring_step: int = 12
) -> tuple[np.ndarray, np.ndarray]:
    """Orientation-marginal radial profile revolved into rings around the axis."""
    params = cap.params
    radial = cap.position_scores()
    iz, ir = np.nonzero(radial >= min_score)
    if len(iz) == 0:
        return np.empty((0, 3)), np.empty(0)
    points, scores = [], []
    z_centers = (np.arange(params.n_z) + 0.5) * params.cell + params.z_min
    for z_idx, r_idx in zip(iz, ir, strict=True):
        radius = (r_idx + 0.5) * params.cell
        n_pts = max(int(2 * np.pi * radius / params.cell / 2), 1) if radius > 0 else 1
        n_pts = min(n_pts, ring_step * 8)
        angles = np.linspace(0, 2 * np.pi, n_pts, endpoint=False)
        ring = np.stack(
            [radius * np.cos(angles), radius * np.sin(angles), np.full(n_pts, z_centers[z_idx])],
            axis=1,
        )
        points.append(ring)
        scores.append(np.full(n_pts, float(radial[z_idx, r_idx])))
    return np.concatenate(points), np.concatenate(scores)


def score_colors(scores: np.ndarray, vmax: float | None = None) -> np.ndarray:
    """Score → red-to-green uint8 colors (red = barely reachable, green = rich)."""
    import matplotlib

    vmax = vmax or max(float(scores.max(initial=1.0)), 1.0)
    rgba = matplotlib.colormaps["RdYlGn"](np.clip(scores / vmax, 0, 1))
    return (rgba[:, :3] * 255).astype(np.uint8)


# ----------------------------------------------------------------------
# Live IK (poses the URDF arm at the gizmo target)


class ArmIK:
    """mink QP IK for one arm on the construction MJCF; non-arm DOF masked."""

    def __init__(self, side: str) -> None:
        import mink

        from dimos.manipulation.reachability.construct import _ArmSampler, g1_spec

        self._mink = mink
        spec = g1_spec(side)
        self._sampler = _ArmSampler(spec)
        self.joint_names = list(spec.joint_names)
        model = self._sampler.model
        self._configuration = mink.Configuration(model)
        self._frame = mink.FrameTask(
            frame_name=spec.ee_body,
            frame_type="body",
            position_cost=1.0,
            orientation_cost=0.6,
            lm_damping=1.0,
        )
        claimed = {
            int(model.jnt_dofadr[j])
            for j in range(model.njnt)
            if int(model.jnt_qposadr[j]) in set(self._sampler.qpos_adr.tolist())
        }
        self._velocity_mask = np.zeros(model.nv)
        self._velocity_mask[list(claimed)] = 1.0
        self._tasks = [self._frame]
        self._limits = [mink.ConfigurationLimit(model)]
        self._q_warm = self._sampler._q_base.copy()

    def solve(
        self, position: np.ndarray, wxyz: np.ndarray, restarts: int = 5
    ) -> tuple[dict[str, float], bool, float]:
        """Solve toward a grasp-center target; returns (arm joints by model
        joint name, reached?, position error in m).

        Warm-started from the previous solve so dragging feels continuous;
        falls back to random restarts (keeping the best) when the warm
        start stalls in a local minimum."""
        import mujoco

        mink = self._mink
        sampler = self._sampler
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, np.asarray(wxyz, dtype=np.float64))
        rotation = rotation.reshape(3, 3)
        body_position = np.asarray(position) - rotation @ sampler.grasp_offset
        self._frame.set_target(
            mink.SE3.from_rotation_and_translation(mink.SO3(np.asarray(wxyz)), body_position)
        )

        best_q, best_error = None, np.inf
        rng = np.random.default_rng(0)
        for attempt in range(1 + restarts):
            q0 = self._q_warm.copy()
            if attempt > 0:
                q0[sampler.qpos_adr] = rng.uniform(sampler.lower, sampler.upper)
            self._configuration.update(q0)
            # The task error decays ~×(1 − gain·dt) per iteration; 300 steps
            # at dt=0.05 reduce it by ~2e-7. Stopping at 60 left ~5% of the
            # initial error (tens of mm) and looked like an IK failure.
            for _ in range(300):
                velocity = (
                    mink.solve_ik(
                        self._configuration, self._tasks, 0.05, "daqp", limits=self._limits
                    )
                    * self._velocity_mask
                )
                self._configuration.integrate_inplace(velocity, 0.05)
                if float(np.linalg.norm(velocity)) < 1e-4:
                    break
            error = self._position_error(self._configuration.q, position)
            if error < best_error:
                best_q, best_error = self._configuration.q.copy(), error
            if best_error < 0.01:
                break

        assert best_q is not None
        self._q_warm = best_q.copy()
        joints = {
            name: float(best_q[adr])
            for name, adr in zip(self.joint_names, sampler.qpos_adr, strict=True)
        }
        return joints, best_error < 0.02, best_error

    def _position_error(self, q: np.ndarray, target_position: np.ndarray) -> float:
        import mujoco

        sampler = self._sampler
        data, model = sampler.data, sampler.model
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        xmat = data.xmat[sampler.ee_body_id].reshape(3, 3)
        reached = data.xpos[sampler.ee_body_id] + xmat @ sampler.grasp_offset
        return float(np.linalg.norm(reached - np.asarray(target_position)))


# ----------------------------------------------------------------------
# Server


def _add_g1(server, pelvis_height: float):
    """G1 URDF at the map pelvis pose; returns (viser_urdf | None, joint names)."""
    try:
        from viser.extras import ViserUrdf
        import yourdfpy

        urdf = yourdfpy.URDF.load(str(_G1_URDF))
        viser_urdf = ViserUrdf(server, urdf, root_node_name="/g1")
        server.scene.add_frame("/g1", position=(0.0, 0.0, pelvis_height), show_axes=False)
        names = list(viser_urdf.get_actuated_joint_names())
        viser_urdf.update_cfg(np.zeros(len(names)))
        return viser_urdf, names
    except Exception as e:  # context only — everything else works without it
        logger.warning(f"G1 URDF context unavailable ({e})")
        server.scene.add_frame(
            "/g1", position=(0.0, 0.0, pelvis_height), show_axes=True, axes_length=0.25
        )
        return None, []


def serve(maps: dict[str, CapabilityMap], port: int = 8082) -> None:
    """Start the one-shot viewer (blocks until Ctrl-C)."""
    import viser

    server = viser.ViserServer(host="0.0.0.0", port=port)
    first = next(iter(maps.values()))
    params = first.params
    server.scene.add_grid("/ground", width=4.0, height=4.0, cell_size=0.25)
    viser_urdf, urdf_joint_names = _add_g1(server, params.pelvis_height)

    with server.gui.add_folder("view"):
        side = server.gui.add_dropdown("arm", tuple(maps), initial_value=next(iter(maps)))
        mode = server.gui.add_dropdown(
            "mode",
            (
                "workspace (body frame)",
                "canonical (approach az = +x)",
                "position (any orientation)",
            ),
        )
        style = server.gui.add_dropdown("style", ("points", "voxels"), initial_value="points")
        point_size = server.gui.add_slider(
            "point size [mm]", min=5, max=60, step=1, initial_value=35
        )
        dexterity_pct = server.gui.add_slider(
            "min dexterity [%]", min=0, max=60, step=1, initial_value=0
        )
        shell = server.gui.add_checkbox("show reachable shell", initial_value=True)
        min_score = server.gui.add_slider("min score", min=1, max=60, step=1, initial_value=1)
        theta_lo = server.gui.add_slider("θ min [deg]", min=0, max=180, step=5, initial_value=0)
        theta_hi = server.gui.add_slider("θ max [deg]", min=0, max=180, step=5, initial_value=180)

    with server.gui.add_folder("slices"):
        show_yaw_slice = server.gui.add_checkbox("vertical slice", initial_value=False)
        yaw_slice = server.gui.add_slider(
            "slice yaw [deg]", min=-180, max=180, step=5, initial_value=0
        )
        show_z_slice = server.gui.add_checkbox("horizontal slice", initial_value=False)
        z_slice = server.gui.add_slider(
            "slice height [m]",
            min=params.z_min,
            max=params.z_max,
            step=params.cell,
            initial_value=0.9,
        )

    with server.gui.add_folder("IK target"):
        ik_enabled = server.gui.add_checkbox("drag-to-reach", initial_value=False)
        ik_status = server.gui.add_text(
            "status", initial_value="enable to pose the arm", disabled=True
        )

    solvers: dict[str, ArmIK | None] = {}
    gizmo = None

    def current_map() -> CapabilityMap:
        return maps[side.value]

    def refresh_volume(_=None) -> None:
        cap = current_map()
        for name in ("/reachability/core", "/reachability/shell", "/reachability/points"):
            try:
                server.scene.remove_by_name(name)
            except Exception:
                pass
        if mode.value.startswith("workspace"):
            n = 0
            if style.value == "voxels":
                core, n = body_voxel_mesh(cap, dexterity_pct.value / 100.0)
                if core is not None:
                    server.scene.add_mesh_trimesh("/reachability/core", core)
            else:
                points, dexterity = body_point_cloud(cap, dexterity_pct.value / 100.0)
                n = len(points)
                if n:
                    server.scene.add_point_cloud(
                        "/reachability/points",
                        points=points.astype(np.float32),
                        colors=score_colors(dexterity, vmax=max(float(dexterity.max()), 1e-9)),
                        point_size=point_size.value / 1000.0,
                        point_shape="circle",
                    )
            if shell.value:
                outer, _ = body_voxel_mesh(cap, 0.0)
                if outer is not None:
                    server.scene.add_mesh_simple(
                        "/reachability/shell",
                        vertices=np.asarray(outer.vertices),
                        faces=np.asarray(outer.faces),
                        color=(140, 200, 150),
                        opacity=0.1,
                    )
            logger.info(f"workspace view: {n} cells at ≥{dexterity_pct.value}% dexterity")
        else:
            if mode.value.startswith("position"):
                points, scores = position_cloud(cap, int(min_score.value))
            else:
                points, scores = canonical_cloud(
                    cap, theta_lo.value, theta_hi.value, None, int(min_score.value)
                )
            if len(points):
                server.scene.add_point_cloud(
                    "/reachability/points",
                    points=points.astype(np.float32),
                    colors=score_colors(scores),
                    point_size=point_size.value / 1000.0,
                    point_shape="circle",
                )

    def refresh_slices(_=None) -> None:
        cap = current_map()
        try:
            server.scene.remove_by_name("/slice/yaw")
        except Exception:
            pass
        try:
            server.scene.remove_by_name("/slice/z")
        except Exception:
            pass
        if show_yaw_slice.value:
            image, width, height = slice_image_yaw(cap, yaw_slice.value)
            yaw = np.deg2rad(yaw_slice.value)
            server.scene.add_image(
                "/slice/yaw",
                image,
                render_width=width,
                render_height=height,
                position=(0.0, 0.0, (params.z_min + params.z_max) / 2),
                wxyz=_plane_wxyz(yaw),
            )
        if show_z_slice.value:
            image, width, height = slice_image_height(cap, z_slice.value)
            server.scene.add_image(
                "/slice/z",
                image,
                render_width=width,
                render_height=height,
                position=(0.0, 0.0, float(z_slice.value)),
                wxyz=(1.0, 0.0, 0.0, 0.0),
            )

    def refresh_ik(_=None) -> None:
        nonlocal gizmo
        if not ik_enabled.value:
            if gizmo is not None:
                gizmo.remove()
                gizmo = None
            return
        if gizmo is None:
            gizmo = server.scene.add_transform_controls(
                "/ik_target", scale=0.18, position=(0.35, 0.2, 1.05)
            )
            gizmo.on_update(solve_ik)
        solve_ik()

    def solve_ik(_=None) -> None:
        if gizmo is None:
            return
        cap = current_map()
        if cap.side not in solvers:
            try:
                solvers[cap.side] = ArmIK(cap.side)
            except Exception as e:
                solvers[cap.side] = None
                logger.warning(f"IK unavailable: {e}")
        solver = solvers[cap.side]
        if solver is None:
            ik_status.value = "IK unavailable (pip install 'dimos[ik]')"
            return
        position = np.asarray(gizmo.position, dtype=np.float64)
        wxyz = np.asarray(gizmo.wxyz, dtype=np.float64)
        joints, reached, error = solver.solve(position, wxyz)

        import mujoco

        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, wxyz)
        score = cap.scores(position[None], rotation.reshape(1, 3, 3))[0]
        iz, ix, iy, valid = cap.body_indices(position[None])
        dexterity = cap.body_dexterity()[iz[0], ix[0], iy[0]] if valid[0] else 0.0
        # Rigid-model kinematics: the real G1 arms are compliant and sag a
        # few cm under gravity at low PD gains — treat the posed arm as the
        # commanded pose, not where the hardware would settle.
        ik_status.value = (
            f"IK {'reached' if reached else 'FAILED'} (err {error * 1000:.0f} mm) | "
            f"map score {score} | dexterity {dexterity:.0%} | rigid model (no sag)"
        )

        if viser_urdf is not None and urdf_joint_names:
            cfg = np.zeros(len(urdf_joint_names))
            for i, name in enumerate(urdf_joint_names):
                if name in joints:
                    cfg[i] = joints[name]
            viser_urdf.update_cfg(cfg)

    for control in (
        side,
        mode,
        style,
        point_size,
        dexterity_pct,
        shell,
        min_score,
        theta_lo,
        theta_hi,
    ):
        control.on_update(refresh_volume)
    for control in (side, show_yaw_slice, yaw_slice, show_z_slice, z_slice):
        control.on_update(refresh_slices)
    ik_enabled.on_update(refresh_ik)
    side.on_update(lambda _: solve_ik())

    refresh_volume()
    refresh_slices()
    logger.info(f"Reachability viewer: http://localhost:{port}")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def _plane_wxyz(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion placing an image plane vertically: local x → yaw direction
    in the ground plane, local y → world +z."""
    from scipy.spatial.transform import Rotation

    matrix = np.array(
        [
            [np.cos(yaw), 0.0, np.sin(yaw)],
            [np.sin(yaw), 0.0, -np.cos(yaw)],
            [0.0, 1.0, 0.0],
        ]
    )
    x, y, z, w = Rotation.from_matrix(matrix).as_quat()
    return (w, x, y, z)


def cli_main() -> None:
    parser = argparse.ArgumentParser(description="Interactive capability-map viewer (viser).")
    parser.add_argument(
        "--map", type=Path, action="append", required=True, help="capability .npz (repeatable)"
    )
    parser.add_argument("--port", type=int, default=8082)
    args = parser.parse_args()

    maps = {}
    for path in args.map:
        cap = CapabilityMap.load(path)
        maps[f"{cap.side} ({path.name})"] = cap
    serve(maps, port=args.port)


if __name__ == "__main__":
    cli_main()


__all__ = [
    "ArmIK",
    "body_voxel_mesh",
    "canonical_cloud",
    "position_cloud",
    "score_colors",
    "serve",
    "slice_image_height",
    "slice_image_yaw",
]
