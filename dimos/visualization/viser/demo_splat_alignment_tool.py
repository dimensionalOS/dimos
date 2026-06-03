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

"""Standalone mesh + splat alignment tool.

Loads a mesh and a Gaussian splat into a viser scene, exposes 7 live controls
for the splat transform (x/y/z, rz/ry/rx, scale), and writes the current
alignment to YAML on demand.

Typical usage:

    /Users/pimvandenbosch/Desktop/dimos/.venv/bin/python \
      -m dimos.visualization.viser.demo_splat_alignment_tool \
      --mesh /path/to/mesh.glb \
      --splat /path/to/scene.ply \
      --out /tmp/alignment.yaml
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

from dimos.simulation.scene_assets.mesh_scene import (
    SceneMeshAlignment,
    load_scene_mesh,
)
from dimos.visualization.viser.splat import SplatAlignment, load_splat
import viser


@dataclass
class _AlignmentState:
    x: float
    y: float
    z: float
    rz_deg: float
    ry_deg: float
    rx_deg: float
    scale: float
    y_up: bool

    @classmethod
    def from_alignment(cls, alignment: SplatAlignment) -> _AlignmentState:
        return cls(
            x=float(alignment.translation[0]),
            y=float(alignment.translation[1]),
            z=float(alignment.translation[2]),
            rz_deg=float(alignment.rotation_zyx_deg[0]),
            ry_deg=float(alignment.rotation_zyx_deg[1]),
            rx_deg=float(alignment.rotation_zyx_deg[2]),
            scale=float(alignment.scale),
            y_up=bool(alignment.y_up),
        )

    def to_alignment(self) -> SplatAlignment:
        return SplatAlignment(
            scale=float(self.scale),
            translation=(float(self.x), float(self.y), float(self.z)),
            rotation_zyx_deg=(
                float(self.rz_deg),
                float(self.ry_deg),
                float(self.rx_deg),
            ),
            y_up=bool(self.y_up),
        )

    def to_yaml_text(self) -> str:
        return (
            f"scale: {self.scale:.8f}\n"
            f"translation: [{self.x:.6f}, {self.y:.6f}, {self.z:.6f}]\n"
            f"rotation_zyx: [{self.rz_deg:.6f}, {self.ry_deg:.6f}, {self.rx_deg:.6f}]\n"
            f"y_up: {str(self.y_up).lower()}\n"
        )


def _load_initial_alignment(path: Path | None, y_up: bool) -> _AlignmentState:
    if path is None or not path.exists():
        return _AlignmentState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, y_up=y_up)
    return _AlignmentState.from_alignment(SplatAlignment.from_yaml(path))


def _set_splat_pose(handle: Any, state: _AlignmentState) -> None:
    handle.position = (state.x, state.y, state.z)
    handle.scale = state.scale
    handle.wxyz = tuple(
        float(v)
        for v in R.from_euler(
            "ZYX", [state.rz_deg, state.ry_deg, state.rx_deg], degrees=True
        ).as_quat(scalar_first=True)
    )


def _compose_mesh_wxyz(
    *,
    y_up: bool,
    rotation_zyx_deg: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    rotation_matrix = np.eye(3, dtype=np.float64)
    if y_up:
        rotation_matrix = np.array(
            [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
            dtype=np.float64,
        )
    rz, ry, rx = (np.deg2rad(angle) for angle in rotation_zyx_deg)
    cz, sz = np.cos(rz), np.sin(rz)
    cy, sy = np.cos(ry), np.sin(ry)
    cx, sx = np.cos(rx), np.sin(rx)
    rotation_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    rotation_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rotation_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    rotation_matrix = rotation_z @ rotation_y @ rotation_x @ rotation_matrix
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, rotation_matrix.flatten())
    return (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))


def _add_mesh(
    server: viser.ViserServer,
    mesh_path: Path,
    mesh_alignment: SceneMeshAlignment,
) -> Any:
    suffix = mesh_path.suffix.lower()
    if suffix in {".glb", ".gltf"}:
        with open(mesh_path, "rb") as f:
            glb_bytes = f.read()
        return server.scene.add_glb(
            "/mesh",
            glb_data=glb_bytes,
            scale=float(mesh_alignment.scale),
            wxyz=_compose_mesh_wxyz(
                y_up=mesh_alignment.y_up,
                rotation_zyx_deg=mesh_alignment.rotation_zyx_deg,
            ),
            position=tuple(float(v) for v in mesh_alignment.translation),
        )

    mesh = load_scene_mesh(mesh_path, alignment=mesh_alignment)
    return server.scene.add_mesh_simple(
        "/mesh",
        vertices=np.asarray(mesh.vertices, dtype=np.float32),
        faces=np.asarray(mesh.triangles, dtype=np.int32),
        color=(180, 180, 180),
        opacity=1.0,
    )


def _compute_opacity_keep_mask(
    splat_path: Path,
    opacity_threshold: float | None,
) -> np.ndarray | None:
    if opacity_threshold is None:
        return None
    from plyfile import PlyData

    vertices = PlyData.read(str(splat_path))["vertex"]
    keep = vertices["opacity"].astype(np.float32) > float(opacity_threshold)
    print(
        f"Applied opacity filter > {opacity_threshold}: "
        f"{int(keep.sum())}/{len(keep)} gaussians kept"
    )
    return keep


def _apply_keep_mask(splat: Any, keep_mask: np.ndarray | None) -> Any:
    if keep_mask is None:
        return splat
    splat.centers = splat.centers[keep_mask]
    splat.covariances = splat.covariances[keep_mask]
    splat.rgbs = splat.rgbs[keep_mask]
    splat.opacities = splat.opacities[keep_mask]
    return splat


def _load_preview_splat(
    splat_path: Path,
    *,
    y_up: bool,
    keep_mask: np.ndarray | None,
) -> Any:
    return _apply_keep_mask(
        load_splat(splat_path, alignment=SplatAlignment(y_up=y_up)),
        keep_mask,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--mesh", type=Path, required=True, help="Mesh asset (.glb/.gltf/.obj/.ply/...)"
    )
    parser.add_argument("--splat", type=Path, required=True, help="Gaussian splat .ply")
    parser.add_argument(
        "--alignment",
        type=Path,
        default=None,
        help="Optional existing splat alignment YAML to initialize the controls.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("/tmp/splat_alignment.yaml"),
        help="Where the Save button writes the current alignment YAML.",
    )
    parser.add_argument("--port", type=int, default=8093, help="Viser port.")
    parser.add_argument(
        "--mesh-scale",
        type=float,
        default=1.0,
        help="Static scale for the mesh.",
    )
    parser.add_argument(
        "--mesh-translation",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="Static translation for the mesh.",
    )
    parser.add_argument(
        "--mesh-rotation-zyx",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("RZ", "RY", "RX"),
        help="Static mesh yaw/pitch/roll in degrees.",
    )
    parser.add_argument(
        "--mesh-y-up",
        action="store_true",
        help="Apply the standard Y-up -> Z-up swap to the mesh.",
    )
    parser.add_argument(
        "--y-up",
        action="store_true",
        help="Initialize the splat alignment with y_up=true.",
    )
    parser.add_argument(
        "--opacity-threshold",
        type=float,
        default=None,
        help="Optional raw 3DGS opacity-logit threshold for preview filtering.",
    )
    args = parser.parse_args()

    if not args.mesh.exists():
        raise SystemExit(f"mesh not found: {args.mesh}")
    if not args.splat.exists():
        raise SystemExit(f"splat not found: {args.splat}")

    mesh_alignment = SceneMeshAlignment(
        scale=float(args.mesh_scale),
        translation=tuple(float(v) for v in args.mesh_translation),
        rotation_zyx_deg=tuple(float(v) for v in args.mesh_rotation_zyx),
        y_up=bool(args.mesh_y_up),
    )
    state = _load_initial_alignment(args.alignment, y_up=bool(args.y_up))
    initial_state = _AlignmentState.from_alignment(state.to_alignment())
    keep_mask = _compute_opacity_keep_mask(args.splat, args.opacity_threshold)

    print(f"Loading mesh: {args.mesh}")
    print(f"Loading splat: {args.splat}")
    splat = _load_preview_splat(args.splat, y_up=state.y_up, keep_mask=keep_mask)

    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.gui.set_panel_label("Splat Alignment")
    server.gui.configure_theme(
        control_layout="collapsible",
        show_logo=False,
        show_share_button=False,
        dark_mode=True,
    )
    server.scene.set_up_direction("+z")
    server.scene.world_axes.visible = True

    mesh_handle = _add_mesh(server, args.mesh, mesh_alignment)
    splat_handle = server.scene.add_gaussian_splats(
        "/splat",
        centers=splat.centers,
        covariances=splat.covariances,
        rgbs=splat.rgbs,
        opacities=splat.opacities,
    )
    _set_splat_pose(splat_handle, state)

    with server.gui.add_folder("Splat Transform"):
        x = server.gui.add_number("X", initial_value=state.x, step=0.01)
        y = server.gui.add_number("Y", initial_value=state.y, step=0.01)
        z = server.gui.add_number("Z", initial_value=state.z, step=0.01)
        rz = server.gui.add_number("RZ (deg)", initial_value=state.rz_deg, step=0.1)
        ry = server.gui.add_number("RY (deg)", initial_value=state.ry_deg, step=0.1)
        rx = server.gui.add_number("RX (deg)", initial_value=state.rx_deg, step=0.1)
        scale = server.gui.add_number("Scale", initial_value=state.scale, step=0.001)
        y_up = server.gui.add_checkbox("Y-Up Pre-Rotate", initial_value=state.y_up)

    with server.gui.add_folder("Display"):
        show_mesh = server.gui.add_checkbox("Show Mesh", initial_value=True)
        show_splat = server.gui.add_checkbox("Show Splat", initial_value=True)

    with server.gui.add_folder("Save"):
        out_path = server.gui.add_text("Output Path", initial_value=str(args.out))
        yaml_preview = server.gui.add_markdown(f"```yaml\n{state.to_yaml_text()}```")
        status = server.gui.add_markdown("_Idle._")
        reset_button = server.gui.add_button("Reset To Initial")
        save_button = server.gui.add_button("Save Alignment YAML")
        print_button = server.gui.add_button("Print Alignment To Terminal")

    def _current_state() -> _AlignmentState:
        return _AlignmentState(
            x=float(x.value),
            y=float(y.value),
            z=float(z.value),
            rz_deg=float(rz.value),
            ry_deg=float(ry.value),
            rx_deg=float(rx.value),
            scale=float(scale.value),
            y_up=bool(y_up.value),
        )

    loaded_y_up = [state.y_up]

    def _refresh_alignment() -> None:
        current = _current_state()
        yaml_preview.content = f"```yaml\n{current.to_yaml_text()}```"

        # Y-up is baked into load_splat's pre-rotation, so changing it requires
        # rebuilding the handle from raw data. The 7 tuned values remain on the
        # handle transform itself and update instantly.
        if current.y_up != loaded_y_up[0]:
            loaded_y_up[0] = current.y_up
            rebuilt = _load_preview_splat(
                args.splat,
                y_up=current.y_up,
                keep_mask=keep_mask,
            )
            nonlocal_splat_handle[0].remove()
            nonlocal_splat_handle[0] = server.scene.add_gaussian_splats(
                "/splat",
                centers=rebuilt.centers,
                covariances=rebuilt.covariances,
                rgbs=rebuilt.rgbs,
                opacities=rebuilt.opacities,
            )
            _set_splat_pose(nonlocal_splat_handle[0], current)
            nonlocal_splat_handle[0].visible = bool(show_splat.value)
            return

        _set_splat_pose(nonlocal_splat_handle[0], current)

    nonlocal_splat_handle = [splat_handle]

    for control in (x, y, z, rz, ry, rx, scale, y_up):
        control.on_update(lambda _event: _refresh_alignment())

    @show_mesh.on_update
    def _on_mesh_toggle(_event: Any) -> None:
        mesh_handle.visible = bool(show_mesh.value)

    @show_splat.on_update
    def _on_splat_toggle(_event: Any) -> None:
        nonlocal_splat_handle[0].visible = bool(show_splat.value)

    @reset_button.on_click
    def _on_reset(_event: Any) -> None:
        x.value = initial_state.x
        y.value = initial_state.y
        z.value = initial_state.z
        rz.value = initial_state.rz_deg
        ry.value = initial_state.ry_deg
        rx.value = initial_state.rx_deg
        scale.value = initial_state.scale
        y_up.value = initial_state.y_up
        status.content = "_Reset to the loaded alignment._"
        _refresh_alignment()

    @save_button.on_click
    def _on_save(_event: Any) -> None:
        current = _current_state()
        path = Path(str(out_path.value)).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(current.to_yaml_text())
        status.content = f"_Saved `{path}`._"

    @print_button.on_click
    def _on_print(_event: Any) -> None:
        current = _current_state()
        print("\nCurrent alignment:\n" + current.to_yaml_text(), flush=True)
        status.content = "_Printed alignment to terminal._"

    print(f"Alignment tool: http://localhost:{args.port}/")
    print(f"Saving YAML to: {args.out}")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
