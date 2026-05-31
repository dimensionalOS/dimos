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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License").

"""Render Go2 odom sources to rerun — memory2 store pipelines (standalone).

Each *pipeline* is a function ``(store, seconds) -> None`` composed from
reusable stream transforms over standard dimos messages. ``leg_odom`` logs both
the per-frame pose (Transform3D) and the accumulated trajectory (nav_msgs/Path)::

    sportmodestate.map_data(pose).tap(log_pose)      # moving frame, full rate
        .transform(throttle(0.1)).transform(accumulate_path).tap(log_path)   # growing path

Standalone — not wired into the dimos CLI:

    uv run python -m dimos.robot.unitree.go2dds.cli.render \
        go2_china_office_indoor.mcap --seconds 120
"""

from __future__ import annotations

from collections.abc import Callable
import shutil
import subprocess
from typing import TYPE_CHECKING, Any

import numpy as np
import typer

from dimos.memory2.transform import throttle
from dimos.memory2.utils.progress import progress
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.robot.unitree.go2dds.extrinsics import EXT_R, EXT_T
from dimos.robot.unitree.go2dds.msgs.SportModeState import SportModeState
from dimos.robot.unitree.go2dds.store import Go2McapStore

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

GRAVITY = np.array([0.0, 0.0, 9.81])
WORLD = "world"


# --- transforms over standard msgs (map / scan / reduce / tap) ---------------
def sportmode_pose(obs: Observation[SportModeState]) -> PoseStamped:
    """map_data: SportModeState -> PoseStamped (leg-inertial pose)."""
    sm = obs.data
    w, x, y, z = (float(v) for v in sm.imu_state.quaternion)  # Unitree order: wxyz
    return PoseStamped(
        ts=obs.ts,
        frame_id=WORLD,
        position=[float(v) for v in sm.position],
        orientation=[x, y, z, w],
    )


def integrate_velocity(state: Any, obs: Observation[Any]) -> tuple[Any, TwistStamped]:
    """scan_data: Imu -> TwistStamped (world accel integrated to velocity)."""
    vel, prev = state
    la = obs.data.linear_acceleration
    a = obs.data.orientation.rotate_vector(Vector3(la.x, la.y, la.z))
    a_world = np.array([a.x, a.y, a.z]) - GRAVITY
    if prev is not None:
        vel = vel + a_world * (obs.ts - prev)
    twist = TwistStamped(ts=obs.ts, frame_id=WORLD, linear=vel.tolist(), angular=[0.0, 0.0, 0.0])
    return (vel, obs.ts), twist


def integrate_position(state: Any, obs: Observation[Any]) -> tuple[Any, PoseStamped]:
    """scan_data: TwistStamped -> PoseStamped (velocity integrated to position)."""
    pos, prev = state
    v = obs.data.linear
    if prev is not None:
        pos = pos + np.array([v.x, v.y, v.z]) * (obs.ts - prev)
    pose = PoseStamped(
        ts=obs.ts, frame_id=WORLD, position=pos.tolist(), orientation=[0.0, 0.0, 0.0, 1.0]
    )
    return (pos, obs.ts), pose


def accumulate_path(upstream: Iterator[Observation[PoseStamped]]) -> Iterator[Observation[Path]]:
    """transform: yield the growing nav_msgs/Path as each pose streams in."""
    path = Path(frame_id=WORLD)
    for obs in upstream:
        path = path.push(obs.data)
        yield obs.derive(data=path)


# --- pipelines: (store, color, seconds) -> None ------------------------------
def leg_odom(store: Go2McapStore, seconds: float | None) -> None:
    """Leg-inertial odometry — pose stream (Transform3D) + accumulated Path line."""
    import rerun as rr

    def log_pose(obs: Observation[PoseStamped]) -> None:
        rr.set_time("time", timestamp=obs.ts)
        # Transform3D carries the pose; TransformAxes3D draws it as a visible gizmo.
        rr.log("world/leg_odom", obs.data.to_rerun(), rr.TransformAxes3D(axis_length=0.2))

    def log_path(obs: Observation[Path]) -> None:
        rr.set_time("time", timestamp=obs.ts)
        rr.log("world/leg_odom_path", obs.data.to_rerun())

    src = store.streams.sportmodestate.to_time(seconds)
    (
        src.tap(progress(src.count(), "leg_odom"))
        .map_data(sportmode_pose)
        .tap(log_pose)
        .transform(throttle(0.1))  # reduce_rate: thin the path to ~10 Hz
        .transform(accumulate_path)  # yield the growing path each step
        .tap(log_path)
        .drain()
    )


def lidar(store: Go2McapStore, seconds: float | None) -> None:
    """Lidar point cloud, under the leg_odom transform (lidar -> base -> world)."""
    import rerun as rr

    # Static lidar->base extrinsic (the L1 is mounted ~upside-down). Parent entity
    # world/leg_odom carries base->world, so rerun composes the full chain.
    rr.log("world/leg_odom/lidar", rr.Transform3D(translation=EXT_T, mat3x3=EXT_R), static=True)
    src = store.streams.lidar.to_time(seconds)
    for obs in src.tap(progress(src.count(), "lidar")):
        rr.set_time("time", timestamp=obs.ts)
        rr.log("world/leg_odom/lidar", obs.data.to_rerun())


# Add a source: write a (store, seconds) -> None function and append it.
PIPELINES: list[Callable[[Go2McapStore, float | None], None]] = [
    leg_odom,
    lidar,
]


def main(
    mcap: str = typer.Argument(..., help="Go2 .mcap (path or data-dir name)"),
    out: str = typer.Option("go2_odom.rrd", "--out", help="Output .rrd"),
    seconds: float = typer.Option(None, "--seconds", help="Only the first N seconds"),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't open the viewer"),
) -> None:
    import rerun as rr

    from dimos.visualization.rerun.init import rerun_init

    store = Go2McapStore(path=mcap)
    rerun_init("go2_odom")  # registers the turbo height colormap for PointCloud2.to_rerun
    rr.save(out)
    for pipeline in PIPELINES:
        pipeline(store, seconds)
    rr.rerun_shutdown()
    print(f"wrote {out}")
    if not no_gui:
        exe = shutil.which("rerun")
        if exe:
            subprocess.Popen([exe, out])
        else:
            print(f"  rerun viewer not on PATH; open manually:\n    rerun {out}")


if __name__ == "__main__":
    typer.run(main)
    typer.run(main)
    typer.run(main)
