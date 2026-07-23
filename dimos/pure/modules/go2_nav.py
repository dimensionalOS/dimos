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

"""GO2 + the pure nav graph — two ways to run the same :class:`NavStack`.

**Deploy mode** (``--replay`` / ``--robot-ip``): the "pure next to legacy" demo.
Legacy nodes (:class:`GO2Connection`, the go2 rerun renderer :func:`vis_module`,
:class:`RerunWebSocketServer`) and the pure :class:`NavStack` graph are joined by
``autoconnect`` and deployed on :class:`ModuleCoordinator` — each pure member in
its own worker via the T8 bridge. This is the **live** path: paced by the robot
(or a wall-clock ``--replay``), lossy under backpressure by design::

    GO2Connection ──lidar──▶ NavStack.lidar ─┐
        │  └────odom────────▶ NavStack.odom   ├─▶ voxel ▶ cost ▶ plan ─▶ path
        │                                     │        (odom ▶ tf ▶ plan)
    RerunWebSocketServer ─clicked_point─▶ NavStack.goal_point (remapped)
      vis_module (rerun) ◀── lidar / map / costmap / path

**Pure eval mode** (``--pure``, needs ``--replay``): the reason pure modules
exist. The recording's mem2 store feeds :meth:`NavStack.over` **in-process** and
the exported map/costmap/path fan into a single pure rerun consumer
(:class:`~dimos.pure.modules.rerunsink.NavRerunSink`) via ``.save`` — the DAG
computes **once** (§0.5), there is **no wall clock in the data path**, so it runs
**as fast as compute allows** (faster than realtime) while the rerun viewer updates
live. mem2 db ──▶ NavStack (PureGraph) ──▶ pure rerun sink.

Run::

    # faster-than-realtime in-process eval, live rerun (no robot, no coordinator):
    python -m dimos.pure.modules.go2_nav --replay go2_hongkong_office --pure

    # realtime coordinator deploy against a recording (no robot):
    python -m dimos.pure.modules.go2_nav --replay go2_hongkong_office

    # realtime coordinator deploy against a live robot:
    python -m dimos.pure.modules.go2_nav --robot-ip 192.168.1.42

The command return path (``cmd_vel``) needs a controller stage plus
:class:`~dimos.pure.modules.translators.TwistUnstamp` (a pure planner emits
``TwistStamped``, GO2 consumes ``Twist``) — left open here since ``NavStack`` is a
sensing/planning graph, not a driver.
"""

from __future__ import annotations

import argparse

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.pure.modules.nav_stack import NavStack, NavStackPGO
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

WORLD = "world"


def go2_nav_blueprint(
    *, voxel_size: float = 0.1, pgo: bool = False, connection: Blueprint | None = None
) -> Blueprint:
    """Autoconnect the go2 rerun renderer + GO2Connection + click server + NavStack.

    Links are by the autoconnect (name, type) convention: GO2's ``lidar``/``odom``
    fan into the graph's rim and into the rerun renderer; the click server's
    ``clicked_point`` is remapped onto ``NavStack.goal_point`` (same ``PointStamped``
    type, different name); the graph's interior edges are namespaced by
    ``NavStack.blueprint()`` so they never collide. ``connection`` overrides the GO2
    atom (e.g. an ``ip="replay"`` one); default is a live connection.

    ``NavStack`` exports its costmap as ``costmap`` (topic ``/costmap``), but the go2
    rerun renderer subscribes it under the legacy name ``global_costmap`` — remapped
    here so the two meet, same as ``clicked_point``.
    """
    return autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
        connection or GO2Connection.blueprint(),
        RerunWebSocketServer.blueprint(),
        (NavStackPGO if pgo else NavStack).blueprint(voxel_size=voxel_size),
    ).remappings(
        [
            (RerunWebSocketServer, "clicked_point", "goal_point"),
            (WebsocketVisModule, "global_costmap", "costmap"),
        ]
    )


def run_pure(
    db_name: str, *, voxel_size: float = 0.1, pgo: bool = False, sink: str = "spawn"
) -> int:
    """Faster-than-realtime in-process eval: mem2 db ▶ NavStack.over ▶ pure rerun sink.

    The recording's ``lidar``/``odom`` streams drive :meth:`NavStack.over`; a static
    origin goal lets the planner emit from the first tick. ``.save`` computes the DAG
    once and fans map/costmap/path into the rerun consumer at their native rates.
    Returns the number of rendered frames.
    """
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.PointStamped import PointStamped
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.pure.modules.rerunsink import NavRerunSink
    from dimos.utils.data import get_data

    goal = PointStamped(x=0.0, y=0.0, z=0.0, frame_id=WORLD)
    goal.ts = 1.0  # below every recording ts, so the planner's latest() sees it from tick 1

    with SqliteStore(path=str(get_data(f"{db_name}.db"))) as store:
        run = (NavStackPGO if pgo else NavStack)(voxel_size=voxel_size).over(
            lidar=store.stream("lidar", PointCloud2),
            odom=store.stream("odom", PoseStamped),
            goal_point=[goal],
        )
        n: int = run.save(NavRerunSink(sink=sink))
    print(f"pure eval of {db_name}: rendered {n} frames")
    return n


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--replay", metavar="DB", help="replay db name (no robot), e.g. go2_hongkong_office"
    )
    src.add_argument("--robot-ip", metavar="IP", help="connect a live Go2 at this address")
    parser.add_argument(
        "--pure",
        action="store_true",
        help="faster-than-realtime in-process eval (needs --replay): NavStack.over ▶ rerun sink",
    )
    parser.add_argument("--voxel-size", type=float, default=0.1)
    parser.add_argument(
        "--pgo",
        action="store_true",
        help="map with PGOVoxelMapper: loop-closure-corrected keyframe map",
    )
    parser.add_argument(
        "--sink", choices=("grpc", "spawn"), default="spawn", help="rerun sink for --pure"
    )
    args = parser.parse_args()

    if args.pure and args.replay is None:
        parser.error("--pure needs a recording: pass --replay DB")

    # Mint a counter-named per-run dir BEFORE building anything, so main.jsonl +
    # debug.db land in LOG_DIR/NNN_<mode>/ (never a shared logs/debug.db). T16.
    from dimos.utils.rundir import mint_run_dir

    mint_run_dir("go2-nav-pure" if args.pure else "go2-nav")

    if args.pure:
        run_pure(args.replay, voxel_size=args.voxel_size, pgo=args.pgo, sink=args.sink)
        return

    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    if args.replay is not None:
        # GO2Connection replays its own db (ip='replay' + g.replay_db) — opaque to the graph.
        blueprint = go2_nav_blueprint(
            voxel_size=args.voxel_size,
            pgo=args.pgo,
            connection=GO2Connection.blueprint(ip="replay"),
        ).global_config(replay_db=args.replay)
    else:
        blueprint = go2_nav_blueprint(voxel_size=args.voxel_size, pgo=args.pgo).global_config(
            robot_ip=args.robot_ip
        )

    coordinator = ModuleCoordinator.build(blueprint)
    coordinator.loop()  # Ctrl+C stops all modules and cleans up


if __name__ == "__main__":
    main()
