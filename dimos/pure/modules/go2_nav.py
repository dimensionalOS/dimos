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

"""GO2 + the pure nav graph, deployed together on the coordinator with live rerun.

The definitive "pure modules next to legacy modules" example. Legacy nodes:
:class:`GO2Connection`, the go2 rerun renderer (:func:`vis_module`), and
:class:`RerunWebSocketServer` (clicks). Pure node: the
:class:`~dimos.pure.modules.nav_stack.NavStack` graph. All linked by
``autoconnect``; each pure member deploys in its own worker via the T8 bridge and
the graph's ``blueprint()`` generates the wiring from its ``wire()`` edges (rim
exposed, interior namespaced). From the outside every node is just a module on
the bus::

    GO2Connection ──lidar──▶ NavStack.lidar ─┐
        │  └────odom────────▶ NavStack.odom   ├─▶ voxel ▶ cost ▶ plan ─▶ path
        │                                     │        (odom ▶ tf ▶ plan)
    RerunWebSocketServer ─clicked_point─▶ NavStack.goal_point (remapped)
      vis_module (rerun) ◀── lidar / map / costmap / path (renders the whole bus)

The rerun viewer renders the scene live; a click publishes ``clicked_point`` (a
``PointStamped``), remapped onto the graph's ``goal_point`` rim, which the planner
uses directly. Same blueprint runs two ways:

    # against a recording, no robot (GO2Connection --replay sources its own db):
    python -m dimos.pure.modules.go2_nav --replay go2_hongkong_office

    # against a live robot:
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
from dimos.pure.modules.nav_stack import NavStack
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module


def go2_nav_blueprint(*, voxel_size: float = 0.1, connection: Blueprint | None = None) -> Blueprint:
    """Autoconnect the go2 rerun renderer + GO2Connection + click server + NavStack.

    Links are by the autoconnect (name, type) convention: GO2's ``lidar``/``odom``
    fan into the graph's rim and into the rerun renderer; the click server's
    ``clicked_point`` is remapped onto ``NavStack.goal_point`` (same ``PointStamped``
    type, different name); the graph's interior edges are namespaced by
    ``NavStack.blueprint()`` so they never collide. ``connection`` overrides the GO2
    atom (e.g. an ``ip="replay"`` one); default is a live connection.
    """
    return autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
        connection or GO2Connection.blueprint(),
        RerunWebSocketServer.blueprint(),
        NavStack.blueprint(voxel_size=voxel_size),
    ).remappings([(RerunWebSocketServer, "clicked_point", "goal_point")])


def main() -> None:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--replay", metavar="DB", help="replay db name (no robot), e.g. go2_hongkong_office"
    )
    src.add_argument("--robot-ip", metavar="IP", help="connect a live Go2 at this address")
    parser.add_argument("--voxel-size", type=float, default=0.1)
    args = parser.parse_args()

    if args.replay is not None:
        # GO2Connection replays its own db (ip='replay' + g.replay_db) — opaque to the graph.
        blueprint = go2_nav_blueprint(
            voxel_size=args.voxel_size, connection=GO2Connection.blueprint(ip="replay")
        ).global_config(replay_db=args.replay)
    else:
        blueprint = go2_nav_blueprint(voxel_size=args.voxel_size).global_config(
            robot_ip=args.robot_ip
        )

    coordinator = ModuleCoordinator.build(blueprint)
    coordinator.loop()  # Ctrl+C stops all modules and cleans up


if __name__ == "__main__":
    main()
