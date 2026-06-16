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

"""PimSim — the usage API.

One facade for talking to a running PimSim, the same way you'd script a real
robot. Connect, place/move the robot, edit the scene:

    sim = PimSim()                 # connects to the running viewer + LCM bus
    sim.set_agent_position(1, 2)   # place the robot
    sim.cmd_vel(vx=0.5)            # drive it (open-loop velocity)
    sim.goto(10.9, 0.6)           # or send a nav goal (closed-loop)
    sim.add_wall(7, -2.5, 7, 3.5)  # author scene geometry
    sim.add_box((0, 1, 0.5))       # drop a collidable box

The robot itself comes from the **blueprint** you launched
(``dimos --simulation <mujoco|pimsim> --scene <name> run <blueprint>``); see
``add_robot`` for why adding one differs between backends.

This grows ``PimSimClient`` (the e2e ``SceneControl`` surface) into the full
API; the test fixture keeps working unchanged.

Backend note — every method here talks to whatever sim is running over the same
WS + LCM surface, so the *call* is backend-blind. The one place backends
genuinely differ is **adding a robot** (see ``add_robot`` / ``set_embodiment``).
"""

from __future__ import annotations

from dimos.experimental.pimsim.client import DEFAULT_URL, PimSimClient
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Backends that own a robot at launch (the robot is welded into the model and
# driven by the ControlCoordinator) — runtime add_robot is impossible; relaunch
# the blueprint. Browser/Havok backends spawn the robot as a kinematic avatar
# and CAN add/swap it at runtime.
_LAUNCH_TIME_ROBOT_BACKENDS = frozenset({"mujoco"})


class PimSim(PimSimClient):
    """Script a running PimSim — place/move the robot and edit the scene.

    ``backend`` lets the facade give correct guidance for the one backend-
    specific operation (``add_robot``); everything else is identical regardless.
    """

    def __init__(self, url: str | None = None, *, backend: str = "pimsim") -> None:
        super().__init__(url if url is not None else DEFAULT_URL)
        self._backend = backend
        self._cmd_vel = None  # lazily created LCM publisher

    # ── movement ──────────────────────────────────────────────────────────
    # set_agent_position(x, y, z)   — inherited; teleport the robot (setup).
    # publish_goal(x, y)            — inherited; nav goal alias is goto().

    def goto(self, x: float, y: float) -> None:
        """Send a navigation goal (closed-loop — the robot's stack plans to it)."""
        self.publish_goal(x, y)

    def cmd_vel(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        """Open-loop base velocity command (m/s, rad/s) — the same ``/cmd_vel``
        a teleop or nav stack publishes. Holds until the next command."""
        if self._cmd_vel is None:
            from dimos.core.transport import LCMTransport

            self._cmd_vel = LCMTransport("/cmd_vel", Twist)
            self._cmd_vel.start()
        self._cmd_vel.publish(Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz)))

    # ── scene editing ─────────────────────────────────────────────────────
    # add_wall(x1, y1, x2, y2)  — inherited; collidable static wall.
    # clear_entities()          — inherited; despawn everything.

    def add_box(self, point: tuple[float, float, float]) -> None:
        """Drop a collidable box at a world point (the viewer's quick obstacle)."""
        x, y, z = point
        self._send({"type": "entity_test_add", "point": [float(x), float(y), float(z)]})

    def add_object(
        self,
        object_id: str,
        *,
        shape: str = "box",
        extents: tuple[float, ...] = (0.1, 0.1, 0.1),
        mesh_ref: str = "",
        at: tuple[float, float, float] = (0.0, 0.0, 0.0),
        kind: str = "dynamic",
        mass: float = 0.0,
        rgba: tuple[float, float, float, float] | None = None,
    ) -> None:
        """Spawn an entity in the scene (the generic ``entity_spawn`` command).

        Primitives (``shape`` = box | sphere | cylinder + ``extents``) work
        immediately. A ``mesh_ref`` GLB requires that asset to be served by the
        viewer (asset upload is a follow-on). ``kind="dynamic"`` is physics-
        driven; ``"static"`` / ``"kinematic"`` are not.
        """
        from dimos.experimental.pimsim.entity import EntityDescriptor, pose_to_wire
        from dimos.msgs.geometry_msgs.Pose import Pose

        descriptor = EntityDescriptor(
            entity_id=object_id,
            kind=kind,
            mesh_ref=mesh_ref,
            shape_hint="mesh" if mesh_ref else shape,
            extents=tuple(extents),
            mass=mass,
            rgba=rgba,
        )
        x, y, z = at
        self._send(
            {
                "type": "entity_spawn",
                "descriptor": descriptor.to_wire(),
                "pose": pose_to_wire(Pose(float(x), float(y), float(z))),
            }
        )

    def add_npc(self, name: str, path: list[tuple[float, float]]) -> None:
        """Spawn a moving character that follows ``path`` (person-follow evals).
        NOT yet wired — needs an ``entity_spawn`` + per-tick ``entity_set_pose``/
        ``apply_entity_velocity`` driver on the viewer."""
        raise NotImplementedError("add_npc needs a moving-entity driver on the viewer")

    # ── robots ────────────────────────────────────────────────────────────

    def add_robot(self, name: str, at: tuple[float, float, float] = (0, 0, 0)) -> None:
        """Add a robot embodiment to the running scene.

        **Backend-dependent — this is the one place sims genuinely differ:**

        * **Browser / Havok** (``backend="pimsim"``): the robot is a kinematic
          avatar, so it can be spawned/swapped at runtime (DimSim-style). NOT
          yet wired on PimSim's viewer — needs an embodiment command on
          ``BabylonSceneViewerModule`` (DimSim has ``set_embodiment``).
        * **MuJoCo** (``backend="mujoco"``): the robot is welded into the model
          and driven by the ``ControlCoordinator`` — it CANNOT be added at
          runtime. Relaunch with the blueprint that owns ``name``:
          ``dimos --simulation mujoco --scene <scene> run <blueprint>``.
        """
        if self._backend in _LAUNCH_TIME_ROBOT_BACKENDS:
            raise RuntimeError(
                f"MuJoCo robots are composed into the blueprint, not added at "
                f"runtime. Relaunch: dimos --simulation mujoco run <blueprint for {name!r}>"
            )
        raise NotImplementedError(
            "runtime add_robot on the browser backend needs an embodiment WS "
            "command on BabylonSceneViewerModule (cf. DimSim set_embodiment)"
        )

    def set_embodiment(self, preset: str, **kwargs: object) -> None:
        """Define/swap the robot's control embodiment (drone | differential-drive
        | ackermann | custom). Browser-backend runtime feature — see
        ``add_robot`` for the backend split. Not yet wired on PimSim's viewer."""
        raise NotImplementedError(
            "set_embodiment is a browser-backend runtime feature; the viewer "
            "needs an embodiment command (cf. DimSim set_embodiment)"
        )


__all__ = ["PimSim"]
