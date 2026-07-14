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

import contextlib
import json
import time
from typing import Any

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.dimsim.scene_client import SceneClient


class DimSimClient:
    _client: SceneClient | None = None

    def __init__(self) -> None:
        self._client = None
        self._goal_request: LCMTransport[PoseStamped] = LCMTransport("/goal_request", PoseStamped)

    def start(self) -> None:
        # self.client should be started lazily to avoid starting the dimsim
        # process before pytest fixtures are ready
        self._goal_request.start()

    def stop(self) -> None:
        self.client.stop()
        self._goal_request.stop()

    @property
    def client(self) -> SceneClient:
        if self._client is None:
            # assign only after a successful start, so a refused connection
            # (sim still booting) can simply be retried by the caller
            client = SceneClient()
            client.start()
            self._client = client
        return self._client

    def wait_for_scene(self, timeout: float = 180.0, poll_s: float = 3.0) -> None:
        """Block until the DimSim browser sandbox answers a trivial exec.

        The MCP-ready LCM topic can fire before (or, on a shared bus, from a
        different robot than) the DimSim browser being ready — poll the scene
        itself before driving NPCs.
        """
        deadline = time.monotonic() + timeout
        last_err: Exception | None = None
        while time.monotonic() < deadline:
            try:
                if self.client.exec("return 1;", timeout=5.0) == 1:
                    return
            except Exception as e:  # connection refused / exec timeout while booting
                last_err = e
                # Stop before dropping: a client that connected but timed out
                # on exec owns a live websocket + recv thread — abandoning the
                # reference would leak one per poll (~60 over a 3-min boot).
                if self._client is not None:
                    with contextlib.suppress(Exception):
                        self._client.stop()
                self._client = None
            time.sleep(poll_s)
        raise TimeoutError(f"DimSim scene not ready after {timeout}s: {last_err}")

    def set_agent_position(self, x: float, y: float, z: float = 0.52) -> None:
        self.client.set_agent_position(y, z, x)

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None:
        self.client.add_wall(y1, x1, y2, x2)

    def publish_goal(self, x: float, y: float) -> None:
        self._goal_request.publish(
            PoseStamped(
                position=(x, y, 0),
                orientation=(0, 0, 0, 1),
                frame_id="world",
            )
        )

    # ------------------------------------------------------------------ #
    # Human NPC helpers
    #
    # Robot/map frame (x, y) on the floor maps to Three.js (x=y, y=up=0, z=x),
    # the same swap used by set_agent_position/add_wall above. Callers stay in
    # robot-frame coordinates; the swap lives here.
    # ------------------------------------------------------------------ #

    def add_human(
        self,
        name: str,
        x: float,
        y: float,
        *,
        model_url: str | None = "/person/person.glb",
        color: int = 0xE0A070,
        height: float = 1.7,
        radius: float = 0.25,
    ) -> dict[str, Any]:
        """Spawn a named human actor at robot-frame ``(x, y)``.

        Loads the real scanned-person GLB (``model_url``) via ``add_npc`` with
        ``collider=False`` — the earlier NPC hang was the trimesh-collider +
        physics-snapshot path, and a visual-only actor is exactly right for
        camera/CV work. If the model fails to load (or ``model_url`` is None),
        falls back to a colored human-sized cylinder via the synchronous
        ``add_object`` path; the returned dict then has ``fallback: True``.
        Movement/readback are identical either way (``getObjectByName``).
        """
        if model_url:
            old_timeout = self.client.timeout
            try:
                self.client.timeout = 120.0  # first GLTF load can be slow headless
                result = self.client.add_npc(
                    url=model_url,
                    name=name,
                    position=(y, 0.0, x),
                    animation=0,
                    collider=False,
                )
                result["fallback"] = False
                return result
            except Exception:  # load failure/timeout — fall through to cylinder
                pass
            finally:
                self.client.timeout = old_timeout
        result = self.client.add_object(
            geometry="cylinder",
            size=(radius, radius, height),
            color=color,
            position=(y, height / 2.0, x),  # center at half height → feet on floor
            name=name,
            dynamic=False,
            collider=None,
        )
        result["fallback"] = True
        return result

    def move_human(self, name: str, x: float, y: float) -> None:
        """Teleport a spawned actor to robot-frame ``(x, y)`` (keeps its floor height)."""
        self.client.exec(
            f"const o = scene.getObjectByName({json.dumps(name)});"
            f"if (o) {{ o.position.x = {y}; o.position.z = {x}; }}"
            "return true;"
        )

    def get_human_position(self, name: str) -> tuple[float, float]:
        """Read a spawned NPC's position back in robot-frame ``(x, y)``."""
        pos = self.client.exec(
            f"const o = scene.getObjectByName({json.dumps(name)});"
            "return o ? { x: o.position.x, z: o.position.z } : null;"
        )
        if not pos:
            raise KeyError(f"NPC not found in scene: {name}")
        # invert the swap: robot_x = threejs_z, robot_y = threejs_x
        return float(pos["z"]), float(pos["x"])

    def remove_human(self, name: str) -> bool:
        """Remove a spawned human by name (NPC or fallback cylinder).

        Fallback humans (see add_human) are plain meshes created via
        add_object, which the NPC registry doesn't know — remove_npc returns
        False for them, so fall through to removing the mesh by name.
        Otherwise ghost 'humans' persist in the shared sim across tests.
        """
        if self.client.remove_npc(name):
            return True
        removed = self.client.exec(
            f"const o = scene.getObjectByName({json.dumps(name)});"
            "if (o) { scene.remove(o); return true; }"
            "return false;"
        )
        return bool(removed)

    def add_prop(
        self,
        name: str,
        x: float,
        y: float,
        *,
        geometry: str = "box",
        size: tuple[float, ...] = (1.0, 1.0, 1.0),
        color: int = 0x888888,
        up: float = 0.5,
    ) -> dict[str, Any]:
        """Add a static prop (furniture stand-in) at robot-frame ``(x, y)``.

        ``up`` is the Three.js height of the prop's center (so its base rests on
        the floor). ``size`` follows the ``add_object`` convention per geometry.
        """
        return self.client.add_object(
            geometry=geometry,
            size=size,
            color=color,
            position=(y, up, x),
            name=name,
            dynamic=False,
            collider="box",
        )

    def remove_prop(self, name: str) -> bool:
        """Remove a spawned prop by name."""
        return self.client.remove_object(name)

    # Scene objects from the apt world get runtime names like
    # "assetPrim:<uid>:bed-mattress-core" / "prim:front-yard-floor", so anchor
    # lookups match by exact name OR by ":<anchor>" suffix.
    _FIND_JS = (
        "let found = null;"
        "scene.traverse(o => {{"
        "  if (!found && o.name && (o.name === {name} || o.name.endsWith(':' + {name}))) "
        "found = o;"
        "}});"
    )

    def get_object_position(self, name: str) -> tuple[float, float]:
        """World robot-frame (x, y) of a named scene object (e.g. furniture).

        Matches exact names or the part suffix of prefixed runtime names, and
        uses ``getWorldPosition`` so parts nested in asset groups resolve to
        their true world location.
        """
        pos = self.client.exec(
            self._FIND_JS.format(name=json.dumps(name))
            + "if (!found) return null;"
            "const v = new THREE.Vector3(); found.getWorldPosition(v);"
            "return { x: v.x, z: v.z };"
        )
        if not pos:
            raise KeyError(f"object not found in scene: {name}")
        return float(pos["z"]), float(pos["x"])

    def object_exists(self, name: str) -> bool:
        """Whether a named object (exact or part-suffix match) is in the scene."""
        return bool(
            self.client.exec(
                self._FIND_JS.format(name=json.dumps(name)) + "return found ? true : false;"
            )
        )

    def refresh_lidar(self) -> bool:
        """Resend the physics snapshot so server-side lidar sees spawned NPCs/props."""
        return self.client.refresh_lidar_snapshot()
