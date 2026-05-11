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

"""Python SDK for DimSim scene manipulation.

Connects to the DimSim bridge server over WebSocket and sends exec commands
to the browser-side SceneEditor.  Provides high-level helpers for common
operations (load map, add NPC, manage colliders) and a raw ``exec()`` escape
hatch for arbitrary Three.js code.

Usage::

    from dimos.robot.sim.scene_client import SceneClient

    scene = SceneClient()                      # connects to ws://localhost:8090
    scene.load_map("https://example.com/map.glb")
    scene.add_npc(url="/npcs/soldier.glb", name="guard", animation="Walk")
    scene.exec("scene.fog = new THREE.Fog(0xcccccc, 10, 50)")
    scene.close()

Or as a context manager::

    with SceneClient() as scene:
        scene.load_map("/local-assets/hotel.glb", scale=2.0)
"""

from __future__ import annotations

import copy
import json
from pathlib import Path
import shutil
import threading
from typing import Any, cast
import urllib.error
import urllib.request
import uuid

import websocket

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# -- Embodiment presets --------------------------------------------------------
# Each preset defines physics mode + default dimensions.  Users can override
# any field, or define fully custom embodiments.

EMBODIMENT_PRESETS: dict[str, dict[str, Any]] = {
    # -- Ground robots (character controller + gravity + collision) --
    "unitree-go2": {
        "radius": 0.12,
        "halfHeight": 0.25,
        "lidarMountHeight": 0.35,
        "embodimentType": "quadruped",
        "avatarUrl": ["/agent-model/unitree_go2.glb", "/agent-model/robot.glb"],
        # Physics
        "maxSpeed": 3.0,
        "turnRate": 3.0,
        "gravity": -9.81,
        "maxStepHeight": 0.25,
        "maxSlopeAngle": 45,
    },
    "quadruped": {  # alias for unitree-go2
        "radius": 0.12,
        "halfHeight": 0.25,
        "lidarMountHeight": 0.35,
        "embodimentType": "quadruped",
        "avatarUrl": ["/agent-model/unitree_go2.glb", "/agent-model/robot.glb"],
        "maxSpeed": 3.0,
        "turnRate": 3.0,
        "gravity": -9.81,
        "maxStepHeight": 0.25,
        "maxSlopeAngle": 45,
    },
    "differential-drive": {
        "radius": 0.15,
        "halfHeight": 0.2,
        "lidarMountHeight": 0.35,
        "embodimentType": "quadruped",  # ground physics
        "avatarUrl": ["/agent-model/robot.glb"],
        "maxSpeed": 2.0,
        "turnRate": 2.5,  # differential drive turns by wheel speed diff
        "gravity": -9.81,
        "maxStepHeight": 0.05,  # small wheels can't climb steps
        "maxSlopeAngle": 20,
    },
    "ackermann": {
        "radius": 0.3,
        "halfHeight": 0.4,
        "lidarMountHeight": 0.8,
        "embodimentType": "quadruped",  # ground physics
        "avatarUrl": ["/agent-model/robot.glb"],
        "maxSpeed": 5.0,
        "turnRate": 1.2,  # car-like: slow turn rate (limited steering angle)
        "gravity": -9.81,
        "maxStepHeight": 0.1,
        "maxSlopeAngle": 30,
    },
    "holonomic": {
        "radius": 0.2,
        "halfHeight": 0.25,
        "lidarMountHeight": 0.4,
        "embodimentType": "quadruped",  # ground physics (strafing via cmd_vel.linear.y)
        "avatarUrl": ["/agent-model/robot.glb"],
        "maxSpeed": 2.5,
        "turnRate": 4.0,  # omnidirectional: fast rotation
        "gravity": -9.81,
        "maxStepHeight": 0.05,
        "maxSlopeAngle": 15,
    },
    "humanoid": {
        "radius": 0.2,
        "halfHeight": 0.8,
        "lidarMountHeight": 1.6,
        "embodimentType": "quadruped",  # ground physics
        "avatarUrl": ["/agent-model/robot.glb"],
        "maxSpeed": 1.5,
        "turnRate": 2.0,
        "gravity": -9.81,
        "maxStepHeight": 0.3,  # can step over things
        "maxSlopeAngle": 45,
    },
    "small-robot": {
        "radius": 0.08,
        "halfHeight": 0.15,
        "lidarMountHeight": 0.25,
        "embodimentType": "quadruped",
        "avatarUrl": ["/agent-model/robot.glb"],
        "maxSpeed": 1.0,
        "turnRate": 3.0,
        "gravity": -9.81,
        "maxStepHeight": 0.03,
        "maxSlopeAngle": 15,
    },
    # -- Flight robots (6DoF, no gravity) --
    "drone": {
        "radius": 0.2,
        "halfHeight": 0.1,
        "lidarMountHeight": 0.15,
        "embodimentType": "drone",
        "avatarUrl": ["/agent-model/robot.glb"],
        "maxSpeed": 5.0,
        "turnRate": 4.0,
        "gravity": 0,  # no gravity in flight
        "maxAltitude": 20.0,
    },
}

ASSETS_DIR = Path.home() / ".dimsim" / "assets"

# Where authored scenes are saved. Lives next to the rest of the sim stack
# so scene JSONs ride along with the dimos repo (and check into git).
# Pass `dir=` to `save()` for explicit overrides.
SCENES_DIR = Path(__file__).resolve().parent / "scenes"


def _resolve_scenes_dir(override: str | Path | None = None) -> Path:
    if override is not None:
        return Path(override).expanduser().resolve()
    return SCENES_DIR


def _fresh_journal() -> dict[str, Any]:
    """Schema-shaped record of authoring calls; save() merges this with the
    bridge's current scene to produce a self-contained JSON. load_map /
    add_npc bump _unjournaled so save() can warn about un-saved GLB asset
    additions (v1 limitation)."""
    return {
        "embodiment": None,
        "primitives": [],
        "spawn_point": None,
        "_removed_names": set(),
        "_unjournaled": 0,
    }


class SceneExecError(RuntimeError):
    """Raised when browser-side JS execution fails."""


class SceneClient:
    """WebSocket client for DimSim scene manipulation.

    Connects to the DimSim bridge server's control channel and sends
    ``{type: "exec", code, id}`` commands.  The browser-side SceneEditor
    evaluates the JS and returns ``{type: "execResult", id, success, result}``.

    All high-level methods (load_map, add_npc, etc.) are thin wrappers that
    generate JS code using the helpers already exposed in the SceneEditor
    sandbox: ``loadGLTF``, ``addCollider``, ``removeCollider``, ``addNPC``,
    ``removeNPC``.

    Parameters
    ----------
    host : str
        Bridge server host (default ``"localhost"``).
    port : int
        Bridge server port (default ``8090``).
    channel : str
        Multi-page channel name (default ``""`` = single page mode).
    timeout : float
        Default timeout in seconds for exec commands.
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 8090,
        channel: str = "",
        timeout: float = 30.0,
    ):
        self.host = host
        self.port = port
        self.channel = channel
        self.timeout = timeout
        self._pending: dict[str, threading.Event] = {}
        self._results: dict[str, dict[str, Any]] = {}
        self._ws: websocket.WebSocket | None = None
        self._recv_thread: threading.Thread | None = None
        self._closed = False
        self._journal: dict[str, Any] = _fresh_journal()
        self._connect()

    # ── connection lifecycle ──────────────────────────────────────────────

    def _connect(self) -> None:
        url = f"ws://{self.host}:{self.port}?ch=control"
        if self.channel:
            url += f"&channel={self.channel}"
        self._ws = websocket.WebSocket()
        self._ws.connect(url)  # type: ignore[no-untyped-call]
        self._ws.settimeout(1.0)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        logger.info(f"SceneClient connected to {url}")

    def _recv_loop(self) -> None:
        while not self._closed and self._ws:
            try:
                raw = self._ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            except (websocket.WebSocketConnectionClosedException, OSError):
                break
            if isinstance(raw, bytes):
                continue
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("type") == "execResult" and "id" in msg:
                mid = msg["id"]
                if mid in self._pending:
                    self._results[mid] = msg
                    self._pending[mid].set()

    def close(self) -> None:
        """Close the WebSocket connection."""
        self._closed = True
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
        self._ws = None

    def __enter__(self) -> SceneClient:
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    # ── core exec ─────────────────────────────────────────────────────────

    def exec(self, code: str, timeout: float | None = None) -> Any:
        """Execute arbitrary JS in the browser SceneEditor sandbox.

        The code runs as an async function body with access to:
        ``scene``, ``THREE``, ``RAPIER``, ``rapierWorld``, ``renderer``,
        ``camera``, ``agent``, ``assets``, ``assetsGroup``,
        ``loadGLTF(url)``, ``addCollider(obj, shape?)``,
        ``removeCollider(obj)``, ``addNPC(opts)``, ``removeNPC(name)``.

        Use ``return`` to send a value back to Python.

        Parameters
        ----------
        code : str
            JavaScript code to execute.
        timeout : float, optional
            Seconds to wait for result (default: ``self.timeout``).

        Returns
        -------
        Any
            The serialized return value from the JS code.

        Raises
        ------
        SceneExecError
            If the JS execution fails.
        TimeoutError
            If no result within timeout.
        """
        timeout = timeout if timeout is not None else self.timeout
        msg_id = str(uuid.uuid4())
        event = threading.Event()
        self._pending[msg_id] = event

        cmd: dict[str, Any] = {"type": "exec", "code": code, "id": msg_id}
        if self.channel:
            cmd["channel"] = self.channel
        self._ws.send(json.dumps(cmd))  # type: ignore[union-attr]

        if not event.wait(timeout):
            self._pending.pop(msg_id, None)
            raise TimeoutError(f"exec timed out after {timeout}s")

        result = self._results.pop(msg_id)
        self._pending.pop(msg_id, None)

        if not result.get("success"):
            raise SceneExecError(result.get("error", "unknown error"))
        return result.get("result")

    # ── high-level helpers ────────────────────────────────────────────────

    def load_map(
        self,
        url: str,
        position: tuple[float, float, float] = (0, 0, 0),
        scale: float = 1.0,
        collider: str | None = "trimesh",
        name: str | None = None,
        auto_scale: bool | float = True,
    ) -> dict[str, Any]:
        """Load a GLTF/GLB map into the scene.

        Parameters
        ----------
        url : str
            URL or path to the .glb/.gltf file (can be absolute URL,
            ``/local-assets/...``, or ``/proxy?url=...`` for CORS).
        position : tuple
            (x, y, z) world position.
        scale : float
            Uniform scale factor (applied before auto_scale).
        collider : str or None
            Collider shape: ``"trimesh"`` (default), ``"box"``, ``"sphere"``,
            or ``None`` to skip collider.
        name : str, optional
            Name for the loaded model (for later lookup via ``scene.getObjectByName``).
        auto_scale : bool or float
            If True, auto-detect cm/m mismatch and normalize (default 50m max).
            If a number, use that as the max dimension in meters.
            If False, skip auto-scaling.

        Returns
        -------
        dict
            ``{name, uuid, collider, scaleFactor}`` info about the loaded model.
        """
        name_js = f"model.name = {json.dumps(name)};" if name else ""
        collider_js = (
            f"const col = addCollider(model, {json.dumps(collider)});"
            if collider
            else "const col = null;"
        )
        if auto_scale is False:
            auto_scale_js = "const scaleFactor = 1.0;"
        else:
            max_dim = 50 if auto_scale is True else float(auto_scale)
            auto_scale_js = f"const scaleFactor = autoScale(model, {max_dim});"
        code = f"""
const gltf = await loadGLTF({json.dumps(url)});
const model = gltf.scene;
model.position.set({position[0]}, {position[1]}, {position[2]});
model.scale.setScalar({scale});
model.updateMatrixWorld(true);
{auto_scale_js}
model.traverse(c => {{ if (c.isMesh) {{ c.castShadow = true; c.receiveShadow = true; }} }});
{name_js}
scene.add(model);
{collider_js}
return {{ name: model.name, uuid: model.uuid, collider: col, scaleFactor }};
"""
        # GLB asset journaling not yet implemented — see save() warning.
        self._journal["_unjournaled"] += 1
        return cast(dict[str, Any], self.exec(code))

    def remove_object(self, name: str) -> bool:
        """Remove a named object from the scene.

        Disposes geometry/materials and removes any associated collider.

        Parameters
        ----------
        name : str
            The ``object.name`` to find and remove.

        Returns
        -------
        bool
            True if object was found and removed.
        """
        code = f"""
const obj = scene.getObjectByName({json.dumps(name)});
if (!obj) return false;
removeCollider(obj);
obj.name = "";
obj.traverse(c => {{ if (c.isMesh) {{ c.geometry?.dispose(); c.material?.dispose(); }} }});
scene.remove(obj);
return true;
"""
        removed = cast(bool, self.exec(code))
        if removed:
            # Drop matching primitives from the journal so save() reflects
            # the deletion. Match by id (set when journal entry was created).
            self._journal["primitives"] = [
                p for p in self._journal["primitives"]
                if p.get("id") != name and p.get("name") != name
            ]
            self._journal["_removed_names"].add(name)
        return removed

    def add_npc(
        self,
        url: str,
        name: str | None = None,
        position: tuple[float, float, float] = (0, 0, 0),
        rotation: float | None = None,
        scale: float | None = None,
        animation: str | int = 0,
        collider: bool = True,
    ) -> dict[str, Any]:
        """Add an animated NPC character to the scene.

        Parameters
        ----------
        url : str
            URL to animated GLTF/GLB model.
        name : str, optional
            NPC name (auto-generated if omitted).
        position : tuple
            (x, y, z) world position.
        rotation : float, optional
            Y-axis rotation in radians.
        scale : float, optional
            Uniform scale factor.
        animation : str or int
            Animation clip name (substring match) or index (default: 0).
        collider : bool
            Whether to add a trimesh collider (default: True).

        Returns
        -------
        dict
            ``{name, animations, activeAnimation, collider}``
        """
        opts: dict[str, Any] = {
            "url": url,
            "position": {"x": position[0], "y": position[1], "z": position[2]},
            "animation": animation,
            "collider": collider,
        }
        if name:
            opts["name"] = name
        if rotation is not None:
            opts["rotation"] = rotation
        if scale is not None:
            opts["scale"] = scale
        # NPC asset journaling not yet implemented — see save() warning.
        self._journal["_unjournaled"] += 1
        return cast(dict[str, Any], self.exec(f"return await addNPC({json.dumps(opts)});"))

    def remove_npc(self, name: str) -> bool:
        """Remove an NPC by name. Stops animation and removes collider.

        Parameters
        ----------
        name : str
            NPC name (as returned by ``add_npc``).

        Returns
        -------
        bool
            True if NPC was found and removed.
        """
        return cast(bool, self.exec(f"return removeNPC({json.dumps(name)});"))

    def add_collider(
        self,
        name: str,
        shape: str = "trimesh",
    ) -> dict[str, Any]:
        """Add a physics collider to a named scene object.

        Parameters
        ----------
        name : str
            Object name to find in scene.
        shape : str
            ``"trimesh"`` (default), ``"box"``, or ``"sphere"``.

        Returns
        -------
        dict
            ``{shape, uuid, size}``
        """
        code = f"""
const obj = scene.getObjectByName({json.dumps(name)});
if (!obj) throw new Error("Object not found: {name}");
return addCollider(obj, {json.dumps(shape)});
"""
        return cast(dict[str, Any], self.exec(code))

    def remove_collider(self, name: str) -> bool:
        """Remove collider from a named scene object.

        Parameters
        ----------
        name : str
            Object name to find in scene.

        Returns
        -------
        bool
            True if a collider existed and was removed.
        """
        code = f"""
const obj = scene.getObjectByName({json.dumps(name)});
if (!obj) throw new Error("Object not found: {name}");
return removeCollider(obj);
"""
        return cast(bool, self.exec(code))

    def add_object(
        self,
        geometry: str = "box",
        size: tuple[float, ...] = (1, 1, 1),
        color: int = 0x888888,
        position: tuple[float, float, float] = (0, 0, 0),
        name: str | None = None,
        dynamic: bool = False,
        mass: float = 1.0,
        restitution: float = 0.3,
        collider: str | None = "box",
    ) -> dict[str, Any]:
        """Add a primitive object to the scene with optional physics.

        Parameters
        ----------
        geometry : str
            ``"box"`` (default), ``"sphere"``, or ``"cylinder"``.
        size : tuple
            Dimensions — (w, h, d) for box, (radius,) for sphere,
            (radiusTop, radiusBottom, height) for cylinder.
        color : int
            Hex color (e.g. ``0xFF0000`` for red).
        position : tuple
            (x, y, z) world position.
        name : str, optional
            Object name.
        dynamic : bool
            If True, object responds to gravity and collisions.
        mass : float
            Mass in kg (only for dynamic objects).
        restitution : float
            Bounciness 0-1 (only for dynamic objects).
        collider : str or None
            Collider shape, or None to skip.

        Returns
        -------
        dict
            ``{name, uuid, collider}``
        """
        if geometry == "sphere":
            r = size[0] if size else 0.5
            geom_js = f"new THREE.SphereGeometry({r}, 24, 24)"
        elif geometry == "cylinder":
            rt = size[0] if len(size) > 0 else 0.5
            rb = size[1] if len(size) > 1 else rt
            h = size[2] if len(size) > 2 else 1.0
            geom_js = f"new THREE.CylinderGeometry({rt}, {rb}, {h}, 24)"
        else:
            w = size[0] if len(size) > 0 else 1
            h = size[1] if len(size) > 1 else 1
            d = size[2] if len(size) > 2 else 1
            geom_js = f"new THREE.BoxGeometry({w}, {h}, {d})"

        name_js = f"mesh.name = {json.dumps(name)};" if name else ""

        if collider:
            opts = {"shape": collider, "dynamic": dynamic, "mass": mass, "restitution": restitution}
            collider_js = f"const col = addCollider(mesh, {json.dumps(opts)});"
        else:
            collider_js = "const col = null;"

        code = f"""
const mesh = new THREE.Mesh(
    {geom_js},
    new THREE.MeshStandardMaterial({{ color: {color} }})
);
{name_js}
mesh.position.set({position[0]}, {position[1]}, {position[2]});
mesh.castShadow = true;
mesh.receiveShadow = true;
scene.add(mesh);
{collider_js}
return {{ name: mesh.name, uuid: mesh.uuid, collider: col }};
"""
        result = cast(dict[str, Any], self.exec(code))

        # Journal: record this primitive in scene-JSON shape so save() can
        # write a self-contained scene file. Built only after exec succeeds.
        if geometry == "sphere":
            dims = {"radius": float(size[0] if size else 0.5)}
        elif geometry == "cylinder":
            rt = float(size[0]) if len(size) > 0 else 0.5
            rb = float(size[1]) if len(size) > 1 else rt
            ch = float(size[2]) if len(size) > 2 else 1.0
            dims = {"radiusTop": rt, "radiusBottom": rb, "height": ch}
        else:
            w = float(size[0]) if len(size) > 0 else 1.0
            h = float(size[1]) if len(size) > 1 else 1.0
            d = float(size[2]) if len(size) > 2 else 1.0
            dims = {"width": w, "height": h, "depth": d}
        self._journal["primitives"].append({
            "id": name or f"prim-{uuid.uuid4().hex[:12]}",
            "type": geometry,
            "name": name or "",
            "dimensions": dims,
            "transform": {
                "position": {"x": position[0], "y": position[1], "z": position[2]},
                "rotation": {"x": 0, "y": 0, "z": 0},
                "scale": {"x": 1, "y": 1, "z": 1},
            },
            "material": {"color": f"#{color:06x}"},
            "castShadow": True,
            "receiveShadow": True,
        })
        return result

    def set_embodiment(
        self,
        preset: str | None = None,
        *,
        radius: float | None = None,
        half_height: float | None = None,
        lidar_mount_height: float | None = None,
        avatar_url: str | list[str] | None = None,
        physics: str | None = None,
        # Physics parameters
        max_speed: float | None = None,
        turn_rate: float | None = None,
        gravity: float | None = None,
        max_step_height: float | None = None,
        ground_snap_dist: float | None = None,
        max_slope_angle: float | None = None,
        friction: float | None = None,
        max_altitude: float | None = None,
    ) -> dict[str, Any]:
        """Set the robot embodiment — from a preset or fully custom.

        Use a named preset as a starting point, then override any field.
        Or skip the preset and specify everything manually.

        **Presets** (see ``EMBODIMENT_PRESETS``):
        - Ground: ``"unitree-go2"``, ``"differential-drive"``, ``"ackermann"``,
          ``"holonomic"``, ``"humanoid"``, ``"small-robot"``
        - Flight: ``"drone"``

        **Physics modes**:
        - ``"ground"`` — gravity, collision, ground snap, slope limits
        - ``"flight"`` — 6DoF, optional gravity, altitude ceiling

        **Avatar URL** can be:
        - Built-in: ``"/agent-model/robot.glb"``
        - Local asset: ``"/local-assets/my-drone.glb"`` (see :meth:`upload_asset`)
        - Any URL: ``"https://example.com/robot.glb"``

        Parameters
        ----------
        preset : str, optional
            Named preset to start from.
        radius : float, optional
            Agent capsule radius in meters.
        half_height : float, optional
            Agent capsule half-height in meters.
        lidar_mount_height : float, optional
            Height of lidar sensor in meters.
        avatar_url : str or list[str], optional
            GLTF model URL(s).
        physics : str, optional
            ``"ground"`` or ``"flight"``.
        max_speed : float, optional
            Linear speed multiplier (default varies by preset).
        turn_rate : float, optional
            Angular speed multiplier (default: same as max_speed).
        gravity : float, optional
            Gravity in m/s² (default -9.81 for ground, 0 for flight).
        max_step_height : float, optional
            Max step-up height in meters (ground only, default 0.25).
        ground_snap_dist : float, optional
            Ground snap distance in meters (ground only, default 0.5).
        max_slope_angle : float, optional
            Max climbable slope in degrees (ground only, default 45).
        friction : float, optional
            Capsule friction coefficient (default 0.8).
        max_altitude : float, optional
            Altitude ceiling in meters (flight only).

        Returns
        -------
        dict
            The final embodiment config that was sent.

        Examples
        --------
        >>> scene.set_embodiment("drone")
        >>> scene.set_embodiment("differential-drive", max_speed=1.5)
        >>> scene.set_embodiment("ackermann", turn_rate=0.8, max_slope_angle=15)
        >>> scene.set_embodiment(
        ...     radius=0.3, half_height=0.5, physics="ground",
        ...     avatar_url="/local-assets/my-robot.glb",
        ...     max_speed=2.0, max_step_height=0.1,
        ... )
        """
        # Start from preset defaults
        if preset:
            if preset not in EMBODIMENT_PRESETS:
                available = ", ".join(sorted(EMBODIMENT_PRESETS))
                raise ValueError(f"Unknown preset '{preset}'. Available: {available}")
            cfg = dict(EMBODIMENT_PRESETS[preset])
        else:
            cfg = dict(EMBODIMENT_PRESETS["unitree-go2"])

        # Apply overrides — geometry
        if radius is not None:
            cfg["radius"] = radius
        if half_height is not None:
            cfg["halfHeight"] = half_height
        if lidar_mount_height is not None:
            cfg["lidarMountHeight"] = lidar_mount_height
        if avatar_url is not None:
            cfg["avatarUrl"] = avatar_url if isinstance(avatar_url, list) else [avatar_url]
        if physics is not None:
            cfg["embodimentType"] = "drone" if physics == "flight" else "quadruped"

        # Apply overrides — physics parameters
        if max_speed is not None:
            cfg["maxSpeed"] = max_speed
        if turn_rate is not None:
            cfg["turnRate"] = turn_rate
        if gravity is not None:
            cfg["gravity"] = gravity
        if max_step_height is not None:
            cfg["maxStepHeight"] = max_step_height
        if ground_snap_dist is not None:
            cfg["groundSnapDist"] = ground_snap_dist
        if max_slope_angle is not None:
            cfg["maxSlopeAngle"] = max_slope_angle
        if friction is not None:
            cfg["friction"] = friction
        if max_altitude is not None:
            cfg["maxAltitude"] = max_altitude

        msg = {"type": "embodimentConfig", **cfg}
        if self.channel:
            msg["channel"] = self.channel
        self._ws.send(json.dumps(msg))  # type: ignore[union-attr]

        # Journal: schema-shaped embodiment for scene JSON.
        self._journal["embodiment"] = {
            "avatarUrl": cfg.get("avatarUrl", []),
            "radius": cfg.get("radius"),
            "halfHeight": cfg.get("halfHeight"),
            "lidarMountHeight": cfg.get("lidarMountHeight"),
            "type": cfg.get("embodimentType", "quadruped"),
            "walkSpeed": cfg.get("maxSpeed", 2.0),
        }

        # Swap the avatar model browser-side via exec
        avatar_urls = cfg.get("avatarUrl", [])
        if avatar_urls:
            urls_js = json.dumps(avatar_urls)
            r = cfg.get("radius", 0.12)
            hh = cfg.get("halfHeight", 0.25)
            self.exec(f"""
                if (agent.model) {{
                    agent.group.remove(agent.model);
                    agent.model = null;
                }}
                agent.avatarUrl = {urls_js};
                agent.radius = {r};
                agent.halfHeight = {hh};
                if (agent.group) agent.group.visible = true;
                agent._loadGLB();
                return "avatar_swap_initiated";
            """)

        return cfg

    @staticmethod
    def upload_asset(
        local_path: str | Path,
        dest_name: str | None = None,
    ) -> str:
        """Copy a local file to ``~/.dimsim/assets/`` for use in the scene.

        The file becomes available at ``/local-assets/<dest_name>`` in the
        browser. Use this to load custom GLTF models, textures, etc.

        Parameters
        ----------
        local_path : str or Path
            Path to the local file (e.g. ``"~/Downloads/my-drone.glb"``).
        dest_name : str, optional
            Filename in the assets dir. Defaults to the original filename.

        Returns
        -------
        str
            The URL path to use in the scene (e.g. ``"/local-assets/my-drone.glb"``).

        Examples
        --------
        >>> url = SceneClient.upload_asset("~/Downloads/drone.glb")
        >>> scene.set_embodiment(avatar_url=url, physics="flight")
        >>> # or for a map:
        >>> url = SceneClient.upload_asset("~/models/warehouse.glb")
        >>> scene.load_map(url)
        """
        src = Path(local_path).expanduser().resolve()
        if not src.exists():
            raise FileNotFoundError(f"File not found: {src}")

        name = dest_name or src.name
        dest = ASSETS_DIR / name
        dest.parent.mkdir(parents=True, exist_ok=True)

        if src != dest:
            shutil.copy2(str(src), str(dest))
            logger.info(f"Uploaded asset: {src.name} → {dest}")

        return f"/local-assets/{name}"

    @staticmethod
    def upload_asset_dir(
        local_dir: str | Path,
        dest_name: str | None = None,
    ) -> str:
        """Copy a directory of files to ``~/.dimsim/assets/`` (for multi-file GLTF).

        Some GLTF models come as a ``.gltf`` + ``.bin`` + textures. This copies
        the entire directory.

        Parameters
        ----------
        local_dir : str or Path
            Path to the local directory.
        dest_name : str, optional
            Subdirectory name in assets. Defaults to the directory name.

        Returns
        -------
        str
            URL path to the directory (e.g. ``"/local-assets/my-model/"``).
            Append the filename to load: ``"/local-assets/my-model/scene.gltf"``.
        """
        src = Path(local_dir).expanduser().resolve()
        if not src.is_dir():
            raise NotADirectoryError(f"Not a directory: {src}")

        name = dest_name or src.name
        dest = ASSETS_DIR / name
        if dest.exists():
            shutil.rmtree(str(dest))
        shutil.copytree(str(src), str(dest))
        logger.info(f"Uploaded asset dir: {src.name}/ → {dest}/")

        return f"/local-assets/{name}/"

    def clear_scene(self) -> int:
        """Remove all user-added objects from the scene.

        Preserves the agent, camera, lights, and renderer. Removes everything
        else (loaded maps, NPCs, etc.).

        Returns
        -------
        int
            Number of objects removed.
        """
        code = """
const keep = new Set();
// Keep agent and its children
if (agent && agent.group) keep.add(agent.group.uuid);
// Keep camera, lights, engine-internal objects (skyDome, avatar, primitive
// groups, etc. — tagged via userData.engineInternal in engine.js), and any
// THREE.Group container. Removing those breaks rendering / future imports.
scene.children.forEach(c => {
  if (c === camera || c.isLight || c.isAmbientLight || c.isDirectionalLight
      || c.isHemisphereLight || c === agent?.group
      || c.userData?.engineInternal === true || c.isGroup === true) {
    keep.add(c.uuid);
  }
});
const toRemove = scene.children.filter(c => !keep.has(c.uuid));
let count = 0;
for (const obj of toRemove) {
  removeCollider(obj);
  obj.name = "";
  obj.traverse(c => { if (c.isMesh) { c.geometry?.dispose(); c.material?.dispose(); } });
  scene.remove(obj);
  count++;
}
return count;
"""
        return cast(int, self.exec(code))

    def get_scene_info(self) -> dict[str, Any]:
        """Get info about the current scene (object names, counts).

        Returns
        -------
        dict
            ``{objectCount, objects: [{name, type, uuid}]}``
        """
        code = """
const objects = [];
scene.traverse(obj => {
  if (obj === scene) return;
  objects.push({ name: obj.name || "(unnamed)", type: obj.type, uuid: obj.uuid });
});
return { objectCount: objects.length, objects: objects.slice(0, 100) };
"""
        return cast(dict[str, Any], self.exec(code))

    def set_agent_position(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        """Teleport the agent to a world position.

        Sends a teleport command to the server-side physics engine, which
        updates the agent's kinematic body directly.  The new position is
        then broadcast to the browser for rendering.

        Parameters
        ----------
        x, y, z : float
            Target position in Three.js world coordinates (Y-up).
        """
        cmd: dict[str, Any] = {"type": "teleport", "x": x, "y": y, "z": z}
        if self.channel:
            cmd["channel"] = self.channel
        self._ws.send(json.dumps(cmd))  # type: ignore[union-attr]

    def get_agent_position(self) -> dict[str, Any]:
        """Get the agent's current world position.

        Returns
        -------
        dict
            ``{x, y, z}``
        """
        code = """
const p = agent.group.position;
return { x: p.x, y: p.y, z: p.z };
"""
        return cast(dict[str, Any], self.exec(code))

    # ── persistence ──────────────────────────────────────────────────────────

    def _fetch_active_scene(self) -> dict[str, Any]:
        """Fetch the bridge's currently-loaded scene over HTTP.

        Falls back to a minimal empty scene if the bridge endpoint is
        unavailable (older bridges without the /api/active-scene route).
        """
        url = f"http://{self.host}:{self.port}/api/active-scene"
        try:
            with urllib.request.urlopen(url, timeout=5) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
            content = payload.get("content")
            if isinstance(content, dict):
                return content
        except (urllib.error.URLError, json.JSONDecodeError, OSError) as e:
            logger.warning(f"Could not fetch active scene from bridge ({e}); using empty base")
        return {
            "version": "2.0",
            "worldKey": "default",
            "tags": ["scene"],
            "embodiment": None,
            "assets": [],
            "primitives": [],
            "lights": [],
            "groups": [],
            "sceneSettings": {},
        }

    def save(
        self,
        name: str,
        dir: str | Path | None = None,
        overwrite: bool = True,
    ) -> Path:
        """Persist the current authored scene to JSON.

        Reads the bridge's currently-loaded scene as the base, applies all
        recorded journal entries (additions, embodiment changes, deletions)
        to produce a self-contained scene file, and writes it to
        ``<scenes_dir>/<name>.json``. Also tells the bridge that this is now
        the active scene so a browser refresh shows it.

        Parameters
        ----------
        name : str
            Scene name (without ``.json``). Used as the filename and as the
            active-scene identifier the bridge will key its cache by.
        dir : str or Path, optional
            Override target directory. Defaults to ``dimos/robot/sim/scenes/``.
        overwrite : bool
            If False, raises ``FileExistsError`` when the file exists.

        Returns
        -------
        Path
            Path to the written JSON file.
        """
        scenes_dir = _resolve_scenes_dir(dir)
        scenes_dir.mkdir(parents=True, exist_ok=True)
        path = scenes_dir / f"{name}.json"
        if path.exists() and not overwrite:
            raise FileExistsError(f"{path} exists; pass overwrite=True or use a different name")

        if self._journal["_unjournaled"] > 0:
            logger.warning(
                f"save({name!r}): {self._journal['_unjournaled']} GLB/NPC asset call(s) "
                "were not recorded in the journal and will not appear in the saved file. "
                "GLB/NPC journaling is a v2 feature."
            )

        base = self._fetch_active_scene()
        merged = copy.deepcopy(base)

        # Embodiment: journal overrides base if set.
        if self._journal["embodiment"] is not None:
            merged["embodiment"] = copy.deepcopy(self._journal["embodiment"])

        # Primitives: drop base entries the user removed AND any base entry
        # whose id/name collides with a journal entry — journal wins, so a
        # rerun against an already-saved scene replaces in place instead of
        # duplicating.
        removed = self._journal["_removed_names"]
        journal_ids = {p.get("id") for p in self._journal["primitives"] if p.get("id")}
        journal_names = {p.get("name") for p in self._journal["primitives"] if p.get("name")}
        base_prims = [
            p for p in (merged.get("primitives") or [])
            if p.get("id") not in removed
            and p.get("name") not in removed
            and p.get("id") not in journal_ids
            and p.get("name") not in journal_names
        ]
        merged["primitives"] = base_prims + copy.deepcopy(self._journal["primitives"])

        if self._journal["spawn_point"] is not None:
            merged["dimosSpawnPoint"] = dict(self._journal["spawn_point"])

        if not isinstance(merged.get("tags"), list):
            merged["tags"] = ["scene"]

        path.write_text(json.dumps(merged, indent=2))
        logger.info(f"Saved scene {name!r} → {path}")

        # Tell the bridge: this is the new active scene. broadcast=False so
        # already-connected browser tabs don't reload (their state already
        # matches the file). Browser refresh will fetch the new content.
        msg: dict[str, Any] = {
            "type": "setActiveScene",
            "name": name,
            "content": merged,
            "broadcast": False,
        }
        if self.channel:
            msg["channel"] = self.channel
        try:
            self._ws.send(json.dumps(msg))  # type: ignore[union-attr]
        except Exception as e:
            logger.warning(f"save({name!r}): file written but bridge sticky-update failed: {e}")

        return path

    def reset(self) -> None:
        """Drop the journal and clear the scene browser-side back to base.

        After reset, ``save()`` would only contain whatever was in the base
        scene. Useful at the top of an authoring script to make reruns
        idempotent.
        """
        self.clear_scene()
        self._journal = _fresh_journal()

    def set_spawn_point(self, x: float, y: float, z: float) -> None:
        """Set where the agent spawns when this scene boots via dimos.

        Saved as ``dimosSpawnPoint`` in the scene JSON. Engine.js's dimos
        boot reads it; without it, the agent falls back to ``(2, 0.5, 3)``
        which may be inside or below user-authored geometry.
        """
        self._journal["spawn_point"] = {"x": float(x), "y": float(y), "z": float(z)}
        self.set_agent_position(x, y, z)

    def reload(self, name: str, dir: str | Path | None = None) -> None:
        """Load a saved scene file and broadcast it to all connected viewers.

        Reads ``<scenes_dir>/<name>.json``, sends it to the bridge with
        ``broadcast=True`` so already-open browser tabs hot-reload via
        ``importLevelFromJSON``. The bridge also caches it as the active
        scene, so future browser refreshes see it.

        Parameters
        ----------
        name : str
            Scene name (without ``.json``).
        dir : str or Path, optional
            Override source directory.
        """
        scenes_dir = _resolve_scenes_dir(dir)
        path = scenes_dir / f"{name}.json"
        content = json.loads(path.read_text())

        # Drop the journal — base is now this scene, nothing pending.
        self._journal = _fresh_journal()

        msg: dict[str, Any] = {
            "type": "setActiveScene",
            "name": name,
            "content": content,
            "broadcast": True,
        }
        if self.channel:
            msg["channel"] = self.channel
        self._ws.send(json.dumps(msg))  # type: ignore[union-attr]


__all__ = ["EMBODIMENT_PRESETS", "SceneClient", "SceneExecError"]
