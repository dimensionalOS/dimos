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

"""Demo referee: the moving target, the simulated LOS sensor, and the ghosts.

Runs OUTSIDE both drone stacks and plays three roles:

1. **World builder / target mover** — spawns the red square (cube) and drives
   it along waypoints with a browser-side animation loop.
2. **Line-of-sight sensor** — the only component allowed to read sim truth.
   For each drone it checks range + field of view + occlusion (raycast) and
   publishes what that drone's *own sensor* would report on the drone's
   private topics (``/<name>/target_event`` and ``[SENSOR]`` messages into
   ``/<name>/human_input``). Drone stacks never touch the truth directly.
3. **Ghost renderer** — subscribes to each drone's ``peer_belief`` stream and
   renders, in the shared 3D view, a translucent "ghost" drone where each
   drone BELIEVES its partner is (from radio only), plus a translucent cube
   where it believes the target is. Ghosts lag and drift from the real
   avatars — that gap is the epistemic isolation made visible.

Coordinates: agents talk ROS coordinates (x, y ground plane, z up); the sim
scene uses Three.js (x, z ground plane, y up). ros(x, y) <-> three(z=x, x=y).
"""

import json
import math
import threading
import time
from dataclasses import dataclass, field

from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.simulation.dimsim.scene_client import SceneClient
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DRONE_COLORS = {"droneA": "#3b82f6", "droneB": "#f59e0b"}


def ros_to_three(x: float, y: float) -> tuple[float, float]:
    """ROS ground-plane (x, y) -> Three.js (x, z)."""
    return y, x


def three_to_ros(x: float, z: float) -> tuple[float, float]:
    return z, x


@dataclass
class SensorConfig:
    range_m: float = 12.0
    fov_deg: float = 140.0
    tick_s: float = 0.5
    # consecutive ticks needed to switch visible/lost (hysteresis)
    debounce: int = 2
    # seconds between "[SENSOR] still visible at ..." context refreshers
    refresh_s: float = 6.0


@dataclass
class _DroneSense:
    visible: bool = False
    streak: int = 0
    last_inject: float = 0.0
    last_seen: tuple[float, float] | None = None


class Referee:
    def __init__(
        self,
        scene: SceneClient,
        drones: list[str],
        sensor: SensorConfig | None = None,
    ) -> None:
        self.scene = scene
        self.drones = drones
        self.sensor = sensor or SensorConfig()
        # SceneClient.exec is called from several referee threads (sensor loop,
        # belief subscriptions); serialize WS round-trips.
        self._exec_lock = threading.Lock()
        self._exec_failures = 0
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._sense = {d: _DroneSense() for d in drones}
        self._target_events = {
            d: pLCMTransport(f"/{d}/target_event") for d in drones
        }
        self._human_inputs = {
            d: pLCMTransport(f"/{d}/human_input") for d in drones
        }
        for t in [*self._target_events.values(), *self._human_inputs.values()]:
            t.start()
        self._belief_subs: list[pLCMTransport] = []
        self.target_pos_ros: tuple[float, float] | None = None

    def _exec(self, code: str, timeout: float = 5.0):  # type: ignore[no-untyped-def]
        """Serialized exec with automatic reconnect after repeated failures
        (the control WS can die silently; the page itself usually survives)."""
        with self._exec_lock:
            try:
                result = self.scene.exec(code, timeout=timeout)
                self._exec_failures = 0
                return result
            except Exception as e:
                self._exec_failures += 1
                if self._exec_failures >= 2:
                    logger.warning(
                        f"referee exec failing ({type(e).__name__}: {e}) — reconnecting SceneClient"
                    )
                    try:
                        self.scene.stop()
                    except Exception:
                        pass
                    fresh = SceneClient(host=self.scene.host, port=self.scene.port)
                    fresh.start()
                    self.scene = fresh
                    self._exec_failures = 0
                    return self.scene.exec(code, timeout=timeout)
                raise

    # -- world building -------------------------------------------------------

    def build_arena(self, half_x: float, half_y: float, wall_height: float = 2.5) -> None:
        """Perimeter walls of a [-half_x..half_x] x [-half_y..half_y] ROS arena."""
        hx, hy = half_x, half_y
        # SceneClient.add_wall takes Three.js ground coords (x1, z1, x2, z2).
        for (rx1, ry1), (rx2, ry2) in [
            ((-hx, -hy), (-hx, hy)),
            ((-hx, hy), (hx, hy)),
            ((hx, hy), (hx, -hy)),
            ((hx, -hy), (-hx, -hy)),
        ]:
            (tx1, tz1) = ros_to_three(rx1, ry1)
            (tx2, tz2) = ros_to_three(rx2, ry2)
            self.scene.add_wall(tx1, tz1, tx2, tz2, height=wall_height, thickness=0.3)

    def add_obstacle_wall(
        self, x1: float, y1: float, x2: float, y2: float, height: float = 2.5
    ) -> None:
        """Obstacle wall between ROS points (x1,y1)-(x2,y2)."""
        (tx1, tz1) = ros_to_three(x1, y1)
        (tx2, tz2) = ros_to_three(x2, y2)
        self.scene.add_wall(tx1, tz1, tx2, tz2, height=height, thickness=0.4, color="#7c6f64")

    def install_target(self, waypoints_ros: list[tuple[float, float]], speed: float = 0.6,
                       altitude: float = 0.6, loop: bool = True) -> None:
        """Spawn the red cube and drive it along ROS waypoints browser-side."""
        pts = [ros_to_three(x, y) for (x, y) in waypoints_ros]
        pts_js = json.dumps([[x, z] for (x, z) in pts])
        self.scene.exec(f"""
        const old = scene.getObjectByName('demo_target');
        if (old) scene.remove(old);
        const geo = new THREE.BoxGeometry(0.8, 0.8, 0.8);
        const mat = new THREE.MeshStandardMaterial({{color: 0xe11d48, emissive: 0x991b33, emissiveIntensity: 0.5}});
        const cube = new THREE.Mesh(geo, mat);
        cube.name = 'demo_target';
        const pts = {pts_js};
        cube.position.set(pts[0][0], {altitude}, pts[0][1]);
        scene.add(cube);
        window.__demoTarget = {{
            cube, pts, speed: {speed}, seg: 0, t: 0, loop: {str(loop).lower()},
            paused: false, last: performance.now(),
        }};
        if (!window.__demoTargetLoop) {{
            window.__demoTargetLoop = true;
            const step = () => {{
                const st = window.__demoTarget;
                if (st) {{
                    const now = performance.now();
                    const dt = Math.min((now - st.last) / 1000, 0.1);
                    st.last = now;
                    if (!st.paused && st.pts.length > 1) {{
                        const a = st.pts[st.seg];
                        const b = st.pts[(st.seg + 1) % st.pts.length];
                        const segLen = Math.hypot(b[0] - a[0], b[1] - a[1]);
                        st.t += (st.speed * dt) / Math.max(segLen, 1e-6);
                        if (st.t >= 1) {{
                            st.t = 0;
                            st.seg = (st.seg + 1) % st.pts.length;
                            if (st.seg === st.pts.length - 1 && !st.loop) st.paused = true;
                        }}
                        const na = st.pts[st.seg];
                        const nb = st.pts[(st.seg + 1) % st.pts.length];
                        st.cube.position.x = na[0] + (nb[0] - na[0]) * st.t;
                        st.cube.position.z = na[1] + (nb[1] - na[1]) * st.t;
                        st.cube.rotation.y += 0.8 * dt;
                    }}
                }}
                requestAnimationFrame(step);
            }};
            requestAnimationFrame(step);
        }}
        return 'target installed';
        """)

    def mark_drones_for_los(self) -> None:
        """Tag drone avatars (and ghosts) so LOS raycasts ignore them."""
        self.scene.exec("""
        for (const [, av] of agents) {
            av.group.traverse(o => { o.userData.__demoIgnoreLOS = true; });
        }
        if (agent) agent.group.traverse(o => { o.userData.__demoIgnoreLOS = true; });
        return 'marked';
        """)

    # -- ghosts ---------------------------------------------------------------

    def install_ghosts(self) -> None:
        """Create translucent belief-ghosts: for each observer, a ghost of its
        peer and a believed-target cube. Hidden until first belief arrives."""
        colors = json.dumps(DRONE_COLORS)
        self.scene.exec(f"""
        const colors = {colors};
        window.__demoGhosts = window.__demoGhosts || {{}};
        for (const [observer, peers] of [['droneA', ['droneB']], ['droneB', ['droneA']]]) {{
            for (const peer of peers) {{
                const key = observer + ':' + peer;
                if (window.__demoGhosts[key]) continue;
                const g = new THREE.Group();
                g.name = 'ghost_' + key;
                const mat = new THREE.MeshStandardMaterial({{
                    color: new THREE.Color(colors[peer] || '#888888'),
                    transparent: true, opacity: 0.28, depthWrite: false,
                }});
                const body = new THREE.Mesh(new THREE.CapsuleGeometry(0.22, 0.25, 6, 12), mat);
                const ring = new THREE.Mesh(
                    new THREE.TorusGeometry(0.45, 0.03, 8, 24),
                    new THREE.MeshBasicMaterial({{
                        color: new THREE.Color(colors[observer] || '#ffffff'),
                        transparent: true, opacity: 0.6,
                    }}));
                ring.rotation.x = Math.PI / 2;
                ring.position.y = -0.25;
                g.add(body); g.add(ring);
                const tgt = new THREE.Mesh(
                    new THREE.BoxGeometry(0.8, 0.8, 0.8),
                    new THREE.MeshBasicMaterial({{
                        color: 0xe11d48, wireframe: true, transparent: true, opacity: 0.5,
                    }}));
                tgt.name = 'ghost_target_' + observer;
                tgt.visible = false;
                g.visible = false;
                g.traverse(o => {{ o.userData.__demoIgnoreLOS = true; }});
                tgt.userData.__demoIgnoreLOS = true;
                scene.add(g); scene.add(tgt);
                window.__demoGhosts[key] = {{group: g, target: tgt}};
            }}
        }}
        return 'ghosts installed';
        """)

    def start_ghost_renderer(self) -> None:
        for drone in self.drones:
            sub = pLCMTransport(f"/{drone}/peer_belief")
            sub.start()
            sub.subscribe(self._on_belief)
            self._belief_subs.append(sub)

    def _on_belief(self, raw: str) -> None:
        try:
            data = json.loads(raw)
            observer = data["observer"]
            for peer, b in data.get("beliefs", {}).items():
                pos = b.get("position")
                sight = b.get("target_sighting")
                js = []
                if pos is not None:
                    # belief positions are ROS (from odom tags) -> three
                    tx, tz = ros_to_three(pos[0], pos[1])
                    ty = pos[2] if len(pos) > 2 else 1.2
                    js.append(f"""
                    const gh = (window.__demoGhosts || {{}})['{observer}:{peer}'];
                    if (gh) {{ gh.group.visible = true;
                        gh.group.position.set({tx:.2f}, {ty:.2f}, {tz:.2f}); }}""")
                if sight is not None:
                    tx, tz = ros_to_three(sight[0], sight[1])
                    js.append(f"""
                    const gt = scene.getObjectByName('ghost_target_{observer}');
                    if (gt) {{ gt.visible = true; gt.position.set({tx:.2f}, 0.6, {tz:.2f}); }}""")
                if js:
                    self._exec("\n".join(js) + "\nreturn 'ok';", timeout=5.0)
        except Exception:
            logger.warning("ghost update failed", exc_info=True)

    # -- LOS sensor loop ------------------------------------------------------

    def start_sensor(self) -> None:
        t = threading.Thread(target=self._sensor_loop, daemon=True)
        t.start()
        self._threads.append(t)

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=3.0)
        # Transports are deliberately NOT stopped here: LCM teardown can hang
        # for minutes and stall the launcher's shutdown (video save!). All
        # referee threads are daemons — they die with the process.

    def _sensor_loop(self) -> None:
        cfg = self.sensor
        cos_half_fov = math.cos(math.radians(cfg.fov_deg) / 2)
        drones_js = json.dumps(self.drones)
        code = f"""
        const t = scene.getObjectByName('demo_target');
        if (!t) return null;
        const out = {{target: [t.position.x, t.position.y, t.position.z], vis: {{}}}};
        const ray = new THREE.Raycaster();
        for (const name of {drones_js}) {{
            const av = agents.get(name);
            if (!av) {{ out.vis[name] = false; continue; }}
            const p = av.group.position.clone();
            const dir = t.position.clone().sub(p);
            const dist = dir.length();
            let visible = dist <= {cfg.range_m};
            if (visible) {{
                const yaw = av.group.rotation.y;
                const facing = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw));
                const flat = new THREE.Vector3(dir.x, 0, dir.z);
                if (flat.length() > 0.3) {{
                    if (facing.dot(flat.normalize()) < {cos_half_fov:.4f}) visible = false;
                }}
            }}
            if (visible) {{
                ray.set(p, dir.clone().normalize());
                ray.far = dist - 0.5;
                const hits = ray.intersectObjects(scene.children, true).filter(h => {{
                    if (!h.object.isMesh || h.object.visible === false) return false;
                    let o = h.object;
                    while (o) {{
                        if (o.userData && o.userData.__demoIgnoreLOS) return false;
                        if (o === t) return false;
                        o = o.parent;
                    }}
                    return true;
                }});
                if (hits.length > 0) visible = false;
            }}
            out.vis[name] = visible;
        }}
        return out;
        """
        while not self._stop.is_set():
            t0 = time.time()
            try:
                result = self._exec(code, timeout=5.0)
                if result:
                    self._process_sense(result)
            except Exception:
                logger.warning("sensor tick failed", exc_info=True)
            dt = time.time() - t0
            self._stop.wait(max(cfg.tick_s - dt, 0.05))

    def _process_sense(self, result: dict) -> None:  # type: ignore[type-arg]
        cfg = self.sensor
        tx, ty, tz = result["target"]
        rx, ry = three_to_ros(tx, tz)
        self.target_pos_ros = (rx, ry)
        now = time.time()
        for drone, raw_visible in result["vis"].items():
            s = self._sense[drone]
            if raw_visible != s.visible:
                s.streak += 1
                if s.streak >= cfg.debounce:
                    s.visible = raw_visible
                    s.streak = 0
                    if s.visible:
                        s.last_seen = (rx, ry)
                        s.last_inject = now
                        self._human_inputs[drone].publish(
                            f"[SENSOR] TARGET IN SIGHT at ({rx:.1f}, {ry:.1f}). "
                            f"Immediately report_sighting({rx:.1f}, {ry:.1f}) and then fly_to it."
                        )
                    else:
                        last = s.last_seen or (rx, ry)
                        self._human_inputs[drone].publish(
                            f"[SENSOR] You LOST sight of the target. Last seen at "
                            f"({last[0]:.1f}, {last[1]:.1f}). Tell your partner and re-search."
                        )
            else:
                s.streak = 0
            if s.visible:
                s.last_seen = (rx, ry)
                if now - s.last_inject >= cfg.refresh_s:
                    s.last_inject = now
                    self._human_inputs[drone].publish(
                        f"[SENSOR] Target still in sight, now at ({rx:.1f}, {ry:.1f}). "
                        f"Keep pursuing: fly_to({rx:.1f}, {ry:.1f}) and update your partner."
                    )
            state = f"VISIBLE at ({rx:.1f}, {ry:.1f})" if s.visible else (
                f"NOT VISIBLE (last seen at ({s.last_seen[0]:.1f}, {s.last_seen[1]:.1f}))"
                if s.last_seen else "NOT VISIBLE (never seen)"
            )
            self._target_events[drone].publish(state)
