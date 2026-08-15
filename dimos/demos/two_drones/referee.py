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
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
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
    range_m: float = 15.0
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
        # belief subscriptions). Each subsystem gets its own connection + lock.
        self._locks: dict[str, threading.Lock] = {}
        self._fail_counts: dict[str, int] = {}
        self._clients: dict[str, SceneClient] = {"main": scene}
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
        return self._exec_on("main", self.scene, code, timeout)

    def _exec_on(self, tag: str, client: SceneClient, code: str, timeout: float = 5.0):  # type: ignore[no-untyped-def]
        lock = self._locks.setdefault(tag, threading.Lock())
        with lock:
            try:
                result = client.exec(code, timeout=timeout)
                self._fail_counts[tag] = 0
                return result
            except Exception as e:
                self._fail_counts[tag] = self._fail_counts.get(tag, 0) + 1
                if self._fail_counts[tag] >= 2:
                    logger.warning(
                        f"referee exec[{tag}] failing ({type(e).__name__}: {e}) — reconnecting"
                    )
                    try:
                        client.stop()
                    except Exception:
                        pass
                    fresh = SceneClient(host=client.host, port=client.port)
                    fresh.start()
                    self._clients[tag] = fresh
                    self._fail_counts[tag] = 0
                    return fresh.exec(code, timeout=timeout)
                raise

    def _client_for(self, tag: str) -> SceneClient:
        """Dedicated connection per subsystem: a stall on one (sensor) must
        not serialize behind the other (ghosts)."""
        if tag not in self._clients:
            c = SceneClient(host=self.scene.host, port=self.scene.port)
            c.start()
            self._clients[tag] = c
        return self._clients[tag]

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
        self.scene.add_wall(tx1, tz1, tx2, tz2, height=height, thickness=0.4, color=0x7C6F64)

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

    def style_scene_for_video(self) -> None:
        """Director-view legibility: procedural quadcopter avatars (spinning
        rotors, heading nose), per-drone flight trails, FOV wedges, an
        on-screen HUD (drone status + radio log), and hidden player HUD."""
        colors = json.dumps(DRONE_COLORS)
        fov = self.sensor.fov_deg
        rng = self.sensor.range_m
        self._exec(f"""
        const colors = {colors};
        window.__demoRotors = [];
        window.__demoTrails = {{}};
        window.__demoWedges = {{}};

        function buildQuad(colorHex) {{
            const c = new THREE.Color(colorHex);
            const g = new THREE.Group();
            const mat = new THREE.MeshStandardMaterial({{color: c}});
            const dark = new THREE.MeshStandardMaterial({{color: 0x2a2a30}});
            const body = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.12, 0.34), mat);
            g.add(body);
            const armLen = 0.66;
            for (const a of [Math.PI/4, 3*Math.PI/4, 5*Math.PI/4, 7*Math.PI/4]) {{
                const arm = new THREE.Mesh(new THREE.BoxGeometry(armLen, 0.045, 0.06), dark);
                arm.rotation.y = a;
                g.add(arm);
                const hx = Math.cos(a) * armLen / 2, hz = -Math.sin(a) * armLen / 2;
                const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.07, 8), dark);
                hub.position.set(hx, 0.05, hz);
                g.add(hub);
                const rotor = new THREE.Mesh(
                    new THREE.CylinderGeometry(0.17, 0.17, 0.012, 16),
                    new THREE.MeshStandardMaterial({{color: c, transparent: true, opacity: 0.45}}));
                rotor.position.set(hx, 0.1, hz);
                g.add(rotor);
                window.__demoRotors.push(rotor);
            }}
            const nose = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.18, 8), mat);
            nose.rotation.x = Math.PI / 2;
            nose.position.set(0, 0, 0.28);
            g.add(nose);
            return g;
        }}

        for (const [name, av] of agents) {{
            // Hide the Go2 stub; mount a quadcopter.
            for (const child of [...av.group.children]) child.visible = false;
            if (!av.group.getObjectByName('demo_quad')) {{
                const quad = buildQuad(colors[name] || '#888888');
                quad.name = 'demo_quad';
                quad.traverse(o => {{ o.userData.__demoIgnoreLOS = true; }});
                av.group.add(quad);
            }}
            // FOV wedge on the floor, rotates with the drone's heading:
            // faint fill (nominal sensor range/FOV — occlusion still applies,
            // the red sighting line shows actual LOS) + a crisp arc rim.
            if (!av.group.getObjectByName('demo_wedge')) {{
                const wedgeGroup = new THREE.Group();
                wedgeGroup.name = 'demo_wedge';
                const c = new THREE.Color(colors[name] || '#888888');
                const wedge = new THREE.Mesh(
                    new THREE.CircleGeometry({rng}, 28,
                        Math.PI / 2 - {math.radians(fov) / 2:.4f}, {math.radians(fov):.4f}),
                    new THREE.MeshBasicMaterial({{
                        color: c, transparent: true, opacity: 0.035,
                        depthWrite: false, side: THREE.DoubleSide,
                    }}));
                const rimPts = [];
                const n = 40;
                for (let i = 0; i <= n; i++) {{
                    const a = Math.PI / 2 - {math.radians(fov) / 2:.4f} + ({math.radians(fov):.4f}) * i / n;
                    rimPts.push(new THREE.Vector3(Math.cos(a) * {rng}, Math.sin(a) * {rng}, 0));
                }}
                const rim = new THREE.Line(
                    new THREE.BufferGeometry().setFromPoints(rimPts),
                    new THREE.LineBasicMaterial({{color: c, transparent: true, opacity: 0.4}}));
                wedgeGroup.add(wedge);
                wedgeGroup.add(rim);
                wedgeGroup.rotation.x = Math.PI / 2;
                wedgeGroup.position.y = -1.12;
                wedgeGroup.traverse(o => {{ o.userData.__demoIgnoreLOS = true; }});
                av.group.add(wedgeGroup);
                window.__demoWedges[name] = wedge;
            }}
            // Ground trail.
            const maxPts = 6000;
            const geo = new THREE.BufferGeometry();
            const attr = new THREE.BufferAttribute(new Float32Array(maxPts * 3), 3);
            geo.setAttribute('position', attr);
            geo.setDrawRange(0, 0);
            const line = new THREE.Line(geo, new THREE.LineBasicMaterial({{
                color: new THREE.Color(colors[name] || '#888888'),
                transparent: true, opacity: 0.85,
            }}));
            line.frustumCulled = false;
            line.userData.__demoIgnoreLOS = true;
            scene.add(line);
            window.__demoTrails[name] = {{line, attr, n: 0, max: maxPts, lx: 1e9, lz: 1e9}};
        }}

        // Rotor spin + trail sampling loop.
        if (!window.__demoFxLoop) {{
            window.__demoFxLoop = true;
            let lastSample = 0;
            const fx = (t) => {{
                for (const r of (window.__demoRotors || [])) r.rotation.y += 0.85;
                if (t - lastSample > 350) {{
                    lastSample = t;
                    for (const [name, av] of agents) {{
                        // The stub GLB loads asynchronously and would pop back
                        // over the quadcopter — keep everything else hidden.
                        for (const child of av.group.children) {{
                            if (child.name !== 'demo_quad' && child.name !== 'demo_wedge') {{
                                child.visible = false;
                            }}
                        }}
                        const tr = (window.__demoTrails || {{}})[name];
                        if (!tr || tr.n >= tr.max) continue;
                        const p = av.group.position;
                        if (Math.hypot(p.x - tr.lx, p.z - tr.lz) > 0.12) {{
                            tr.attr.setXYZ(tr.n, p.x, 0.06, p.z);
                            tr.n++; tr.lx = p.x; tr.lz = p.z;
                            tr.attr.needsUpdate = true;
                            tr.line.geometry.setDrawRange(0, tr.n);
                        }}
                    }}
                }}
                requestAnimationFrame(fx);
            }};
            requestAnimationFrame(fx);
        }}

        // On-screen HUD: per-drone status + radio log.
        if (!document.getElementById('demo-hud')) {{
            const d = document.createElement('div');
            d.id = 'demo-hud';
            d.style.cssText = 'position:fixed;top:10px;left:10px;z-index:9999;' +
                'font:12px/1.55 ui-monospace,monospace;color:#fff;' +
                'background:rgba(10,10,14,0.62);padding:10px 14px;border-radius:10px;' +
                'max-width:560px;pointer-events:none';
            d.innerHTML = '<div id="demo-hud-status"></div>' +
                '<div id="demo-hud-radio" style="margin-top:6px;border-top:1px solid ' +
                'rgba(255,255,255,0.25);padding-top:6px"></div>';
            document.body.appendChild(d);
            window.__demoHudStatus = (lines) => {{
                const el = document.getElementById('demo-hud-status');
                if (!el) return;
                el.innerHTML = '';
                for (const [color, text] of lines) {{
                    const row = document.createElement('div');
                    const dot = document.createElement('span');
                    dot.style.color = color;
                    dot.textContent = '● ';
                    const span = document.createElement('span');
                    span.textContent = text;
                    row.appendChild(dot); row.appendChild(span);
                    el.appendChild(row);
                }}
            }};
            window.__demoHudRadio = (sender, text, color) => {{
                const el = document.getElementById('demo-hud-radio');
                if (!el) return;
                const row = document.createElement('div');
                const s = document.createElement('span');
                s.style.color = color || '#aaa';
                s.textContent = '📡 ' + sender + ': ';
                const b = document.createElement('span');
                b.textContent = text;
                row.appendChild(s); row.appendChild(b);
                el.appendChild(row);
                while (el.children.length > 5) el.removeChild(el.firstChild);
            }};
        }}

        for (const sel of ['#shortcuts', '#crosshair', '.shortcuts-floating', '#interaction-hint']) {{
            const el = document.querySelector(sel);
            if (el) el.style.display = 'none';
        }}
        return 'styled';
        """, timeout=15.0)

    def start_goal_markers(self) -> None:
        """Render a ground circle at every navigation goal a drone commits to:
        small ring = one follower hop, large double ring = an LLM-level fly_to
        destination. Markers persist and fade, so goal cadence reads as density."""
        for drone in self.drones:
            sub = pLCMTransport(f"/{drone}/goal_marker")
            sub.start()
            color = DRONE_COLORS.get(drone, "#dddddd")

            def on_marker(raw: str, color: str = color) -> None:
                try:
                    m = json.loads(raw)
                    tx, tz = ros_to_three(float(m["x"]), float(m["y"]))
                    big = m.get("kind") == "destination"
                    self._exec_on(
                        "ghosts", self._client_for("ghosts"),
                        f"""
                        window.__demoGoalMarks = window.__demoGoalMarks || [];
                        const marks = window.__demoGoalMarks;
                        const c = new THREE.Color({json.dumps(color)});
                        const g = new THREE.Group();
                        const mk = (r, w, op) => new THREE.Mesh(
                            new THREE.RingGeometry(r - w, r, 20),
                            new THREE.MeshBasicMaterial({{color: c, transparent: true,
                                opacity: op, depthWrite: false, side: THREE.DoubleSide}}));
                        if ({str(big).lower()}) {{
                            g.add(mk(0.55, 0.07, 0.95));
                            g.add(mk(0.32, 0.05, 0.95));
                        }} else {{
                            g.add(mk(0.22, 0.05, 0.7));
                        }}
                        g.rotation.x = -Math.PI / 2;
                        g.position.set({tx:.2f}, 0.045, {tz:.2f});
                        g.traverse(o => {{ o.userData.__demoIgnoreLOS = true; }});
                        scene.add(g);
                        // Older marks fade so recency is visible; cap the total.
                        for (const old of marks) {{
                            old.traverse(o => {{
                                if (o.material) o.material.opacity *= 0.965;
                            }});
                        }}
                        marks.push(g);
                        if (marks.length > 300) {{
                            const dead = marks.shift();
                            scene.remove(dead);
                        }}
                        return 'ok';
                        """,
                        timeout=5.0,
                    )
                except Exception:
                    logger.warning("goal marker render failed", exc_info=True)

            sub.subscribe(on_marker)
            self._belief_subs.append(sub)

            # Frontier-exploration goals bypass the skill container (the
            # explorer publishes straight to goal_request) — mark those too.
            gsub = LCMTransport(f"/{drone}/goal_request", PoseStamped)
            gsub.start()

            def on_goal(msg: PoseStamped, drone: str = drone) -> None:
                try:
                    if not (math.isfinite(msg.position.x) and math.isfinite(msg.position.y)):
                        return
                    on_marker(json.dumps({"x": msg.position.x, "y": msg.position.y, "kind": "hop"}))
                except Exception:
                    pass

            gsub.subscribe(on_goal)
            self._belief_subs.append(gsub)  # type: ignore[arg-type]

    def start_radio_hud(self) -> None:
        """Mirror non-beacon radio traffic into the on-screen HUD."""
        sub = pLCMTransport("/radio")
        sub.start()

        def on_radio(raw: str) -> None:
            try:
                ev = json.loads(raw)
                if ev.get("kind") == 2:
                    return
                tags = ev.get("tags", [])
                sender = next((t[1] for t in tags if t and t[0] == "sender"), "?")
                content = ev.get("content", "")
                if not content:
                    content = "; ".join(
                        " ".join(str(v) for v in t) for t in tags if t and t[0] != "sender"
                    )
                color = DRONE_COLORS.get(str(sender), "#dddddd")
                self._exec_on(
                    "ghosts", self._client_for("ghosts"),
                    f"if (window.__demoHudRadio) window.__demoHudRadio("
                    f"{json.dumps(str(sender))}, {json.dumps(str(content)[:160])}, "
                    f"{json.dumps(color)});\nreturn 'ok';",
                    timeout=5.0,
                )
            except Exception:
                logger.warning("radio hud update failed", exc_info=True)

        sub.subscribe(on_radio)
        self._belief_subs.append(sub)

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
                    self._exec_on(
                        "ghosts", self._client_for("ghosts"),
                        "\n".join(js) + "\nreturn 'ok';", timeout=5.0,
                    )
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
        // Raycast only against occluder MESHES. Testing scene.children
        // recursively also tests the lidar viz Points cloud, whose cost grows
        // with the map until every tick blows the exec timeout.
        const occluders = [];
        scene.traverse(o => {{
            if (!o.isMesh || o.visible === false || o === t) return;
            let p = o, skip = false;
            while (p) {{
                if (p.userData && p.userData.__demoIgnoreLOS) {{ skip = true; break; }}
                if (p === t) {{ skip = true; break; }}
                p = p.parent;
            }}
            if (!skip) occluders.push(o);
        }});
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
                if (ray.intersectObjects(occluders, false).length > 0) visible = false;
            }}
            out.vis[name] = visible;

            // --- visual feedback: sighting line + wedge highlight ---
            window.__demoLos = window.__demoLos || {{}};
            let lo = window.__demoLos[name];
            if (!lo) {{
                const geo = new THREE.BufferGeometry();
                geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
                const line = new THREE.Line(geo, new THREE.LineBasicMaterial({{
                    color: 0xe11d48, transparent: true, opacity: 0.9 }}));
                line.frustumCulled = false;
                line.userData.__demoIgnoreLOS = true;
                scene.add(line);
                lo = window.__demoLos[name] = {{line}};
            }}
            if (visible) {{
                const a = lo.line.geometry.getAttribute('position');
                a.setXYZ(0, p.x, p.y, p.z);
                a.setXYZ(1, t.position.x, t.position.y, t.position.z);
                a.needsUpdate = true;
                lo.line.visible = true;
            }} else {{
                lo.line.visible = false;
            }}
            const wedge = (window.__demoWedges || {{}})[name];
            if (wedge) wedge.material.opacity = visible ? 0.14 : 0.035;
        }}

        // --- HUD status (director info: positions in mission/ROS coords) ---
        if (window.__demoHudStatus) {{
            const colors = {{'droneA': '#3b82f6', 'droneB': '#f59e0b'}};
            const lines = [];
            for (const name of {drones_js}) {{
                const av = agents.get(name);
                if (!av) continue;
                const p = av.group.position;
                const st = out.vis[name]
                    ? 'TARGET IN SIGHT (' + t.position.z.toFixed(1) + ', ' + t.position.x.toFixed(1) + ')'
                    : 'searching';
                lines.push([colors[name] || '#ccc',
                    name + '  (' + p.z.toFixed(1) + ', ' + p.x.toFixed(1) + ')  ' + st]);
            }}
            lines.push(['#e11d48',
                'target (' + t.position.z.toFixed(1) + ', ' + t.position.x.toFixed(1) + ')']);
            window.__demoHudStatus(lines);
        }}
        return out;
        """
        while not self._stop.is_set():
            t0 = time.time()
            try:
                result = self._exec_on("sensor", self._client_for("sensor"), code, timeout=5.0)
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
                            f"[SENSOR] TARGET IN SIGHT at ({rx:.1f}, {ry:.1f}). Your flight "
                            f"firmware is already intercepting it. Immediately "
                            f"report_sighting({rx:.1f}, {ry:.1f}) so your partner converges too."
                        )
                    else:
                        last = s.last_seen or (rx, ry)
                        self._human_inputs[drone].publish(
                            f"[SENSOR] You LOST sight of the target. Last seen at "
                            f"({last[0]:.1f}, {last[1]:.1f}). Tell your partner and organize a "
                            f"local re-search around that point."
                        )
            else:
                s.streak = 0
            if s.visible:
                s.last_seen = (rx, ry)
                if now - s.last_inject >= cfg.refresh_s:
                    s.last_inject = now
                    self._human_inputs[drone].publish(
                        f"[SENSOR] Target still in sight, now at ({rx:.1f}, {ry:.1f}); firmware "
                        f"pursuit active. Keep your partner updated (report_sighting)."
                    )
            state = f"VISIBLE at ({rx:.1f}, {ry:.1f})" if s.visible else (
                f"NOT VISIBLE (last seen at ({s.last_seen[0]:.1f}, {s.last_seen[1]:.1f}))"
                if s.last_seen else "NOT VISIBLE (never seen)"
            )
            self._target_events[drone].publish(state)
