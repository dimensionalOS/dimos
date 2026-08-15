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

"""Two-drone coordinated search demo launcher.

Boots ONE dimsim world with two physical drones, two fully isolated
dimensional agent stacks (one per drone, local LLM via ollama), the referee
(moving red square + per-drone LOS sensor + belief ghosts), and optionally a
recording browser page (the only page, so it is also the renderer).

Usage:
    uv run python -m dimos.demos.two_drones.run_demo --scenario open
    uv run python -m dimos.demos.two_drones.run_demo --scenario obstacles --record out/obstacles.webm
    uv run python -m dimos.demos.two_drones.run_demo --scenario hide --duration 420

Scenarios: open (no obstacles), obstacles, hide (target weaves behind walls).
"""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.demos.two_drones.referee import Referee, SensorConfig, ros_to_three
from dimos.simulation.dimsim.scene_client import SceneClient
from dimos.utils.deno import ensure_deno
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DRONES = ["droneA", "droneB"]
MCP_PORTS = {"droneA": 9990, "droneB": 9991}
SIM_PORT = 8090

# Arena: 44 m x 70 m (ROS x in [-22, 22], y in [-35, 35]).
ARENA_HALF_X = 22.0
ARENA_HALF_Y = 35.0
CRUISE_ALT = 1.2
# Flyable interior handed to the drones (goals clamped inside).
BOUND_X = ARENA_HALF_X - 0.8
BOUND_Y = ARENA_HALF_Y - 0.8

SPAWNS = {"droneA": (-19.0, -30.0), "droneB": (-19.0, 30.0)}

# Search geometry: radar reach and the planned overlap between the two drones'
# footprints (they aim to stay 2*range - overlap apart).
SENSOR_RANGE_M = 15.0
RADAR_OVERLAP_M = 2.0
# Radio interference cycle: usable for RADIO_ON_S, jammed for RADIO_OFF_S.
# The agents are not told the schedule — they must repeat what matters.
RADIO_ON_S = 10.0
RADIO_OFF_S = 5.0
# Mission success: a drone must get this close to the target.
REACH_RADIUS_M = 1.0

SCENARIOS: dict[str, dict] = {  # type: ignore[type-arg]
    "open": {
        "obstacles": [],
        # The cube patrols most of the arena, so exploring drones cross its
        # path instead of having to reach a far corner it hides in.
        "target_path": [(15, -25), (15, 25), (-12, 25), (-12, -25)],
        # Slow enough that a drone that reaches it can close to < 1 m.
        "target_speed": 0.9,
    },
    "obstacles": {
        "obstacles": [
            ((0.0, -35.0), (0.0, -10.0)),
            ((0.0, 10.0), (0.0, 35.0)),
            ((10.0, -15.0), (10.0, 15.0)),
            ((-10.0, -7.5), (-10.0, 17.5)),
        ],
        "target_path": [(17, -25), (17, 25), (5, 25), (5, -25)],
        "target_speed": 1.4,
    },
    "hide": {
        "obstacles": [
            ((4.0, -27.5), (4.0, 2.5)),
            ((13.0, -5.0), (13.0, 27.5)),
            ((-6.0, 5.0), (-6.0, 30.0)),
            ((-12.0, -30.0), (-12.0, -7.5)),
        ],
        "target_path": [(18, -25), (8, -17.5), (9, 15), (18, 22.5), (9, 15), (8, -17.5)],
        "target_speed": 1.6,
    },
}

MISSION = (
    "MISSION BRIEFING for {name}: find the moving RED TARGET in the arena "
    "x in [-21, 21], y in [-34, 34] (42 x 68 m) and GET WITHIN 1 METRE of it, "
    "together with your partner {partner}. You start at ({sx:.0f}, {sy:.0f}); "
    "your partner starts at ({px:.0f}, {py:.0f}). Your radar reaches 15 m and the "
    "search plan keeps a 2 m overlap between your radar footprint and your "
    "partner's, so you cover different ground. ACT NOW in a single response: "
    "call BOTH claim_sector (your half — don't overlap your partner's claim) "
    "AND begin_coordinated_search. Then react to every [SENSOR] and [RADIO] "
    "message: report sightings repeatedly (the radio suffers interference and "
    "drops messages), and let your firmware fly the intercept."
)


def _port_open(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) == 0


def _wait_for(predicate, timeout: float, what: str) -> None:  # type: ignore[no-untyped-def]
    t0 = time.time()
    while time.time() - t0 < timeout:
        if predicate():
            return
        time.sleep(1.0)
    raise TimeoutError(f"Timed out waiting for {what}")


def start_sim(log_dir: Path) -> subprocess.Popen:  # type: ignore[type-arg]
    deno = ensure_deno()
    cli = DIMOS_PROJECT_ROOT / "misc" / "DimSim" / "cli" / "cli.ts"
    cmd = [
        deno, "run", "--allow-all", "--unstable-net", str(cli), "dev",
        "--scene", "empty", "--port", str(SIM_PORT), "--no-depth",
        # Camera images are unused by this demo (perception is the LOS
        # referee); publishing them costs a render+readback+JPEG per tick.
        "--image-rate", "3600000", "--lidar-rate", "600",
        "--robots", ",".join(DRONES),
    ]
    logf = open(log_dir / "sim.log", "w")
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    _wait_for(lambda: _port_open(SIM_PORT), 90, "dimsim bridge port")
    return proc


def open_viewer_page(record_path: Path | None, headed: bool = False):  # type: ignore[no-untyped-def]
    """Open THE single browser page (renderer + optional video recorder).

    ``headed=True`` shows the window live on screen. Do NOT additionally open
    localhost:8090 in another browser: a second engine page re-ships its own
    (wall-less) physics snapshot and wipes the referee-built world.
    """
    from playwright.sync_api import sync_playwright

    pw = sync_playwright().start()
    browser = pw.chromium.launch(
        headless=not headed,
        args=[
            "--enable-webgl", "--enable-webgl2", "--ignore-gpu-blocklist",
            "--enable-gpu", "--use-gl=angle", "--use-angle=metal",
            "--in-process-gpu", "--disable-gpu-sandbox",
        ],
    )
    ctx_kwargs: dict = {"viewport": {"width": 1280, "height": 720}}  # type: ignore[type-arg]
    if record_path is not None:
        record_path.parent.mkdir(parents=True, exist_ok=True)
        ctx_kwargs["record_video_dir"] = str(record_path.parent)
        ctx_kwargs["record_video_size"] = {"width": 1280, "height": 720}
    ctx = browser.new_context(**ctx_kwargs)
    page = ctx.new_page()
    page.goto(f"http://localhost:{SIM_PORT}", wait_until="domcontentloaded")
    return pw, browser, ctx, page


def start_stack(name: str, log_dir: Path, model: str) -> subprocess.Popen:  # type: ignore[type-arg]
    env = os.environ.copy()
    env.update(
        DRONE_NAME=name,
        DRONE_BOUND_X=str(BOUND_X),
        DRONE_BOUND_Y=str(BOUND_Y),
        DRONE_SENSOR_RANGE=str(SENSOR_RANGE_M),
        DRONE_RADAR_OVERLAP=str(RADAR_OVERLAP_M),
        RADIO_ON_S=str(RADIO_ON_S),
        RADIO_OFF_S=str(RADIO_OFF_S),
        DRONE_MCP_PORT=str(MCP_PORTS[name]),
        # Workers re-read GlobalConfig from the environment, so the MCP port
        # must travel via env (the --mcp-port flag stays in the coordinator).
        MCP_PORT=str(MCP_PORTS[name]),
        DRONE_MODEL=model,
        DIMOS_SKIP_SYSTEM_CONFIG="1",
        DIMOS_TRANSPORT="lcm",
    )
    logf = open(log_dir / f"stack_{name}.log", "w")
    proc = subprocess.Popen(
        ["uv", "run", "dimos", "--transport=lcm", "--simulation", "dimsim",
         "--dimsim-external", "--mcp-port", str(MCP_PORTS[name]), "run", "dimsim-drone"],
        stdout=logf, stderr=subprocess.STDOUT, env=env,
        start_new_session=True,
        cwd=str(DIMOS_PROJECT_ROOT),
    )
    return proc


def stop_process_group(proc: subprocess.Popen) -> None:  # type: ignore[type-arg]
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        proc.terminate()
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            proc.kill()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="open")
    parser.add_argument("--duration", type=float, default=720.0, help="run seconds")
    parser.add_argument("--record", type=str, default="", help="record video to this .webm path")
    parser.add_argument("--headed", action="store_true",
                        help="show the sim window live on screen (can combine with --record)")
    parser.add_argument("--model", type=str, default=os.environ.get("DRONE_MODEL", "ollama:gemma4"))
    parser.add_argument("--log-dir", type=str, default="out/two_drones")
    parser.add_argument("--keep-sim", action="store_true", help="leave the sim running on exit")
    args = parser.parse_args()

    scenario = SCENARIOS[args.scenario]
    log_dir = Path(args.log_dir) / args.scenario
    log_dir.mkdir(parents=True, exist_ok=True)
    record_path = Path(args.record) if args.record else None

    if _port_open(SIM_PORT):
        print(f"ERROR: port {SIM_PORT} already in use — stop the old sim first.")
        sys.exit(1)

    procs: list[subprocess.Popen] = []  # type: ignore[type-arg]
    pw = browser = ctx = page = None
    referee = None
    radio_log: list[str] = []
    try:
        print(f"[demo] starting dimsim ({args.scenario})...")
        sim = start_sim(log_dir)
        procs.append(sim)

        print("[demo] opening viewer/recorder page...")
        pw, browser, ctx, page = open_viewer_page(record_path, headed=args.headed)
        t_video0 = time.time()  # video timeline starts ~here (page created)

        scene = SceneClient(port=SIM_PORT)
        _wait_for(lambda: _try_connect(scene), 60, "scene control channel")

        print("[demo] building world...")
        # Server-side physics only exists after the page ships its Rapier
        # snapshot — teleports sent before that are silently dropped. Wait for
        # odom to flow (physics live), then place the drones and VERIFY.
        latest_odom: dict[str, PoseStamped] = {}
        odom_subs = []
        for name in DRONES:
            sub = LCMTransport(f"/{name}/odom", PoseStamped)
            sub.start()
            sub.subscribe(lambda m, name=name: latest_odom.__setitem__(name, m))
            odom_subs.append(sub)
        _wait_for(lambda: len(latest_odom) == len(DRONES), 120, "server physics odom")

        for name in DRONES:
            scene.set_embodiment(
                "drone", robot=name, max_speed=6.0, turn_rate=3.5, max_altitude=5.0
            )
        time.sleep(1.5)

        def _in_place(name: str) -> bool:
            od = latest_odom.get(name)
            if od is None:
                return False
            sx, sy = SPAWNS[name]
            return (
                abs(od.position.x - sx) < 1.0
                and abs(od.position.y - sy) < 1.0
                and abs(od.position.z - CRUISE_ALT) < 0.5
            )

        for attempt in range(6):
            for name, (sx, sy) in SPAWNS.items():
                if not _in_place(name):
                    tx, tz = ros_to_three(sx, sy)
                    scene.set_agent_position(tx, CRUISE_ALT, tz, robot=name)
            time.sleep(1.5)
            if all(_in_place(n) for n in DRONES):
                break
        else:
            raise RuntimeError(f"drones not at spawn after retries: "
                               f"{ {n: str(latest_odom.get(n)) for n in DRONES} }")
        print("[demo] drones placed at spawns")

        referee = Referee(
            scene,
            DRONES,
            SensorConfig(range_m=SENSOR_RANGE_M, reach_radius_m=REACH_RADIUS_M),
        )
        referee.build_arena(ARENA_HALF_X, ARENA_HALF_Y)
        for (p1, p2) in scenario["obstacles"]:
            referee.add_obstacle_wall(p1[0], p1[1], p2[0], p2[1])
        referee.install_target(scenario["target_path"], speed=scenario["target_speed"])
        referee.mark_drones_for_los()
        referee.install_ghosts()
        referee.style_scene_for_video()
        referee.start_ghost_renderer()
        referee.start_radio_hud()
        referee.start_goal_markers()
        referee.start_link_telemetry()

        # Director camera: high oblique view of the whole arena (render-loop
        # override; player/agent camera modes can't fight it).
        scene.exec(
            "window.__demoDirectorCam = {x: 0, y: 56, z: -26, tx: 0, ty: 0, tz: -2};"
            "return 'director camera set';"
        )

        # Radio traffic console log
        radio_sub = pLCMTransport("/radio")
        radio_sub.start()

        first_radio_ts: list[float] = []

        def _log_radio(raw: str) -> None:
            try:
                ev = json.loads(raw)
                sender = next((t[1] for t in ev.get("tags", []) if t and t[0] == "sender"), "?")
                if ev.get("kind") != 2:
                    if not first_radio_ts:
                        first_radio_ts.append(time.time())
                    line = f"[radio] {sender}: {ev.get('content', '')}"
                    print(line, flush=True)
                    radio_log.append(line)
            except Exception:
                pass

        radio_sub.subscribe(_log_radio)

        print("[demo] launching agent stacks...")
        for name in DRONES:
            procs.append(start_stack(name, log_dir, args.model))

        def _stack_ready(name: str) -> bool:
            f = log_dir / f"stack_{name}.log"
            return f.exists() and "Discovered tools" in f.read_text(errors="ignore")

        _wait_for(lambda: all(_stack_ready(n) for n in DRONES), 180, "agent stacks")
        print("[demo] agents ready — starting sensor + mission")
        referee.start_sensor()
        time.sleep(2.0)

        briefings = {}
        for name in DRONES:
            partner = next(d for d in DRONES if d != name)
            sx, sy = SPAWNS[name]
            px, py = SPAWNS[partner]
            briefings[name] = MISSION.format(
                name=name, partner=partner, sx=sx, sy=sy, px=px, py=py
            )
        inputs = {n: pLCMTransport(f"/{n}/human_input") for n in DRONES}
        for n, t in inputs.items():
            t.start()
        time.sleep(0.5)
        for n, t in inputs.items():
            t.publish(briefings[n])
            print(f"[demo] briefed {n}")

        print(f"[demo] running scenario '{args.scenario}' for {args.duration:.0f}s...")
        t0 = time.time()
        while time.time() - t0 < args.duration:
            time.sleep(10)
            tp = referee.target_pos_ros
            if tp:
                dists = " ".join(
                    f"{d}={referee._dist.get(d, float('nan')):.0f}m" for d in DRONES
                )
                print(
                    f"[demo] t={time.time() - t0:6.0f}s target ({tp[0]:.1f}, {tp[1]:.1f})  {dists}",
                    flush=True,
                )
            # Mission accomplished — let the pursuit be visible for a bit, then wrap.
            if referee.reached:
                first = min(referee.reached.values())
                if time.time() - first > 60:
                    print("[demo] mission accomplished — wrapping up", flush=True)
                    break

        print("[demo] done. radio messages exchanged:", len(radio_log))
        if referee.reached:
            for drone, ts in sorted(referee.reached.items(), key=lambda kv: kv[1]):
                print(
                    f"[demo] MISSION: {drone} reached the target at "
                    f"t+{ts - referee.started_at:.0f}s",
                    flush=True,
                )
        else:
            print("[demo] MISSION NOT ACCOMPLISHED: nobody got within "
                  f"{REACH_RADIUS_M:.1f} m", flush=True)
        # Suggest a head-trim so the published video starts just before the
        # first radio transmission instead of the boot/briefing dead time.
        if first_radio_ts:
            trim = max(0.0, first_radio_ts[0] - t_video0 - 20.0)
            (log_dir / "trim.json").write_text(json.dumps({"head_trim_s": round(trim, 1)}))
            print(f"[demo] suggested head trim: {trim:.0f}s (saved to trim.json)")

    finally:
        print("[demo] shutting down...")
        if referee:
            try:
                referee.stop()
            except Exception:
                pass
        # Save the video FIRST — anything below that hangs must not cost us
        # the recording.
        if ctx is not None:
            try:
                ctx.close()
                if record_path is not None and page is not None:
                    video = page.video
                    if video:
                        video.save_as(str(record_path))
                        print(f"[demo] video saved: {record_path}")
            except Exception:
                logger.warning("video save failed", exc_info=True)
        for proc in procs[1:]:
            stop_process_group(proc)
        if browser is not None:
            try:
                browser.close()
            except Exception:
                pass
        if pw is not None:
            try:
                pw.stop()
            except Exception:
                pass
        if not args.keep_sim and procs:
            procs[0].terminate()
            try:
                procs[0].wait(timeout=10)
            except subprocess.TimeoutExpired:
                procs[0].kill()
        print("[demo] bye")


def _try_connect(scene: SceneClient) -> bool:
    try:
        scene.start()
        scene.get_scene_info()
        return True
    except Exception:
        return False


if __name__ == "__main__":
    main()
