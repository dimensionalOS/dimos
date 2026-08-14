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
from dimos.core.transport import pLCMTransport
from dimos.demos.two_drones.referee import Referee, SensorConfig, ros_to_three
from dimos.simulation.dimsim.scene_client import SceneClient
from dimos.utils.deno import ensure_deno
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DRONES = ["droneA", "droneB"]
MCP_PORTS = {"droneA": 9990, "droneB": 9991}
SIM_PORT = 8090

ARENA_HALF_X = 11.0  # ROS x extent: [-11, 11]
ARENA_HALF_Y = 7.0  # ROS y extent: [-7, 7]
CRUISE_ALT = 1.2

SPAWNS = {"droneA": (-9.0, -5.0), "droneB": (-9.0, 5.0)}

SCENARIOS: dict[str, dict] = {  # type: ignore[type-arg]
    "open": {
        "obstacles": [],
        "target_path": [(8, -4), (8, 4), (2, 4), (2, -4)],
        "target_speed": 0.5,
    },
    "obstacles": {
        "obstacles": [
            ((0.0, -7.0), (0.0, -2.0)),
            ((0.0, 2.0), (0.0, 7.0)),
            ((5.0, -3.0), (5.0, 3.0)),
            ((-5.0, -1.5), (-5.0, 3.5)),
        ],
        "target_path": [(8.5, -5), (8.5, 5), (2.5, 5), (2.5, -5)],
        "target_speed": 0.5,
    },
    "hide": {
        "obstacles": [
            ((2.0, -5.5), (2.0, 0.5)),
            ((6.5, -1.0), (6.5, 5.5)),
            ((-3.0, 1.0), (-3.0, 6.0)),
            ((-6.0, -6.0), (-6.0, -1.5)),
        ],
        "target_path": [(9, -5), (4, -3.5), (4.5, 3), (9, 4.5), (4.5, 3), (4, -3.5)],
        "target_speed": 0.65,
    },
}

MISSION = (
    "MISSION BRIEFING for {name}: You and your partner {partner} must find and "
    "then keep pursuing a moving RED TARGET somewhere in the arena "
    "x in [-11, 11], y in [-7, 7]. You start at ({sx:.0f}, {sy:.0f}); your partner "
    "starts at ({px:.0f}, {py:.0f}). There may be walls; your planner avoids them. "
    "Plan: 1) claim_sector for YOUR half of the arena over the radio (split it "
    "fairly, don't overlap your partner), 2) sweep_area your sector, 3) react "
    "immediately to [SENSOR] and [RADIO] messages. Begin now."
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
        "--image-rate", "500", "--lidar-rate", "200",
        "--robots", ",".join(DRONES),
    ]
    logf = open(log_dir / "sim.log", "w")
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    _wait_for(lambda: _port_open(SIM_PORT), 90, "dimsim bridge port")
    return proc


def open_viewer_page(record_path: Path | None):  # type: ignore[no-untyped-def]
    """Open THE single browser page (renderer + optional video recorder)."""
    from playwright.sync_api import sync_playwright

    pw = sync_playwright().start()
    browser = pw.chromium.launch(
        headless=True,
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
        DRONE_MCP_PORT=str(MCP_PORTS[name]),
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
    parser.add_argument("--duration", type=float, default=360.0, help="run seconds")
    parser.add_argument("--record", type=str, default="", help="record video to this .webm path")
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
        pw, browser, ctx, page = open_viewer_page(record_path)

        scene = SceneClient(port=SIM_PORT)
        _wait_for(lambda: _try_connect(scene), 60, "scene control channel")

        print("[demo] building world...")
        for name in DRONES:
            scene.set_embodiment(
                "drone", robot=name, max_speed=2.0, turn_rate=2.5, max_altitude=5.0
            )
        time.sleep(1.0)
        for name, (sx, sy) in SPAWNS.items():
            tx, tz = ros_to_three(sx, sy)
            scene.set_agent_position(tx, CRUISE_ALT, tz, robot=name)

        referee = Referee(scene, DRONES, SensorConfig())
        referee.build_arena(ARENA_HALF_X, ARENA_HALF_Y)
        for (p1, p2) in scenario["obstacles"]:
            referee.add_obstacle_wall(p1[0], p1[1], p2[0], p2[1])
        referee.install_target(scenario["target_path"], speed=scenario["target_speed"])
        referee.mark_drones_for_los()
        referee.install_ghosts()
        referee.start_ghost_renderer()

        # Director camera: high oblique view of the whole arena.
        scene.exec(
            "camera.position.set(0, 18, -20); camera.lookAt(0, 0, 0);"
            "if (typeof controls !== 'undefined' && controls) controls.enabled = false;"
            "return 'camera set';"
        )

        # Radio traffic console log
        radio_sub = pLCMTransport("/radio")
        radio_sub.start()

        def _log_radio(raw: str) -> None:
            try:
                ev = json.loads(raw)
                sender = next((t[1] for t in ev.get("tags", []) if t and t[0] == "sender"), "?")
                if ev.get("kind") != 2:
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
            time.sleep(5)
            tp = referee.target_pos_ros
            if tp:
                print(f"[demo] t={time.time() - t0:6.0f}s target at ({tp[0]:.1f}, {tp[1]:.1f})", flush=True)

        print("[demo] done. radio messages exchanged:", len(radio_log))

    finally:
        print("[demo] shutting down...")
        if referee:
            try:
                referee.stop()
            except Exception:
                pass
        for proc in procs[1:]:
            stop_process_group(proc)
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
