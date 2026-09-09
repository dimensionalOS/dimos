#!/usr/bin/env python3
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

"""Start, stop, or check the whole FRANK stack with one command.

    uv run python dimos/experimental/frank/up.py            # bring everything up, open the dashboard, run Frank
    uv run python dimos/experimental/frank/up.py --rerun    # same, plus the Rerun viewer (lidar, map, camera, poses)
    uv run python dimos/experimental/frank/up.py status     # what is running
    uv run python dimos/experimental/frank/up.py down       # stop Frank, watcher, server, robot stack
    uv run python dimos/experimental/frank/up.py down --wipe  # also delete app/data (end of day)

Order on the way up: robot stack -> chat server -> public funnel -> face watcher
-> dashboard in the browser -> Frank's loop in the foreground (ctrl-c stops only the loop; the rest
keeps running so you can restart Frank without restarting the robot).
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from typing import Any

from dotenv import load_dotenv
import psutil
import requests

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
CACHE = HERE / "cache"
SKILL = str(HERE.relative_to(REPO))

load_dotenv(HERE / ".env")

ROBOT_IP = os.environ.get("FRANK_ROBOT_IP", "10.0.0.79")
PORT = int(os.environ.get("FRANK_PORT", "7790"))
MCP_PORT = 9990


def funnel_url() -> str | None:
    """The public HTTPS address Tailscale Funnel serves for this machine.

    FRANK_FUNNEL_URL overrides it; otherwise it is derived from this machine's tailnet name.
    """
    if url := os.environ.get("FRANK_FUNNEL_URL"):
        return url.rstrip("/")
    if not shutil.which("tailscale"):
        return None
    try:
        r = subprocess.run(
            ["tailscale", "status", "--json"], capture_output=True, text=True, timeout=10
        )
        name = json.loads(r.stdout)["Self"]["DNSName"].rstrip(".")
    except (subprocess.SubprocessError, ValueError, KeyError):
        return None
    return f"https://{name}" if name else None


FUNNEL_URL = (
    funnel_url() or "https://<machine>.<tailnet>.ts.net (tailscale status --json shows the name)"
)
SERVER_URL = f"http://127.0.0.1:{PORT}"
BLUEPRINT = "unitree-go2-agentic"
# What the run records to recordings/<run-id>/memory.db for Frank's recall.py: camera, pose, lidar.
# global_map and global_costmap are left out; they republish whole maps and were 1 GB per 5 min.
RECORD_TOPICS = "color_image,odom,tf,lidar"
ENV = {
    **os.environ,
    "DIMOS_TRANSPORT": "lcm",
}
# Vision models for the stack. Unset means the DimOS defaults (local Qwen for VLM queries,
# local Moondream for the look-out detector). Set both to "cerebras" to use hosted Gemma;
# that also needs CEREBRAS_API_KEY in dimos/experimental/frank/.env.
VL_MODEL = os.environ.get("FRANK_VL_MODEL")
DETECTION_MODEL = os.environ.get("FRANK_DETECTION_MODEL")

MARKS = {"ok": "\033[32m✓\033[0m", "bad": "\033[31m✗\033[0m", "wait": "\033[33m…\033[0m"}


def say(mark: str, msg: str) -> None:
    print(f"  {MARKS[mark]} {msg}", flush=True)


def sh(*args: str, timeout: float = 60.0, check: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(args), cwd=REPO, env=ENV, capture_output=True, text=True, timeout=timeout, check=check
    )


def spawn(cmd: list[str], log: str) -> int:
    """Detach a long-running piece, logging to cache/<log>. Returns its pid."""
    CACHE.mkdir(exist_ok=True)
    out = open(CACHE / log, "ab")
    p = subprocess.Popen(
        cmd,
        cwd=REPO,
        env=ENV,
        stdout=out,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )
    return p.pid


def pids_of(pattern: str) -> list[int]:
    r = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    return [int(x) for x in r.stdout.split() if int(x) != os.getpid()]


def port_open(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def wait_for(what: str, pred: Any, timeout: float, every: float = 1.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(every)
    say("bad", f"{what}: gave up after {timeout:.0f}s")
    return False


# ---- the pieces -------------------------------------------------------------------------------


def robot_reachable() -> bool:
    return (
        subprocess.run(["ping", "-c", "1", "-W", "1", ROBOT_IP], capture_output=True).returncode
        == 0
    )


def dimos_running() -> bool:
    return "Run ID" in sh("uv", "run", "dimos", "status").stdout


def robot_stack_pids() -> list[int]:
    """Find this robot's daemons even when the active-run registry lost one."""
    found = []
    for process in psutil.process_iter(["pid", "cmdline"]):
        argv = process.info["cmdline"] or []
        if BLUEPRINT in argv and "--daemon" in argv and ROBOT_IP in argv:
            found.append(process.pid)
    return found


def up_robot(rerun: bool = False) -> bool:
    pids = robot_stack_pids()
    registered = dimos_running()
    if len(pids) > 1 or (pids and not registered):
        say(
            "bad",
            f"Unclean robot stack state (PIDs {pids}); refusing a duplicate launch. Run up.py down first.",
        )
        return False
    if registered:
        say(
            "ok",
            "robot stack already running; VLM settings require a stack restart if it predates this configuration",
        )
        if rerun:
            up_rerun()
        return True
    if not robot_reachable():
        say("bad", f"robot {ROBOT_IP} does not answer ping; is it on and on this network?")
        return False
    say("wait", f"starting {BLUEPRINT} on {ROBOT_IP} (robot will stand up; ~1 min)")
    # the daemon keeps the launcher's stdout open, so don't wait on the launcher: log it and poll
    viewer = "rerun" if rerun else "none"
    spawn(
        [
            "uv",
            "run",
            "dimos",
            "--transport",
            "lcm",
            "--viewer",
            viewer,
            *(["--vl-model", VL_MODEL] if VL_MODEL else []),
            *(["--detection-model", DETECTION_MODEL] if DETECTION_MODEL else []),
            "--record",
            "--record-topics",
            RECORD_TOPICS,
            "run",
            BLUEPRINT,
            "--robot-ip",
            ROBOT_IP,
            "--daemon",
        ],
        "launch.log",
    )
    ok = wait_for("MCP server", lambda: port_open(MCP_PORT), 150, every=2.0)
    if not ok:
        tail = (CACHE / "launch.log").read_text(errors="ignore")[-1200:]
        say("bad", f"see cache/launch.log:\n{tail}")
    if ok:
        say("ok", f"robot stack up, MCP on {MCP_PORT}")
    return ok


def up_rerun() -> None:
    """The Rerun viewer against a stack that is already running (the bridge is a separate process).
    Costs memory over hours: the bridge grew ~100 MB/min with the viewer on. Close it when done."""
    if pids_of(r"dimos rerun-bridge"):
        say("ok", "rerun bridge already running")
        return
    spawn(["uv", "run", "dimos", "rerun-bridge"], "rerun.log")
    say(
        "ok",
        "rerun viewer starting (cache/rerun.log); manual driving is the Command Center, http://127.0.0.1:7779/command-center",
    )


def server_up() -> bool:
    try:
        return requests.get(SERVER_URL + "/", timeout=2).status_code == 200
    except requests.RequestException:
        return False


def up_server() -> bool:
    if server_up():
        say("ok", f"chat server already on {PORT}")
        return True
    spawn([sys.executable, "-u", str(HERE / "app" / "server.py")], "server.log")
    ok = wait_for("chat server", server_up, 20)
    if ok:
        say("ok", f"chat server on {PORT}")
    return ok


def funnel_on() -> bool:
    r = subprocess.run(
        ["tailscale", "funnel", "status"], capture_output=True, text=True, timeout=20
    )
    return "Funnel on" in r.stdout


def up_funnel() -> bool:
    if not shutil.which("tailscale"):
        say("bad", "tailscale not installed; phones cannot reach the app")
        return False
    if funnel_on():
        say("ok", f"public URL {FUNNEL_URL}")
        return True
    r = subprocess.run(
        ["tailscale", "funnel", "--bg", str(PORT)], capture_output=True, text=True, timeout=30
    )
    if funnel_on():
        say("ok", f"public URL {FUNNEL_URL}")
        return True
    say("bad", f"funnel did not start: {(r.stdout + r.stderr).strip()[-300:]}")
    return False


def watcher_pids() -> list[int]:
    return pids_of(r"frank/watch\.py")


def up_watcher() -> bool:
    if watcher_pids():
        say("ok", "face watcher already running")
        return True
    spawn(
        [sys.executable, "-u", str(HERE / "tools" / "watch.py"), "--source", "robot"], "watch.log"
    )
    ok = wait_for(
        "face watcher",
        lambda: bool(watcher_pids())
        and "watching" in (CACHE / "watch.log").read_text(errors="ignore")[-2000:],
        40,
    )
    if ok:
        say("ok", "face watcher on the robot camera")
    return ok


def motion_on() -> bool:
    """Motion is on unless the dashboard's switch file exists."""
    return not (CACHE / "MOTION_OFF").exists()


def open_dashboard() -> None:
    url = SERVER_URL + "/ops"
    if shutil.which("xdg-open"):
        subprocess.Popen(["xdg-open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        say("ok", f"dashboard {url}")
    else:
        say("ok", f"dashboard {url} (open it yourself)")


def run_loop(harness: str, model: str | None, session: str | None) -> int:
    cmd = [sys.executable, "-u", str(HERE / "loop.py"), "--harness", harness]
    if model:
        cmd += ["--model", model]
    if session:
        cmd += ["--session", session]
    print("\n  Frank is listening. ctrl-c stops him; everything else keeps running.\n", flush=True)
    try:
        return subprocess.run(cmd, cwd=REPO, env=ENV).returncode
    except KeyboardInterrupt:
        return 0


# ---- commands ---------------------------------------------------------------------------------


def cmd_up(args: argparse.Namespace) -> int:
    print("FRANK up", flush=True)
    if not up_robot(rerun=args.rerun):
        return 1
    if not up_server():
        return 1
    up_funnel()  # phones only; not fatal for a local test
    up_watcher()
    say(
        "ok" if motion_on() else "bad",
        "motion on" if motion_on() else "motion OFF (dashboard toggle)",
    )
    if not args.no_browser:
        open_dashboard()
    if args.no_loop:
        say("ok", "loop not started (--no-loop)")
        return 0
    return run_loop(args.harness, args.model, args.session)


def kill(pattern: str, label: str) -> None:
    pids = pids_of(pattern)
    for p in pids:
        try:
            os.kill(p, signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(1)
    for p in pids_of(pattern):
        try:
            os.kill(p, signal.SIGKILL)
        except ProcessLookupError:
            pass
    say("ok", f"{label} stopped" if pids else f"{label} was not running")


def cmd_down(args: argparse.Namespace) -> int:
    print("FRANK down", flush=True)
    kill(r"frank/loop\.py", "Frank's loop")
    kill(r"frank/watch\.py", "face watcher")
    kill(r"frank/app/server\.py", "chat server")
    kill(r"dimos rerun-bridge", "rerun bridge")
    roots = [psutil.Process(pid) for pid in robot_stack_pids()]
    descendants = []
    for process in roots:
        try:
            descendants.extend(process.children(recursive=True))
        except psutil.NoSuchProcess:
            pass
    if dimos_running():
        stopped = sh("uv", "run", "dimos", "stop", timeout=60)
        if stopped.returncode:
            say("bad", stopped.stderr.strip() or stopped.stdout.strip())
        say("ok", "robot stack shutdown requested")
    else:
        say("ok", "robot stack was not running")
    # A timed-out coordinator shutdown can leave live RPC workers on the bus.
    for process in [*reversed(descendants), *roots]:
        try:
            process.kill()
        except psutil.NoSuchProcess:
            pass
    psutil.wait_procs([*descendants, *roots], timeout=5)
    if args.wipe:
        shutil.rmtree(HERE / "app" / "data", ignore_errors=True)
        say("ok", "app/data wiped")
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    print("FRANK status", flush=True)
    say("ok" if dimos_running() else "bad", f"robot stack ({BLUEPRINT})")
    say("ok" if server_up() else "bad", f"chat server {SERVER_URL}")
    try:
        say("ok" if funnel_on() else "bad", f"public URL {FUNNEL_URL}")
    except Exception:
        say("bad", "tailscale unavailable")
    say("ok" if watcher_pids() else "bad", "face watcher")
    say("ok" if pids_of(r"frank/loop\.py") else "bad", "Frank's loop")
    say(
        "ok" if motion_on() else "bad",
        "motion on" if motion_on() else "motion OFF (dashboard toggle)",
    )
    dbs = sorted((REPO / "recordings").glob("*/memory.db"), key=lambda p: p.stat().st_mtime)
    if dbs and time.time() - dbs[-1].stat().st_mtime < 120:
        say("ok", f"recording {dbs[-1].relative_to(REPO)} ({dbs[-1].stat().st_size / 1e9:.1f} GB)")
    else:
        say(
            "bad",
            "no live recording (stack started without --record?); Frank has no memory to recall",
        )
    if server_up():
        try:
            rows = requests.get(SERVER_URL + "/agent/world", timeout=3).json().get("people", [])
            say(
                "ok",
                f"{len(rows)} people known, {sum(1 for r in rows if r.get('in_view'))} in view",
            )
        except Exception:
            pass
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest="cmd")
    u = sub.add_parser("up", help="start everything (default)")
    u.add_argument("--harness", default="pi")
    u.add_argument("--model")
    u.add_argument("--session", help="loop session id; default frank-YYYYMMDD")
    u.add_argument(
        "--rerun",
        action="store_true",
        help="also open the Rerun viewer (memory grows over hours; close it when done)",
    )
    u.add_argument("--no-browser", action="store_true")
    u.add_argument("--no-loop", action="store_true", help="bring up the stack but don't run Frank")
    d = sub.add_parser("down", help="stop everything")
    d.add_argument("--wipe", action="store_true", help="also delete app/data")
    sub.add_parser("status")
    argv = sys.argv[1:]
    if not argv or argv[0].startswith("-"):
        argv = ["up", *argv]
    args = p.parse_args(argv)
    return {"up": cmd_up, "down": cmd_down, "status": cmd_status}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
