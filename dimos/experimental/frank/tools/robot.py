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

"""FRANK's hands on the Go2: posture, camera, motion, through the running DimOS instance.

Posture goes straight to the Unitree sport API over the GO2Connection RPC. Motion and
camera go through the McpServer so they get the same navigation stack the agent uses.
Nothing here runs on the robot.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import requests
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.porcelain.dimos import Dimos

MAX_PITCH_DEG = 40.0  # Unitree documents about ±0.75 rad for Euler
MOTION_OFF = (
    Path(__file__).resolve().parents[1] / "cache" / "MOTION_OFF"
)  # operator switch: exists = no motion (ops dashboard toggle)
_app: Dimos | None = None


def _require_move_ok(what: str) -> None:
    if MOTION_OFF.exists():
        raise SystemExit(
            f"movement is off: {what} refused. The operator switched motion off on the dashboard; ask them."
        )


def _dimos() -> Dimos:
    global _app
    if _app is None:
        _app = Dimos.connect(timeout=10.0)
    return _app


def close() -> None:
    global _app
    if _app is not None:
        _app.stop()
        _app = None


# --- posture -------------------------------------------------------------------


def _sport_raw(name: str, parameter: dict[str, Any] | None = None) -> Any:
    data: dict[str, Any] = {"api_id": SPORT_CMD[name]}
    if parameter is not None:
        data["parameter"] = parameter
    return _dimos().GO2Connection.publish_request(RTC_TOPIC["SPORT_MOD"], data)


def sport(name: str, parameter: dict[str, Any] | None = None) -> Any:
    """Send a SPORT_MOD request by Unitree command name, with an optional parameter dict."""
    _require_move_ok(f"sport {name}")
    return _sport_raw(name, parameter)


def tilt(pitch_deg: float, roll_deg: float = 0.0, yaw_deg: float = 0.0) -> Any:
    """Nod the body to an attitude. Negative pitch is nose-up (verified on hardware 2026-09-03).

    On firmware 1.1.x this is momentary: the body tilts and settles back level within a second,
    even in Pose mode and BalanceStand. Use sit() when you need a held face-height view.
    Only while standing still. Call level() before any move.
    """
    pitch_deg = max(-MAX_PITCH_DEG, min(MAX_PITCH_DEG, pitch_deg))
    return sport(
        "Euler",
        {"x": math.radians(roll_deg), "y": math.radians(pitch_deg), "z": math.radians(yaw_deg)},
    )


def level() -> Any:
    """Back to flat. Always allowed: it is the safe posture."""
    return _sport_raw("Euler", {"x": 0.0, "y": 0.0, "z": 0.0})


def pose_mode(enable: bool) -> Any:
    """Enter/leave pose mode. Try this if tilt() does not hold the attitude."""
    return sport("Pose", {"flag": enable})


def sit() -> Any:
    return sport("Sit")


def rise() -> Any:
    return sport("RiseSit")


# --- sensing -------------------------------------------------------------------


def pose() -> tuple[float, float, float]:
    """World x, y (m) and yaw (deg) from odometry."""
    od = _dimos().peek_stream("odom", timeout=2.0)
    yaw = float(od.yaw)
    yaw = math.degrees(yaw) if abs(yaw) < 7 else yaw
    return float(od.position.x), float(od.position.y), yaw


def observe(out: str = "observe.jpg", timeout: int = 20) -> str:
    """Grab one camera frame via the observe skill and write it as JPEG. Returns the path."""
    result = McpAdapter(timeout=timeout).call_tool("observe", {})
    for item in result.get("content", []):
        if item.get("type") == "image" and item.get("data"):
            data = item["data"]
        elif item.get("type") == "image_url":
            data = item["image_url"]["url"].split(",", 1)[1]
        else:
            continue
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        with open(out, "wb") as f:
            f.write(base64.b64decode(data))
        return out
    raise RuntimeError(f"observe returned no image: {result}")


# --- motion --------------------------------------------------------------------


def move(
    x: float, y: float, degrees: float | None = None, relative: bool = False, timeout: int = 180
) -> str:
    """move_to through the McpServer. Blocks; returns the outcome text and final pose.
    If it is not 'goal reached', call stop() and poll pose() until the robot is still."""
    _require_move_ok("move")
    args: dict[str, Any] = {"x": x, "y": y, "relative": relative}
    if degrees is not None:
        args["degrees"] = degrees
    return str(call_skill("move_to", args))


def skill_arguments(name: str, raw: str) -> dict[str, Any]:
    """Accept JSON objects or a scalar for a skill's single required argument."""
    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        value = raw
    if isinstance(value, dict):
        return value
    tool = next((tool for tool in McpAdapter().list_tools() if tool["name"] == name), None)
    if tool is None:
        raise ValueError(f"Unknown skill: {name}. Run robot.py skills to list available tools.")
    schema = tool.get("inputSchema", {})
    required = schema.get("required", [])
    if len(required) != 1:
        raise ValueError(f"{name} needs a JSON object with named arguments; see robot.py skills.")
    return {required[0]: value}


def call_skill(name: str, arguments: dict[str, Any]) -> dict[str, Any]:
    """Call any exposed Go2 skill, preserving the operator motion switch."""
    if name == "execute_sport_command":
        command = arguments.get("command_name")
        if not isinstance(command, str) or command not in SPORT_CMD:
            return {
                "isError": True,
                "content": [
                    {
                        "type": "text",
                        "text": (
                            f"Unknown sport command {command!r}. For a relative turn use robot.py turn DEGREES "
                            "(right 20 degrees: robot.py turn -20; left: positive). "
                            "Run robot.py skills for the sport command catalog."
                        ),
                    }
                ],
            }
    if name in {
        "move_to",
        "navigate_with_text",
        "follow_person",
        "begin_exploration",
        "start_patrol",
        "execute_sport_command",
        "look_out_for",
    }:
        _require_move_ok(name)
    if name in {
        "end_exploration",
        "stop_patrol",
        "stop_following",
        "stop_looking_out",
        "stop_navigation",
    }:
        return McpAdapter(timeout=20).call_tool(name, arguments)
    url = os.environ.get("FRANK_URL", "http://127.0.0.1:7790").rstrip("/") + "/agent/operations"
    headers = {}
    if os.environ.get("FRANK_AGENT_TOKEN"):
        headers["Authorization"] = "Bearer " + os.environ["FRANK_AGENT_TOKEN"]
    token = os.environ.get("FRANK_OPERATION_TOKEN")
    managed = token is not None
    if token is None:
        registered = requests.post(
            url, json={"action": "start", "tool": name}, headers=headers, timeout=5
        )
        registered.raise_for_status()
        token = registered.json()["token"]
    failed = True
    result: dict[str, Any] = {}
    try:
        adapter = McpAdapter(timeout=180)
        result = adapter._unwrap(
            adapter.call(
                "tools/call",
                {
                    "name": name,
                    "arguments": arguments,
                    "_meta": {"progressToken": token},
                },
            )
        )
        if name == "execute_sport_command" and any(
            item.get("type") == "text"
            and item.get("text", "").startswith(("There's no '", "Failed to execute"))
            for item in result.get("content", [])
        ):
            result["isError"] = True
        for item in result.get("content", []):
            if item.get("type") == "text":
                try:
                    outcome = json.loads(item["text"])
                except (ValueError, KeyError):
                    continue
                if isinstance(outcome, dict) and outcome.get("success") is False:
                    result["isError"] = True
        failed = bool(result.get("isError"))
        return result
    finally:
        try:
            if managed:
                pass  # The service owns completion, including subprocess failures.
            else:
                reported = requests.post(
                    url,
                    json={
                        "action": "finish",
                        "token": token,
                        "failed": failed,
                        "result": json.dumps(result),
                    },
                    headers=headers,
                    timeout=5,
                )
                reported.raise_for_status()
        except requests.RequestException as exc:
            print(f"Could not report operation status: {exc}", file=sys.stderr)


def stop() -> str:
    """Stop background movement owners before cancelling the current goal."""
    direct_errors: list[str] = []
    try:
        _sport_raw("StopMove")
    except Exception as exc:
        direct_errors.append(f"StopMove: {exc}")
    adapter = McpAdapter(timeout=20)
    results: list[str] = []
    errors: list[str] = []
    for name in (
        "end_exploration",
        "stop_patrol",
        "stop_following",
        "stop_looking_out",
        "stop_navigation",
    ):
        try:
            result = adapter.call_tool(name, {})
            if result.get("isError"):
                errors.append(f"{name}: {result}")
            results.append(str(result))
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    try:
        _sport_raw("StopMove")
    except Exception as exc:
        errors.extend(direct_errors or [f"StopMove: {exc}"])
    if errors:
        raise RuntimeError("; ".join(errors))
    return "\n".join(results)


def _wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def face(yaw_deg: float | None = None, at: tuple[float, float] | None = None) -> str:
    """Turn in place to an absolute world heading (0 = east, 90 = north), or to face the world
    point `at`. Same closed loop as turn(); reports the heading actually reached."""
    x, y, yaw = pose()
    if at is not None:
        yaw_deg = math.degrees(math.atan2(at[1] - y, at[0] - x))
    if yaw_deg is None:
        raise SystemExit("face needs a heading in degrees or --at x y")
    return turn(_wrap(yaw_deg - yaw))


def turn(degrees: float, tolerance: float = 3.0, timeout: float = 25.0) -> str:
    """Turn in place by `degrees` relative to where the nose points now: positive = left
    (counter-clockwise, yaw increases), negative = right. Watches odometry until within `tolerance`.

    The navigator's turn (`move --degrees`) stops as soon as it is within 20 degrees of the
    heading it was given, so a 90 becomes a 70. This drives the body's yaw rate directly and
    keeps going until odometry agrees, then reports what actually happened."""
    _require_move_ok("turn")
    if not math.isfinite(degrees):
        raise ValueError("turn requires a finite angle in degrees")
    stop()  # release background movement owners before driving yaw directly
    x0, y0, yaw0 = pose()
    target = yaw0 + degrees
    conn = _dimos().GO2Connection
    deadline = time.monotonic() + timeout
    err = _wrap(target - yaw0)
    try:
        while abs(err) > tolerance and time.monotonic() < deadline:
            _require_move_ok("turn")
            rate = math.copysign(min(0.9, max(0.25, math.radians(abs(err)) * 1.2)), err)
            accepted = conn.move(
                Twist(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, rate)), duration=0.2
            )
            if accepted is False:
                raise RuntimeError("Robot rejected the turn velocity command")
            time.sleep(0.15)
            _, _, yaw = pose()
            err = _wrap(target - yaw)
    finally:
        conn.move(Twist(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)), duration=0.1)
    time.sleep(0.3)
    x, y, yaw = pose()
    done = _wrap(yaw - yaw0)
    err = _wrap(target - yaw)
    if abs(err) > tolerance:
        raise RuntimeError(
            f"Turn incomplete: turned {done:+.0f} of {degrees:+.0f} degrees; {err:+.0f} degrees remain"
        )
    return f"turned {done:+.0f} deg of the {degrees:+.0f} asked; now facing yaw={yaw:.0f} deg at x={x:.1f} y={y:.1f}"


# --- cli -----------------------------------------------------------------------


def _main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("tilt", help="hold body pitch in degrees (negative = nose up, verify)")
    t.add_argument("pitch", type=float)
    sub.add_parser("level")
    sub.add_parser("sit")
    sub.add_parser("rise")
    sub.add_parser("pose")
    pm = sub.add_parser("pose-mode")
    pm.add_argument("state", choices=["on", "off"])
    o = sub.add_parser("observe")
    o.add_argument("out", nargs="?", default="observe.jpg")
    s = sub.add_parser("sport", help="any parameterless sport command, e.g. Hello, Stretch")
    s.add_argument("name")
    m = sub.add_parser("move")
    m.add_argument("x", type=float)
    m.add_argument("y", type=float)
    m.add_argument("--degrees", type=float)
    m.add_argument("--relative", action="store_true")
    sk = sub.add_parser("skill", help="call any exposed Go2 skill with JSON arguments")
    sk.add_argument("name")
    sk.add_argument("arguments", nargs="?", default="{}")
    sub.add_parser("skills", help="list live skill descriptions and argument schemas")
    sub.add_parser("stop")
    tn = sub.add_parser(
        "turn",
        help="turn in place by degrees relative to now: positive = left, negative = right, exact to 3 degrees",
    )
    tn.add_argument("degrees", type=float)
    fc = sub.add_parser(
        "face",
        help="turn to an absolute world heading (0 = east, 90 = north), or --at x y to face a world point",
    )
    fc.add_argument("yaw", type=float, nargs="?")
    fc.add_argument("--at", type=float, nargs=2, metavar=("X", "Y"))
    # Exposed MCP names can be used directly as well as through `skill NAME`.
    if argv and not argv[0].startswith("-") and argv[0] not in sub.choices:
        argv = ["skill", *argv]
    a = p.parse_args(argv)

    try:
        if a.cmd == "skills":
            print(json.dumps(McpAdapter().list_tools(), indent=2))
        elif a.cmd == "skill":
            try:
                arguments = skill_arguments(a.name, a.arguments)
            except ValueError as exc:
                p.error(str(exc))
            result = call_skill(a.name, arguments)
            print(json.dumps(result))
            if result.get("isError"):
                return 1
        elif a.cmd == "tilt":
            print(tilt(a.pitch))
        elif a.cmd == "level":
            print(level())
        elif a.cmd == "sit":
            print(sit())
        elif a.cmd == "rise":
            print(rise())
        elif a.cmd == "pose":
            x, y, yaw = pose()
            print(f"x={x:.2f} y={y:.2f} yaw={yaw:.0f}deg")
        elif a.cmd == "pose-mode":
            print(pose_mode(a.state == "on"))
        elif a.cmd == "observe":
            print(observe(a.out))
        elif a.cmd == "sport":
            print(sport(a.name))
        elif a.cmd == "move":
            print(move(a.x, a.y, a.degrees, a.relative))
        elif a.cmd == "stop":
            print(stop())
        elif a.cmd == "turn":
            print(turn(a.degrees))
        elif a.cmd == "face":
            print(face(a.yaw, tuple(a.at) if a.at else None))
    finally:
        close()
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
