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

"""PACK MIND — keyboard demo driver for one live Go2 running unitree-go2-pack.

Drive the dog with the keyboard; after every move (when it stops) it AUTO-runs
``look_for_red`` and, on a find, AUTO-``speak``s the announcement. One operator,
hands on the keyboard, zero command typing during the demo.

    uv run python -m dimos.experimental.pack_mind.demo_drive

Keys:
    w / s   forward / back (one step)
    a / d   turn left / right
    space   look for red now
    f       speak "red object found"
    +/-     bigger / smaller step
    x       quit

Movement uses the ``relative_move`` skill, which blocks until the dog stops — so
"look on stop" is just: issue the move, then look the moment it returns.
"""

from __future__ import annotations

import argparse
import sys
import termios
import tty

from dimos.agents.mcp.mcp_adapter import McpAdapter

_FOUND_MARK = "i see a red object"
_FOUND_SPEECH = "red object found"


def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _call(adapter: McpAdapter, name: str, args: dict[str, object] | None = None) -> str:
    try:
        return adapter.call_tool_text(name, args or {})
    except Exception as exc:  # never let one flaky call kill the driver
        return f"[{name} error: {exc}]"


def _look_and_announce(adapter: McpAdapter) -> None:
    result = _call(adapter, "look_for_red")
    print(f"  👁  {result}")
    if _FOUND_MARK in result.lower():
        print(f"  📣  {_call(adapter, 'speak', {'text': _FOUND_SPEECH})}")


def main() -> None:
    parser = argparse.ArgumentParser(description="PACK MIND keyboard driver")
    parser.add_argument("--url", default=None, help="MCP URL (default localhost:9990/mcp)")
    parser.add_argument("--step", type=float, default=0.5, help="forward speed (m/s)")
    parser.add_argument("--turn", type=float, default=0.8, help="turn speed (rad/s)")
    parser.add_argument(
        "--no-auto-look", action="store_true", help="don't auto look_for_red after each move"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=8,
        help="per-call seconds. Short on purpose: the dog physically moves in a few "
        "seconds, but relative_move then waits on a flaky LCM RPC (is_goal_reached) "
        "that can hang for 120s — we don't block on it, the dog already moved.",
    )
    args = parser.parse_args()

    adapter = McpAdapter(args.url, timeout=args.timeout)
    step, turn = args.step, args.turn

    print("PACK MIND driver — w/s move · a/d turn · space=look · f=speak · +/- step · x=quit")
    print(f"connected to {adapter.url}  (step={step}m turn={turn}°)\n")

    # Velocity teleop (drive) — instant, bypasses the planner + flaky is_goal_reached
    # RPC. step = m/s forward burst; turn = rad/s; each press moves for `burst` seconds.
    burst = 0.6
    moves = {
        "w": lambda: ("drive", {"forward": step, "turn": 0.0, "duration": burst}),
        "s": lambda: ("drive", {"forward": -step, "turn": 0.0, "duration": burst}),
        "a": lambda: ("drive", {"forward": 0.0, "turn": turn, "duration": burst}),
        "d": lambda: ("drive", {"forward": 0.0, "turn": -turn, "duration": burst}),
    }

    while True:
        key = _getch().lower()
        if key in ("x", "\x03", "\x04", "q"):  # x, Ctrl-C, Ctrl-D, q
            print("\nbye")
            return
        if key in moves:
            name, margs = moves[key]()
            arrow = {"w": "↑", "s": "↓", "a": "↰", "d": "↱"}[key]
            print(f"{arrow}  {_call(adapter, name, margs)}")
            if not args.no_auto_look:
                _look_and_announce(adapter)
        elif key == " ":
            _look_and_announce(adapter)
        elif key == "f":
            print(f"  📣  {_call(adapter, 'speak', {'text': _FOUND_SPEECH})}")
        elif key == "+":
            step += 0.2
            print(f"  step = {step:.1f}m")
        elif key == "-":
            step = max(0.2, step - 0.2)
            print(f"  step = {step:.1f}m")


if __name__ == "__main__":
    main()
