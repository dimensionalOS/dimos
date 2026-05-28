"""Send DimOS Go2 frontier-exploration LCM commands.

Run this while a `unitree-go2` or `unitree-go2-memory` stack is already
running. The stack's `WavefrontFrontierExplorer` subscribes to these Boolean
command topics:

- `/explore_cmd`: start autonomous frontier exploration.
- `/stop_explore_cmd`: stop exploration and publish the current pose as a goal.

Examples:
    python scripts/go2_exploration.py start
    python scripts/go2_exploration.py stop
"""

from __future__ import annotations

import argparse
import time
from typing import Literal

from dimos.core.transport import LCMTransport
from dimos_lcm.std_msgs import Bool


Action = Literal["start", "stop"]


_TOPICS: dict[Action, str] = {
    "start": "/explore_cmd",
    "stop": "/stop_explore_cmd",
}


def send_exploration_command(
    action: Action,
    repeats: int,
    interval_s: float,
) -> None:
    """Publish a frontier-exploration command over LCM.

    Args:
        action: `start` begins exploration; `stop` ends it.
        repeats: Number of times to publish the command.
        interval_s: Delay between repeated publishes.

    Raises:
        ValueError: If repeat or interval settings are invalid.
    """
    if repeats < 1:
        raise ValueError("repeats must be at least 1")
    if interval_s < 0.0:
        raise ValueError("interval_s must be non-negative")

    topic = _TOPICS[action]
    transport = LCMTransport(topic, Bool)
    message = Bool(data=True)
    for index in range(repeats):
        transport.broadcast(None, message)
        if index + 1 < repeats and interval_s > 0.0:
            time.sleep(interval_s)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start or stop DimOS Go2 autonomous frontier exploration.",
    )
    parser.add_argument(
        "action",
        choices=sorted(_TOPICS),
        help="Exploration command to send.",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=3,
        help="Publish count. Repeating makes startup timing less brittle.",
    )
    parser.add_argument(
        "--interval-s",
        type=float,
        default=0.2,
        help="Seconds between repeated publishes.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    send_exploration_command(args.action, args.repeats, args.interval_s)
    verb = "Started" if args.action == "start" else "Stopped"
    print(f"{verb} autonomous frontier exploration.")


if __name__ == "__main__":
    main()
