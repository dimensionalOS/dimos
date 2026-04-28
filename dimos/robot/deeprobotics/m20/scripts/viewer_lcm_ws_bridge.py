#!/usr/bin/env python3
"""Forward local dimos-viewer LCM events to the NOS WebSocket bridge.

The current native dimos-viewer emits click-to-goal selections and WASD
teleop twists on local LCM. On remote M20 deployments that LCM multicast
does not cross the SSH tunnel, so this helper translates those local messages
into the JSON protocol accepted by RerunWebSocketServer on NOS.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import signal
import time
from typing import Any

import lcm
import websockets.asyncio.client as ws_client

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.PointStamped import PointStamped

DEFAULT_CLICK_CHANNEL = "/clicked_point#geometry_msgs.PointStamped"
DEFAULT_TWIST_CHANNEL = "/cmd_vel#geometry_msgs.Twist"
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
DEFAULT_WS_URL = "ws://127.0.0.1:3030/ws"


def point_to_click_message(point: PointStamped) -> dict[str, Any]:
    """Convert a local LCM PointStamped into the viewer WebSocket click shape."""
    return {
        "type": "click",
        "x": point.x,
        "y": point.y,
        "z": point.z,
        "entity_path": point.frame_id,
        "timestamp_ms": int(point.ts * 1000),
    }


def twist_to_ws_message(twist: Twist) -> dict[str, Any]:
    """Convert a local LCM Twist into the viewer WebSocket twist shape."""
    return {
        "type": "twist",
        "linear_x": twist.linear.x,
        "linear_y": twist.linear.y,
        "linear_z": twist.linear.z,
        "angular_x": twist.angular.x,
        "angular_y": twist.angular.y,
        "angular_z": twist.angular.z,
        "timestamp_ms": int(time.time() * 1000),
    }


async def send_message(ws_url: str, message: dict[str, Any]) -> None:
    """Send one viewer message, opening a short-lived WebSocket connection."""
    async with ws_client.connect(ws_url) as websocket:
        await websocket.send(json.dumps(message))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--click-channel", default=DEFAULT_CLICK_CHANNEL)
    parser.add_argument("--twist-channel", default=DEFAULT_TWIST_CHANNEL)
    parser.add_argument("--ws-url", default=DEFAULT_WS_URL)
    parser.add_argument(
        "--lcm-url",
        default=os.environ.get("LCM_DEFAULT_URL", DEFAULT_LCM_URL),
        help="LCM URL to subscribe to for local dimos-viewer events.",
    )
    args = parser.parse_args()

    should_stop = False

    def _request_stop(_signum: int, _frame: Any) -> None:
        nonlocal should_stop
        should_stop = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    lc = lcm.LCM(args.lcm_url)

    def _on_click(_channel: str, data: bytes) -> None:
        try:
            point = PointStamped.lcm_decode(data)
            message = point_to_click_message(point)
            asyncio.run(send_message(args.ws_url, message))
        except Exception as exc:
            print(f"[viewer_lcm_ws_bridge] failed to forward click: {exc}", flush=True)
            return

        print(
            "[viewer_lcm_ws_bridge] forwarded click "
            f"x={point.x:+.3f} y={point.y:+.3f} z={point.z:+.3f} "
            f"frame={point.frame_id!r}",
            flush=True,
        )

    def _on_twist(_channel: str, data: bytes) -> None:
        try:
            twist = Twist.lcm_decode(data)
            message = twist_to_ws_message(twist)
            asyncio.run(send_message(args.ws_url, message))
        except Exception as exc:
            print(f"[viewer_lcm_ws_bridge] failed to forward twist: {exc}", flush=True)
            return

        print(
            "[viewer_lcm_ws_bridge] forwarded twist "
            f"lin=({twist.linear.x:+.3f},{twist.linear.y:+.3f},{twist.linear.z:+.3f}) "
            f"ang=({twist.angular.x:+.3f},{twist.angular.y:+.3f},{twist.angular.z:+.3f})",
            flush=True,
        )

    click_subscription = lc.subscribe(args.click_channel, _on_click)
    twist_subscription = lc.subscribe(args.twist_channel, _on_twist)
    print(
        "[viewer_lcm_ws_bridge] listening "
        f"{args.click_channel} and {args.twist_channel} on {args.lcm_url}; "
        f"forwarding to {args.ws_url}",
        flush=True,
    )

    try:
        while not should_stop:
            lc.handle_timeout(100)
            time.sleep(0.001)
    finally:
        lc.unsubscribe(click_subscription)
        lc.unsubscribe(twist_subscription)
        print("[viewer_lcm_ws_bridge] stopped", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
