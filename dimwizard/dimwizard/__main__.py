from __future__ import annotations

import os
import signal
import socket
import sys
import threading

from dimwizard.advertise import Advertiser
from dimwizard.install import is_installed


def _lcm_url() -> str:
    return os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=1")


def _run_beacon(robot_name: str) -> None:
    advertiser = Advertiser(
        robot_name=robot_name,
        lcm_url=_lcm_url(),
    )
    stop_event = threading.Event()

    def _handle_signal(sig: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    advertiser.start()
    stop_event.wait()
    advertiser.stop()


def main() -> None:
    if not is_installed():
        print("dimwizard: not installed, exiting.", file=sys.stderr)
        sys.exit(0)
    robot_name = os.environ.get("DIMENSIONAL_ROBOT_NAME", socket.gethostname().split(".")[0])
    try:
        _run_beacon(robot_name)
    except Exception as e:
        print(f"dimwizard: beacon error: {e}", file=sys.stderr)
        sys.exit(0)


if __name__ == "__main__":
    main()
