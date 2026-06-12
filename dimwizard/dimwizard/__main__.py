from __future__ import annotations

import os
import signal
import threading

from dimwizard.advertise import Advertiser
from dimwizard.setup import load_config


def _lcm_url() -> str:
    return os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=1")


def _run_beacon(robot_name: str) -> None:
    advertiser = Advertiser(
        robot_name=robot_name,
        lcm_url=_lcm_url(),
        port=7667,
    )
    advertiser.start()

    stop_event = threading.Event()

    def _handle_signal(sig: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    stop_event.wait()
    advertiser.stop()


def main() -> None:
    config = load_config()
    _run_beacon(config["robot_name"])


main()
