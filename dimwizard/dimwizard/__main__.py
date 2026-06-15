from __future__ import annotations

import signal
import sys
import threading

from dimwizard.advertise import Advertiser
from dimwizard.install import is_installed


def _run_beacon() -> None:
    advertiser = Advertiser()

    stop_event = threading.Event()

    def _handle_signal(sig: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    try:
        advertiser.start()
        stop_event.wait()
    finally:
        advertiser.stop()


def main() -> None:
    if not is_installed():
        print("dimwizard: not installed, exiting.", file=sys.stderr)
        sys.exit(0)
    try:
        _run_beacon()
    except Exception as e:
        print(f"dimwizard: beacon error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
