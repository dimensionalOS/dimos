#!/usr/bin/env python3
"""Pause/resume the live OpenPI policy while keeping dimOS hardware active."""

from __future__ import annotations

import argparse
import time

from dimos.porcelain.dimos import Dimos


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("pause", "resume", "status"))
    args = parser.parse_args()

    app = Dimos.connect()
    try:
        client = app.OpenPIClient
        coordinator = app.ControlCoordinator
        if args.command == "pause":
            if not client.pause():
                raise RuntimeError("OpenPI client is not running")
            deadline = time.monotonic() + 40.0
            while not client.is_paused():
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "Policy did not finish its current chunk; hold was not engaged"
                    )
                time.sleep(0.1)
            hold = coordinator.task_invoke(
                "operator_hold", "request", {"route": "manual", "reason": "OpenPI policy paused"}
            )
            if hold is None or not hold.on:
                raise RuntimeError("Policy paused, but operator hold was not engaged")
            print("Paused after current chunk; operator hold requested, motors remain enabled.")
        elif args.command == "resume":
            hold = coordinator.task_invoke("operator_hold", "get_status")
            paused = client.is_paused()
            holding = bool(hold and hold.on)
            if not paused and not holding:
                print("Policy inference is already running.")
                return
            if not paused or not holding:
                raise RuntimeError("Policy is not paused with an active operator hold")
            coordinator.task_invoke("operator_hold", "acknowledge")
            if not client.resume():
                raise RuntimeError("Hold released, but OpenPI client could not resume")
            print("Operator hold released; policy inference resumed.")
        else:
            hold = coordinator.task_invoke("operator_hold", "get_status")
            print(f"policy_paused={client.is_paused()} operator_hold={bool(hold and hold.on)}")
    finally:
        app.stop()


if __name__ == "__main__":
    main()
