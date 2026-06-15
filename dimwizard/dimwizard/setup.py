from __future__ import annotations

import os
import socket

import questionary

from dimwizard.install import install, is_installed


def setup_wizard() -> None:
    """Hook for dimos run - runs setup on invocation, skips if already installed."""
    if is_installed():
        return

    robot_name = os.environ.get("DIMENSIONAL_ROBOT_NAME", socket.gethostname().split(".")[0])
    print()
    confirmed = questionary.confirm(
        "Set up robot network discovery? (recommended)",
        default=True,
    ).ask()

    if confirmed is None:
        return
    if not confirmed:
        return

    if not install():
        print("  ✗ Service installation failed — re-run `dimos run` to retry.\n")
        return
    print(f"  ✓ {robot_name} is now discoverable on the network\n")
