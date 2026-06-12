from __future__ import annotations

import json
import os
import socket
from pathlib import Path

import questionary

from dimwizard.install import install

_STATE_HOME = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state"))
_CONFIG_PATH = _STATE_HOME / "dimwizard" / "config.json"


def is_configured() -> bool:
    return _CONFIG_PATH.exists()


def load_config() -> dict[str, str]:
    return json.loads(_CONFIG_PATH.read_text())


def save_config(robot_name: str) -> None:
    _CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    _CONFIG_PATH.write_text(json.dumps({"robot_name": robot_name}, indent=2))


def clear_config() -> None:
    if _CONFIG_PATH.exists():
        _CONFIG_PATH.unlink()


def setup_wizard() -> None:
    """Hook for dimos run — runs setup on first invocation, skips on subsequent runs."""
    if is_configured():
        return

    robot_name = os.environ.get("DIMENSIONAL_ROBOT_NAME", socket.gethostname().split(".")[0])
    print()
    confirmed = questionary.confirm(
        "First time running DimOS - set up robot network discovery? (recommended)",
        default=True,
    ).ask()

    if confirmed is None:
        raise KeyboardInterrupt
    if not confirmed:
        return

    if not install():
        print("  ✗ Service installation failed — re-run `dimos run` to retry.\n")
        return
    save_config(robot_name)
    print(f"  ✓ {robot_name} is now discoverable on the network\n")
