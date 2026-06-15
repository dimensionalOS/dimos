from __future__ import annotations

import os
import socket

import typer

from dimwizard.advertise import SERVICE_TYPE, local_ip
from dimwizard.install import is_installed, is_running, uninstall

app = typer.Typer(help="dimwizard - robot network beacon")


@app.command()
def status() -> None:
    """Show beacon status."""
    installed = is_installed()
    running = is_running()

    print(f"installed: {'yes' if installed else 'no'}")
    print(f"running:   {'yes' if running else 'no'}")

    if installed:
        robot_name = os.environ.get("DIMENSIONAL_ROBOT_NAME", socket.gethostname().split(".")[0])
        lcm_url = os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=1")
        print(f"robot:     {robot_name}")
        print(f"mdns:      {robot_name}.{SERVICE_TYPE}")
        print(f"lcm url:   {lcm_url}")
        try:
            print(f"ip:        {local_ip()}")
        except OSError as e:
            print(f"ip:        unavailable ({e})")


@app.command()
def kill() -> None:
    """Remove the dimwizard beacon service."""
    uninstall()


def main() -> None:
    app()
