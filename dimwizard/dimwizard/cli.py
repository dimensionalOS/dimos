from __future__ import annotations

import typer

from dimwizard.install import uninstall

app = typer.Typer(help="dimwizard — robot network beacon")


@app.command()
def kill() -> None:
    """Remove the dimwizard beacon service."""
    uninstall()


def main() -> None:
    app()


if __name__ == "__main__":
    main()
