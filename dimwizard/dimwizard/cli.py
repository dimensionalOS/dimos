from __future__ import annotations

import typer

app = typer.Typer(help="dimwizard — robot network beacon")


@app.command()
def kill() -> None:
    """Remove the dimwizard beacon service and reset configuration."""
    from dimwizard.install import uninstall
    from dimwizard.setup import clear_config
    uninstall()
    clear_config()


def main() -> None:
    app()


if __name__ == "__main__":
    main()
