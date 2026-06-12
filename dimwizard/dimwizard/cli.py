from __future__ import annotations

import typer

from dimwizard.setup import is_configured

app = typer.Typer(help="dimwizard — robot network beacon")


@app.command()
def install() -> None:
    """Install and start the dimwizard beacon service."""
    if not is_configured():
        typer.echo("Not configured — run dimos first to complete setup.")
        raise typer.Exit(1)
    from dimwizard.install import install as _install
    _install()


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
