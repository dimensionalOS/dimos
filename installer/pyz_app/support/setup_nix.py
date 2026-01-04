#!/usr/bin/env python3
# Helper for generating a sample Nix flake for Dimos.
from __future__ import annotations

from pathlib import Path
from typing import Iterable

from . import prompt_tools as p
from .bundled_data import FLAKE_TEMPLATE

def setup_nix_flake(project_dir: str | Path) -> Path:
    """Write flake.example.nix with the installer flake contents."""
    project_dir = Path(project_dir)
    example_path = project_dir / "flake.example.nix"
    if example_path.exists():
        if not p.ask_yes_no(f"{example_path.name} exists. Overwrite?"):
            return example_path
    example_path.write_text(FLAKE_TEMPLATE)
    return example_path


__all__ = ["setup_nix_flake", "FLAKE_TEMPLATE"]
