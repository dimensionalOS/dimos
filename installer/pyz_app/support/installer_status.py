#!/usr/bin/env python3
# Central location for installer execution flags.

from __future__ import annotations

from typing import Dict, Any

# Defaults mirror previous behavior; can be updated at runtime.
installer_status: Dict[str, Any] = {
    "dry_run": False,  # can be set via CLI
    "dev": True,  # can be set via CLI in the future
    "non_interactive": False,  # set by __main__ when detected/passed
    "template_repo": False,  # set when invoked from a dimos template repo
}
