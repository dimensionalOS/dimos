#!/usr/bin/env python3
# Central location for installer execution flags.

from __future__ import annotations

from typing import Dict, Any

# Defaults mirror previous behavior; can be updated at runtime.
installer_status: Dict[str, Any] = {
    "dry_run": False, # hard-coded for now, probably should be a CLI arg. NOTE: dry run is not completely dry
    "dev": True, # hard-coded for now, probably should be a CLI arg
    "non_interactive": False, # set by __main__ when detected/passed
}