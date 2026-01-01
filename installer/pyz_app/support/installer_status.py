#!/usr/bin/env python3
# Central location for installer execution flags.

from __future__ import annotations

from typing import Dict, Any

# Defaults mirror previous behavior; can be updated at runtime.
installer_status: Dict[str, Any] = {
    "dry_run": True,          # was previously in misc.py
    "dev": True,              # was previously in misc.py
    "non_interactive": False, # set by __main__ when detected/passed
}