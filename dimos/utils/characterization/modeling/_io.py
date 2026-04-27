# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Atomic JSON write helper, mirroring ``processing/noise.py``."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any


def atomic_write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, default=str) + "\n")
    os.replace(tmp, path)


__all__ = ["atomic_write_json"]
