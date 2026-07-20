from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml


def load_config_mapping(path: Path, *, missing_ok: bool = False) -> dict[str, Any]:
    """Load a YAML or JSON configuration file with a mapping at its root."""
    try:
        data = yaml.safe_load(path.read_text())
    except FileNotFoundError:
        if missing_ok:
            return {}
        raise ValueError(f"Could not read config file {path}: file does not exist") from None
    except OSError as exc:
        raise ValueError(f"Could not read config file {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"Could not parse YAML or JSON config file {path}: {exc}") from exc

    if data is None:
        return {}
    if not isinstance(data, Mapping):
        raise ValueError(
            f"Config file {path} must contain a mapping at the top level, got {type(data).__name__}"
        )
    return dict(data)
