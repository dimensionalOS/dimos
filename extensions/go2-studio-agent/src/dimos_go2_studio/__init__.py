"""User-editable Go2 skills and Blueprint for DimOS Studio.

The robot blueprint is loaded lazily so importing pure mission policy helpers
does not eagerly import Open3D, perception models, or hardware modules.
"""

from typing import Any

__all__ = ["go2_studio_agentic"]


def __getattr__(name: str) -> Any:
    if name == "go2_studio_agentic":
        from .blueprint import go2_studio_agentic

        return go2_studio_agentic
    raise AttributeError(name)
