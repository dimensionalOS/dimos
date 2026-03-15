from typing import Any

from dimos.models.vl.types import VlModelName
from dimos.models.vl.base import VlModel


def create(name: VlModelName) -> VlModel[Any]:
    # This uses inline imports to only import what's needed.
    match name:
        case "qwen":
            from dimos.models.vl.qwen import QwenVlModel
            return QwenVlModel()
        case "moondream":
            from dimos.models.vl.moondream import MoondreamVlModel
            return MoondreamVlModel()
