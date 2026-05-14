"""Manipulation × memory2 integration.

Three agent-callable perception skills built on memory2 primitives:
``find_objects``, ``find_objects_near``, ``recall``. See ``spec.py``
for the architecture docstring, configs, and Protocol definitions.
"""

from dimos.manipulation.memory2.lazy_perception import LazyPerceptionModule
from dimos.manipulation.memory2.recorder import RGBDCameraRecorder
from dimos.manipulation.memory2.spec import (
    LazyPerceptionModuleConfig,
    LazyPerceptionModuleSpec,
    RGBDCameraRecorderConfig,
    RGBDCameraRecorderSpec,
)

__all__ = [
    "LazyPerceptionModule",
    "LazyPerceptionModuleConfig",
    "LazyPerceptionModuleSpec",
    "RGBDCameraRecorder",
    "RGBDCameraRecorderConfig",
    "RGBDCameraRecorderSpec",
]
