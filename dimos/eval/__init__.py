# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Hardware benchmark eval utility for DimOS.
python -m dimos.eval <blueprint> [--duration 15] [--simulation]
"""

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.eval.hardware import HardwareProfile, detect_hardware
    from dimos.eval.metrics import EvalResult, WorkerFootprint
    from dimos.eval.runner import run_eval

__all__ = [
    "EvalResult",
    "HardwareProfile",
    "WorkerFootprint",
    "detect_hardware",
    "run_eval",
]

_LAZY = {
    "HardwareProfile": "dimos.eval.hardware",
    "detect_hardware": "dimos.eval.hardware",
    "EvalResult": "dimos.eval.metrics",
    "WorkerFootprint": "dimos.eval.metrics",
    "run_eval": "dimos.eval.runner",
}


def __getattr__(name: str) -> object:
    module_path = _LAZY.get(name)
    if module_path is None:
        raise AttributeError(f"module 'dimos.eval' has no attribute {name!r}")
    import importlib

    return getattr(importlib.import_module(module_path), name)
