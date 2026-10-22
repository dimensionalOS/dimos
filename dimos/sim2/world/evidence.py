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

"""Small run records, not another model cache or a promise of deterministic replay."""

from dataclasses import fields, is_dataclass
from functools import cache
import hashlib
from importlib.metadata import version
import inspect
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from dimos.sim2.world.runtime import STATE, SimulationRuntime


def file_identity(path: Path) -> dict[str, Any]:
    with path.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    return {"path": str(path), "sha256": digest, "bytes": path.stat().st_size}


def run_record(runtime: SimulationRuntime, compiled_model: Path) -> dict[str, Any]:
    @cache
    def identify(path: Path) -> dict[str, Any]:
        return file_identity(path) if path.is_file() else {"path": str(path)}

    def resolved(value: Any) -> Any:
        if isinstance(value, Path):
            return identify(value)
        if is_dataclass(value) and not isinstance(value, type):
            return {f.name: resolved(getattr(value, f.name)) for f in fields(value)}
        if isinstance(value, dict):
            return {str(k): resolved(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [resolved(v) for v in value]
        if callable(value):
            path = inspect.getsourcefile(value)
            return {
                "callable": f"{value.__module__}.{value.__qualname__}",
                "source": identify(Path(path)) if path else None,
            }
        return value

    state = np.empty(mujoco.mj_stateSize(runtime.model, STATE))
    with runtime.lock:
        mujoco.mj_getState(runtime.model, runtime.data, state, STATE)
        return {
            "world_id": runtime.world_id,
            "mujoco_version": version("mujoco"),
            "runtime_source": identify(Path(inspect.getfile(SimulationRuntime))),
            "compiled_model": identify(compiled_model),
            "resolved_world": resolved(runtime.config),
            "initial_state": {"spec": int(STATE), "values": state.tolist()},
            "clock": "simulation time; epoch reanchored to host time on reset/resume",
            "reproducibility": "initial physics state and settings; asynchronous commands and scheduling are not replayed",
            "calibration_status": "declared evidence only; unlisted parameters are unvalidated",
        }


def write_record(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)
