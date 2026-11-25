# Copyright 2025-2026 Dimensional Inc.
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

"""Resolved deployment declarations without subscribing to application traffic."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
from typing import Any

from dimos.diagnostics.runtime import get_runtime


def fingerprint(value: Any) -> str:
    # Only a digest leaves the process; config values and credentials never do.
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, default=lambda v: type(v).__qualname__).encode()
    ).hexdigest()


def record_deployment(blueprint: Any, transports: Any, config: Any) -> None:
    runtime = get_runtime()
    if runtime is None:
        return
    source = Path(__file__).resolve().parents[2]
    revision = "unknown"
    dirty_hash = "unknown"
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=source,
            capture_output=True,
            text=True,
            timeout=2,
            check=True,
        ).stdout.strip()
        diff = subprocess.run(
            ["git", "diff", "HEAD", "--", "."],
            cwd=source,
            capture_output=True,
            timeout=2,
            check=True,
        ).stdout
        dirty_hash = hashlib.sha256(diff).hexdigest()
    except (OSError, subprocess.SubprocessError):
        pass
    generation = fingerprint(
        [(key[0], key[1].__qualname__, type(value).__name__) for key, value in transports.items()]
    )
    runtime.event(
        "deployment.build",
        generation=generation,
        revision=revision,
        tracked_diff_sha256=dirty_hash,
        config_sha256=fingerprint(config),
    )
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            name = blueprint.remapping_map.get((atom.name, stream.name), stream.name)
            transport = transports.get((name, stream.type))
            runtime.event(
                "topology.declared",
                generation=generation,
                module=atom.name,
                endpoint=stream.name,
                direction=stream.direction,
                channel=str(getattr(transport, "channel", name)),
                transport=type(transport).__name__,
                type_name=f"{stream.type.__module__}.{stream.type.__qualname__}",
            )
