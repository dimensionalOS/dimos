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

from __future__ import annotations

from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import config, logs, models
from experimental.gateway.utils.http import STARTED_AT, STARTED_FROM


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/paths",
        response_model=models.Paths,
        **route_doc(
            "server",
            "Where dimos keeps its things: the checkout, run registry, log folders, recordings folder",
            "Absolute paths, read from the environment and Desktop's config.yaml. Desktop compares `dimosDir` with its configured checkout to tell whether this gateway is the right one, and `server` with its own binary to tell whether its built-in gateway is outdated (this gateway runs python, so it never is). No side effects.",
            answer="`{ dimosDir, runsDir, logsDirs, recordingsDir, server: { exe, exeModified, kind, startedAt, zenohNamespace } }`",
        ),
    )
    async def paths() -> dict[str, Any]:
        from dimos.core.run_registry import REGISTRY_DIR

        return {
            "dimosDir": str(s.dimos_dir),
            "runsDir": str(REGISTRY_DIR),
            "logsDirs": [str(d) for d in logs.logs_dirs(s.dimos_dir)],
            "recordingsDir": str(config.recordings_dir()),
            "server": {
                "exe": STARTED_FROM[0],
                "exeModified": STARTED_FROM[1],
                "kind": "dimos",
                "startedAt": STARTED_AT,
                "zenohNamespace": s.zenoh_namespace,
            },
        }
