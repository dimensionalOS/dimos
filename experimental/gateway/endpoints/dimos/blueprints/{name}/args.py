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
from experimental.gateway.utils import models
from experimental.gateway.utils.api_helpers import ApiHelpers
from experimental.gateway.utils.http import BlueprintParam


def register(app: FastAPI, state: ServerState) -> None:
    helpers = ApiHelpers(state)

    @app.post(
        "/dimos/blueprints/{name}/args",
        response_model=models.ArgsCheck,
        **route_doc(
            "blueprints",
            "Check `dimos run <name>` arguments without running anything: what each one sets, or why dimos won't take it",
            "Reads `args` (argv items, never a shell string) with dimos's own blueprint config parser, in a child process that imports the blueprint (cached 10 min per blueprint and args): each option on its own, with the items it took, what it sets (`g.<key>`, `<module>.<field>`, `run --<option>`) and dimos's error for it. `--daemon` and `--help` are refused (a launch runs in the foreground), and so is anything but options (one blueprint per launch). POST /dimos/runs checks its `args` the same way. No side effects. New in API 1.17.",
            errors=(400, 500),
            agent=True,
            answer="`{ name, args: [{ tokens, target, error }] }`",
        ),
    )
    async def check_blueprint_args(name: BlueprintParam, request: models.ArgsCheckRequest) -> Any:
        helpers.check_name(name)
        return await helpers.checked_args(name, request.args)
