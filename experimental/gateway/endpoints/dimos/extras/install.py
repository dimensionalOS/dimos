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

import asyncio
from collections.abc import Awaitable, Callable
from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import config, desktop, extras, models
from experimental.gateway.utils.discovery import python_for
from experimental.gateway.utils.discovery_helpers import DiscoveryHelpers
from experimental.gateway.utils.http import ApiError
from experimental.gateway.utils.jobs import MissingForJobError


def register(app: FastAPI, state: ServerState) -> None:
    installing: dict[str, str | None] = {"shell": None}
    s = state
    discovery = state.discovery
    assert discovery is not None
    jobs = state.jobs
    assert jobs is not None
    helpers = DiscoveryHelpers(state)

    @app.post(
        "/dimos/extras/install",
        response_model=models.ExtrasInstallStarted,
        **route_doc(
            "extras",
            "Install dimos extras through Desktop's shell tool (the user sees the commands and presses Run); discovery rescans when it ends",
            "Runs scripts/install.sh's command: `uv sync --locked --inexact --extra <x> ...` in a checkout (keeps every extra and group already installed), else `uv pip install --python <venv python> --torch-backend cpu|cu128 'dimos[<x>,...]==<version>'`; through Desktop, one such command per extra, run one at a time. Never sudo. When that builds the cyclonedds package from source (unitree-dds, dds: it has wheels for python 3.10 only), it first gets the CycloneDDS C library it builds against: $CYCLONEDDS_HOME, else `nix build <flake.lock's nixpkgs>#cyclonedds` (an out-link in the venv), else Homebrew's; none = 400 with `code: cyclonedds_missing`. With Desktop (`$DESKTOP_URL`) the commands go to its `POST /api/desktop/shell` for the asking app (default `launcher`): it shows them over that app, nothing runs until the user presses Run, a failure can be fixed in its terminal (by the user or Desktop's agent) and retried; the answer is `{ shell }`, the session to follow with Desktop's `GET /api/desktop/shell/{id}?wait=`. The gateway waits for it and rescans. Without Desktop it runs as a job (`{ job }`: follow `<ns>/dimos/jobs/<job>` or GET /dimos/jobs/{job}/log). 400 for an unknown extra; 409 while another install runs; 500 when uv isn't found.",
            errors=(400, 409, 500),
            agent=True,
            answer="`{ shell, job, command }` (one of shell / job)",
        ),
    )
    async def install_extras(request: models.ExtrasInstall) -> dict[str, Any]:
        declared = extras.declared(s.dimos_dir, await helpers.probe())
        unknown = [name for name in request.extras if name not in declared]
        if unknown:
            raise ApiError(
                400, f"unknown extra(s): {', '.join(unknown)} (known: {', '.join(declared)})"
            )
        running = jobs.running("extras")
        if running is not None or installing["shell"] is not None:
            which = (
                f"job {running.id}"
                if running is not None
                else f"Desktop shell session {installing['shell']}"
            )
            raise ApiError(409, f"an extras install is already running: {which}")
        uv = extras.find_uv()
        if uv is None:
            raise ApiError(500, "uv isn't installed (https://docs.astral.sh/uv/)")
        wanted = list(dict.fromkeys(request.extras))
        probed = await helpers.probe()
        cyclonedds = extras.builds_cyclonedds(
            s.dimos_dir,
            extras.status(s.dimos_dir, probed, lock_sizes=False),
            wanted,
            probed.get("environment", {}),
        )
        python = python_for(s.dimos_dir)
        version = config.info(s.dimos_dir).version
        command = extras.install_command(s.dimos_dir, wanted, python, version, uv)

        def then(_: Any) -> None:
            s.cache.forget("packages")
            discovery.refresh("extras installed")

        try:
            steps = extras.shell_commands(s.dimos_dir, wanted, python, version, uv, cyclonedds)
        except MissingForJobError as error:
            raise ApiError(400, f"{error.code}: {error}")
        title = f"Install the dimOS extras {', '.join(wanted)}"
        try:
            session = await desktop.request_shell(
                title,
                "dimOS installs these optional packages into its Python environment with uv (no sudo). It can take a few minutes.",
                steps,
                request.app or "launcher",
            )
        except desktop.DesktopUnavailableError:
            session = None
        if session is not None:
            installing["shell"] = session

            async def follow() -> None:
                try:
                    await desktop.wait_shell(session)
                finally:
                    installing["shell"] = None
                    then(None)

            s.background.append(asyncio.create_task(follow()))
            return {"shell": session, "job": None, "command": command}
        prepare = None
        if cyclonedds:

            async def prepare(
                _: Any, step: Callable[[list[str]], Awaitable[int]]
            ) -> dict[str, str]:
                return await extras.prepare_cyclonedds(s.dimos_dir, step)

        env = (
            {"VIRTUAL_ENV": str(config.venv_dir(s.dimos_dir))}
            if extras.is_checkout(s.dimos_dir)
            else {}
        )
        job = jobs.start(
            f"Install extras: {', '.join(wanted)}",
            "extras",
            command,
            s.dimos_dir,
            env,
            then,
            prepare,
        )
        return {"shell": None, "job": job.id, "command": command}
