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

"""The discovery routes: the cache of blueprints, modules and message types (discovery.py), modules ranked per robot,
a module's default config, dimos's docs (docs.py), its extras (extras.py) and the jobs that install them (jobs.py)."""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from typing import TYPE_CHECKING, Annotated, Any

from fastapi import FastAPI, Path as PathParam, Query
from packaging.markers import default_environment

from dimos.gateway import config, desktop, docs, extras, models
from dimos.gateway.discovery import python_for
from dimos.gateway.jobs import MissingForJobError
from dimos.gateway.openapi import route_doc

if TYPE_CHECKING:
    from dimos.gateway.app import ServerState

EXTRAS_TTL_S = 10.0

ModuleParam = Annotated[
    str,
    PathParam(
        description="a module's registry name, class name or `module.Class`",
        examples=["go2-connection"],
    ),
]
JobParam = Annotated[str, PathParam(description="a job id", examples=["extras-1-1791000000"])]


def add(app: FastAPI, state: ServerState) -> None:
    from dimos.gateway.app import ApiError

    s = state
    assert s.discovery is not None and s.jobs is not None
    discovery = s.discovery
    jobs = s.jobs

    async def probe() -> dict[str, Any]:
        async def compute() -> dict[str, Any]:
            answer = await discovery.child_answer("packages")
            if "error" in answer:
                raise ApiError(500, answer["error"])
            return answer

        result: dict[str, Any] = await s.cache.get("packages", EXTRAS_TTL_S, compute)
        return result

    @app.get(
        "/dimos/discovery",
        response_model=models.DiscoveryStatus,
        **route_doc(
            "discovery",
            "Where the discovery scan (every blueprint and module imported once, cached) is: progress, errors",
            "The scan starts when the gateway starts. A restart with the same checkout commit, dirty files and "
            "installed packages answers from the disk cache at once (no scan); a change is noticed within 30 s and "
            "rescanned (the old answer is served meanwhile, `stale: true`). `discovery` events follow it. No side "
            "effects.",
            agent=True,
            answer="`DiscoveryStatus`: `{ state, reason, key, stale, blueprints_total, blueprints_done, importable, "
            "not_importable, modules_total, modules_done, current, errors, ... }`",
        ),
    )
    async def discovery_status() -> dict[str, Any]:
        discovery.update_counts()
        return dict(discovery.status)

    @app.post(
        "/dimos/discovery/refresh",
        response_model=models.DiscoveryStatus,
        **route_doc(
            "discovery",
            "Check the checkout and packages again now, and rescan what changed (or everything, `full`)",
            "Wakes the discovery loop: it recomputes the cache key and scans what's missing; with `full` it forgets "
            "the answer and imports everything again. Answers the status at once; `discovery` events follow the "
            "scan. The body is optional.",
            answer="`DiscoveryStatus`",
        ),
    )
    async def discovery_refresh(request: models.DiscoveryRefresh | None = None) -> dict[str, Any]:
        discovery.refresh("requested", bool(request and request.full))
        return dict(discovery.status)

    @app.get(
        "/dimos/discovery/blueprints",
        response_model=models.DiscoveredBlueprints,
        **route_doc(
            "discovery",
            "Every blueprint from the discovery cache: whether it imports (and why not), its robot, its modules and "
            "their streams with topics",
            "Answers from the cache (no import): every blueprint scanned so far, in registry order, with "
            "`suggested_extras` for one that's missing a package. No side effects.",
            agent=True,
            answer="`{ stale, blueprints: [{ name, ref, robot, importable, import_error, missing_module, "
            "suggested_extras, modules: [{ name, class, module, streams: [{ name, type, direction, topic }] }] }] }`",
        ),
    )
    async def discovered_blueprints() -> dict[str, Any]:
        providers = extras.providing_extras(
            s.dimos_dir, {key: str(value) for key, value in default_environment().items()}
        )
        records = [
            {**record, "suggested_extras": providers(record.get("missing_module"))}
            for record in discovery.blueprint_list()
        ]
        return {"stale": bool(discovery.status["stale"]), "blueprints": records}

    @app.get(
        "/dimos/modules",
        response_model=models.ModuleList,
        **route_doc(
            "discovery",
            "Every module from the discovery cache: streams in and out, skills, and how many blueprints use it",
            "Answers from the cache (no import): the registry's modules and every module a blueprint uses, with "
            "`blueprint_count` (importable blueprints using it) and `robots`. No side effects.",
            agent=True,
            answer="`{ stale, modules: [{ name, class, doc, inputs, outputs, skills, blueprint_count, robots }] }`",
        ),
    )
    async def module_list() -> dict[str, Any]:
        return {"stale": bool(discovery.status["stale"]), "modules": discovery.module_list()}

    @app.get(
        "/dimos/modules/{module}/config",
        response_model=models.ModuleConfigAnswer,
        **route_doc(
            "discovery",
            "A module's config fields and defaults: name, type, default, description, enum values, and whether it "
            "converts to and from JSON",
            "Read from the module's real config class (pydantic model or dataclass) by the discovery scan; a "
            "registry module the scan hasn't reached yet is imported now, in a child process. A field that isn't "
            "`json_compatible` (a class, a callable, an array) can't be set from a form: leave it out. 404 for a "
            "module that isn't in the registry or any blueprint; 500 when it can't be imported.",
            errors=(404, 500),
            agent=True,
            answer="`{ module, class, fields: [{ name, type, default, description, required, base, enum, "
            "json_compatible, reason }], error }`",
        ),
    )
    async def module_config(module: ModuleParam) -> dict[str, Any]:
        record = discovery.find_module(module)
        if record is None:
            record = await discovery.module_now(module)
        if record is None:
            raise ApiError(404, f"no module {module} (not in dimos's registry or any blueprint)")
        if "error" in record and "class" not in record:
            raise ApiError(500, f"module {module} doesn't import: {record['error']}")
        return {
            "module": record["name"],
            "class": record["class"],
            "fields": record.get("config", []),
            "error": record.get("config_error"),
        }

    @app.get(
        "/dimos/message-types",
        response_model=models.MessageTypes,
        **route_doc(
            "discovery",
            "Every message type on a module's stream, with the modules that publish and read it",
            "Answers from the discovery cache. No side effects.",
            agent=True,
            answer="`{ types: [{ type, publishers, subscribers }] }`",
        ),
    )
    async def message_types() -> dict[str, Any]:
        return {"types": discovery.message_types()}

    @app.get(
        "/dimos/robots/{robot}/modules",
        response_model=models.RobotModules,
        **route_doc(
            "discovery",
            "The modules a robot's blueprints use, most specific to that robot first (TF-IDF style)",
            "score = (fraction of the robot's importable blueprints that use the module) x ln(robots / robots "
            "using the module): its own connection module comes first, a module every robot uses scores 0, and "
            "a widely shared one (RerunBridgeModule) well below the robot's own. A robot is a robots.json id (GET "
            "/dimos/robots); a blueprint belongs to the robot that lists it or whose dirs hold its file. "
            "From the discovery cache; 404 for a robot with no blueprint.",
            errors=(404,),
            agent=True,
            answer="`{ robot, blueprints, blueprints_importable, robots_total, formula, modules: [{ name, class, "
            "score, in_robot_blueprints, robot_blueprints, robots_using, blueprint_count }] }`",
        ),
    )
    async def robot_modules(
        robot: Annotated[
            str, PathParam(description="a robot id from GET /dimos/robots", examples=["go2"])
        ],
    ) -> dict[str, Any]:
        answer = discovery.robot_modules(robot)
        if answer is None:
            known = ", ".join(sorted(discovery.robots())) or "none scanned yet"
            raise ApiError(404, f"no blueprints for robot {robot} (robots: {known})")
        return answer

    @app.get(
        "/dimos/docs/custom-robot",
        response_model=models.CustomRobotDoc,
        **route_doc(
            "docs",
            "dimos's guide to adding a robot of your own: markdown and HTML, with absolute links",
            "Found in the checkout's docs/ by file name and title (adding a custom/new robot, arm or platform); "
            "links point at the docs site (mkdocs.yml site_url), other repo files at GitHub. `html` is rendered "
            "here with markdown-it (raw HTML in the markdown is escaped), so a page can show it as is; `markdown` is "
            "there for a page that renders it itself. 404 when the checkout has no such page.",
            errors=(404,),
            agent=True,
            answer="`{ title, markdown, html, source_path, url, others }`",
        ),
    )
    async def custom_robot_doc() -> dict[str, Any]:
        answer = await asyncio.to_thread(docs.custom_robot, s.dimos_dir)
        if answer is None:
            raise ApiError(404, f"no guide to adding a robot in {s.dimos_dir / 'docs'}")
        return answer

    @app.get(
        "/dimos/docs/links",
        response_model=models.DocLinks,
        **route_doc(
            "docs",
            "Links into dimos's published docs: configuring a robot, adding one, blueprints, modules, install",
            "Each link is the docs-site URL of the checkout's page by that name (docs/**/configuration.md, ...); "
            "null when there's no such page. No side effects.",
            agent=True,
            answer="`{ site, repo, configure_robot, custom_robot, blueprints, modules, installation, quickstart, "
            "cli }`",
        ),
    )
    async def doc_links() -> dict[str, Any]:
        return await asyncio.to_thread(docs.links, s.dimos_dir)

    @app.get(
        "/dimos/extras",
        response_model=models.ExtrasList,
        **route_doc(
            "extras",
            "dimos's optional extras (`dimos[sim]`, ...): which are installed, what's missing, a download-size hint",
            "Extras from the checkout's pyproject.toml (else the installed dimos's metadata); installed packages "
            "are asked of the checkout's python in a child process (cached 10 s). `download_bytes` is an upper-bound "
            "hint from uv.lock. No side effects.",
            agent=True,
            answer="`{ mode, python, extras: [{ name, installed, applicable, requires, includes, missing, "
            "download_bytes }] }`",
        ),
    )
    async def extras_list() -> dict[str, Any]:
        found = await probe()
        listed = await asyncio.to_thread(extras.status, s.dimos_dir, found)
        return {
            "mode": "checkout" if extras.is_checkout(s.dimos_dir) else "library",
            "python": found["python"],
            "extras": listed,
        }

    # the extras install Desktop is running for us (its shell session), until it ends
    installing: dict[str, str | None] = {"shell": None}

    @app.post(
        "/dimos/extras/install",
        response_model=models.ExtrasInstallStarted,
        **route_doc(
            "extras",
            "Install dimos extras through Desktop's shell tool (the user sees the commands and presses Run); "
            "discovery rescans when it ends",
            "Runs scripts/install.sh's command: `uv sync --locked --inexact --extra <x> ...` in a checkout (keeps "
            "every extra and group already installed), else `uv pip install --python <venv python> --torch-backend "
            "cpu|cu128 'dimos[<x>,...]==<version>'`; through Desktop, one such command per extra, run one at a time. "
            "Never sudo. When that builds the cyclonedds package "
            "from source (unitree-dds, dds: it has wheels for python 3.10 only), it first gets the CycloneDDS C "
            "library it builds against: $CYCLONEDDS_HOME, else `nix build <flake.lock's nixpkgs>#cyclonedds` (an "
            "out-link in the venv), else Homebrew's; none = 400 with `code: cyclonedds_missing`. With Desktop "
            "(`$DESKTOP_URL`) the commands go to its `POST /api/desktop/shell` for the asking app (default "
            "`launcher`): it shows them over that app, nothing runs until the user presses Run, a failure can be fixed "
            "in its terminal (by the user or Desktop's agent) and retried; the answer is `{ shell }`, the session to "
            "follow with Desktop's `GET /api/desktop/shell/{id}?wait=`. The gateway waits for it and rescans. Without "
            "Desktop it runs as a job (`{ job }`: follow `<ns>/dimos/jobs/<job>` or GET /dimos/jobs/{job}/log). "
            "400 for an unknown extra; 409 while another install runs; 500 when uv isn't found.",
            errors=(400, 409, 500),
            agent=True,
            answer="`{ shell, job, command }` (one of shell / job)",
        ),
    )
    async def install_extras(request: models.ExtrasInstall) -> dict[str, Any]:
        declared = extras.declared(s.dimos_dir, await probe())
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
        probed = await probe()
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
                "dimOS installs these optional packages into its Python environment with uv (no sudo). It can "
                "take a few minutes.",
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
        # no Desktop: a job of our own
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

    @app.get(
        "/dimos/jobs",
        response_model=models.JobList,
        **route_doc(
            "jobs",
            "The dimos gateway's jobs (extras installs): running, and finished in the last 30 minutes",
            "No side effects.",
            answer="`{ jobs: [{ job, title, kind, done, ok, started_at, finished_at }] }`",
        ),
    )
    async def job_list() -> dict[str, Any]:
        return {"jobs": jobs.listing()}

    @app.get(
        "/dimos/jobs/{job}/log",
        response_model=models.JobLog,
        **route_doc(
            "jobs",
            "A job's output so far, and how it ended (`error`, and `failure`: its last lines)",
            "Lines from `after` on (default 0) and `next`, the `n` the next line will have: subscribe to "
            "`<ns>/dimos/jobs/<job>` first, then fetch this, then apply live lines with `n` >= `next`. 404 for an "
            "unknown (or expired) job.",
            errors=(404,),
            agent=True,
            answer="`{ job, title, kind, command, lines, next, done, ok, error, failure, started_at, finished_at }`",
        ),
    )
    async def job_log(
        job: JobParam,
        after: Annotated[int, Query(description="the first line's `n` to answer")] = 0,
    ) -> dict[str, Any]:
        try:
            return jobs.get(job).log(after)
        except KeyError as error:
            raise ApiError(404, error.args[0])

    @app.delete(
        "/dimos/jobs/{job}",
        response_model=models.JobLog,
        **route_doc(
            "jobs",
            "Cancel a running job",
            "Sends the job's process group SIGTERM; it ends with `ok: false`, `error: cancelled`. A finished job is "
            "answered as it is. 404 for an unknown job.",
            errors=(404,),
            answer="`JobLog`",
        ),
    )
    async def cancel_job(job: JobParam) -> dict[str, Any]:
        try:
            return jobs.cancel(job).log()
        except KeyError as error:
            raise ApiError(404, error.args[0])
